#!/usr/bin/env python3
"""
Box-Moving Robot - Raspberry Pi Master Controller
Coordinates navigation and manipulation via UART to two ESP32 controllers
"""

import cv2
import sqlite3
import serial
import time
import json
import threading
import queue
from pyzbar import pyzbar
import numpy as np
from datetime import datetime
import heapq
import math

class RobotMasterController:
    def __init__(self):
        # UART connections to ESP32 controllers
        self.nav_uart = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        self.arm_uart = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
        
        # Camera setup
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Database setup
        self.init_database()
        
        # Robot state
        self.current_position = (0, 0)  # Grid coordinates
        self.current_orientation = 0    # Degrees
        self.robot_busy = False
        self.obstacle_map = {}
        
        # Grid configuration (adjust to your environment)
        self.grid_size = 20  # 20x20 grid
        self.cell_size = 50  # 50mm per cell
        
        # Shelf positions (QR code -> grid coordinates)
        self.shelf_positions = {
            'SHELF_A': (18, 5),
            'SHELF_B': (18, 10),
            'SHELF_C': (18, 15),
            'HOME': (1, 1)
        }
        
        # Command queues
        self.nav_response_queue = queue.Queue()
        self.arm_response_queue = queue.Queue()
        
        # Start communication threads
        self.start_communication_threads()
        
        print("Robot Master Controller Initialized")
    
    def init_database(self):
        """Initialize SQLite database for tracking boxes and tasks"""
        self.db = sqlite3.connect('robot_tasks.db', check_same_thread=False)
        cursor = self.db.cursor()
        
        # Create tables
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS boxes (
                id TEXT PRIMARY KEY,
                status TEXT DEFAULT 'pending',
                source_position TEXT,
                destination_shelf TEXT,
                pickup_time TIMESTAMP,
                delivery_time TIMESTAMP,
                weight REAL,
                attempts INTEGER DEFAULT 0
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS robot_log (
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                event_type TEXT,
                description TEXT,
                position TEXT,
                success BOOLEAN
            )
        ''')
        
        self.db.commit()
    
    def log_event(self, event_type, description, success=True):
        """Log robot events to database"""
        cursor = self.db.cursor()
        cursor.execute('''
            INSERT INTO robot_log (event_type, description, position, success)
            VALUES (?, ?, ?, ?)
        ''', (event_type, description, str(self.current_position), success))
        self.db.commit()
    
    def start_communication_threads(self):
        """Start threads for UART communication"""
        nav_thread = threading.Thread(target=self.nav_communication_handler, daemon=True)
        arm_thread = threading.Thread(target=self.arm_communication_handler, daemon=True)
        nav_thread.start()
        arm_thread.start()
    
    def nav_communication_handler(self):
        """Handle navigation ESP32 communication"""
        while True:
            try:
                if self.nav_uart.in_waiting:
                    response = self.nav_uart.readline().decode().strip()
                    self.nav_response_queue.put(response)
            except Exception as e:
                print(f"Nav UART error: {e}")
            time.sleep(0.01)
    
    def arm_communication_handler(self):
        """Handle arm ESP32 communication"""
        while True:
            try:
                if self.arm_uart.in_waiting:
                    response = self.arm_uart.readline().decode().strip()
                    self.arm_response_queue.put(response)
            except Exception as e:
                print(f"Arm UART error: {e}")
            time.sleep(0.01)
    
    def send_nav_command(self, action, param1="", param2="", timeout=10):
        """Send command to navigation ESP32"""
        command = f"NAV:{action}:{param1}:{param2}\n"
        self.nav_uart.write(command.encode())
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                response = self.nav_response_queue.get(timeout=0.1)
                return response
            except queue.Empty:
                continue
        return "TIMEOUT"
    
    def send_arm_command(self, action, param1="", param2="", timeout=15):
        """Send command to arm ESP32"""
        command = f"ARM:{action}:{param1}:{param2}\n"
        self.arm_uart.write(command.encode())
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                response = self.arm_response_queue.get(timeout=0.1)
                return response
            except queue.Empty:
                continue
        return "TIMEOUT"
    
    def scan_qr_codes(self):
        """Scan for QR codes in camera view"""
        ret, frame = self.camera.read()
        if not ret:
            return []
        
        # Decode QR codes
        qr_codes = pyzbar.decode(frame)
        detected_codes = []
        
        for qr in qr_codes:
            data = qr.data.decode('utf-8')
            # Get QR code center position in image
            x = qr.rect.left + qr.rect.width // 2
            y = qr.rect.top + qr.rect.height // 2
            
            detected_codes.append({
                'data': data,
                'position': (x, y),
                'type': self.classify_qr_code(data)
            })
        
        return detected_codes
    
    def classify_qr_code(self, data):
        """Classify QR code type based on data"""
        if data.startswith('BOX_'):
            return 'box'
        elif data.startswith('SHELF_'):
            return 'shelf'
        elif data.startswith('FLOOR_'):
            return 'floor_marker'
        else:
            return 'unknown'
    
    def parse_box_qr(self, qr_data):
        """Parse box QR code data"""
        # Format: BOX_ID123_SHELF_A_WEIGHT_2.5
        parts = qr_data.split('_')
        if len(parts) >= 4:
            box_id = parts[1]
            destination = f"SHELF_{parts[3]}"
            weight = float(parts[5]) if len(parts) > 5 else 1.0
            return {'id': box_id, 'destination': destination, 'weight': weight}
        return None
    
    def a_star_pathfinding(self, start, goal):
        """A* pathfinding algorithm with obstacle avoidance"""
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        def get_neighbors(pos):
            x, y = pos
            neighbors = []
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if (nx, ny) not in self.obstacle_map:
                        neighbors.append((nx, ny))
            return neighbors
        
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            for neighbor in get_neighbors(current):
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found
    
    def navigate_to_position(self, target_pos):
        """Navigate to target position using A* pathfinding"""
        print(f"Navigating from {self.current_position} to {target_pos}")
        
        # Plan path
        path = self.a_star_pathfinding(self.current_position, target_pos)
        if not path:
            print("No path found!")
            return False
        
        # Execute path
        for i in range(1, len(path)):
            current = path[i-1]
            next_pos = path[i]
            
            # Calculate movement direction
            dx = next_pos[0] - current[0]
            dy = next_pos[1] - current[1]
            
            # Convert to robot commands
            if dx == 1 and dy == 0:
                direction = "FORWARD"
                target_angle = 0
            elif dx == -1 and dy == 0:
                direction = "FORWARD"
                target_angle = 180
            elif dx == 0 and dy == 1:
                direction = "FORWARD"
                target_angle = 90
            elif dx == 0 and dy == -1:
                direction = "FORWARD"
                target_angle = 270
            else:
                # Diagonal movement
                direction = "FORWARD"
                target_angle = math.degrees(math.atan2(dy, dx))
            
            # Turn to correct orientation
            angle_diff = target_angle - self.current_orientation
            if abs(angle_diff) > 5:  # 5 degree tolerance
                if angle_diff > 180:
                    angle_diff -= 360
                elif angle_diff < -180:
                    angle_diff += 360
                
                turn_direction = "LEFT" if angle_diff > 0 else "RIGHT"
                response = self.send_nav_command("TURN", turn_direction, str(abs(angle_diff)))
                
                if "SUCCESS" in response:
                    self.current_orientation = target_angle
                else:
                    print(f"Turn failed: {response}")
                    return False
            
            # Move forward
            distance = self.cell_size
            response = self.send_nav_command("MOVE", "FORWARD", str(distance))
            
            if "SUCCESS" in response:
                self.current_position = next_pos
                print(f"Moved to {self.current_position}")
            else:
                print(f"Movement failed: {response}")
                return False
            
            # Check for obstacles during movement
            obstacle_data = self.send_nav_command("SCAN")
            if "OBSTACLE" in obstacle_data:
                print("Obstacle detected, replanning...")
                self.update_obstacle_map(obstacle_data)
                return self.navigate_to_position(target_pos)  # Replan
        
        return True
    
    def update_obstacle_map(self, sensor_data):
        """Update obstacle map based on ultrasonic sensor data"""
        # Parse sensor data format: "FRONT:25,LEFT:15,RIGHT:30"
        sensors = sensor_data.split(',')
        obstacle_threshold = 20  # 20cm
        
        for sensor in sensors:
            if ':' in sensor:
                direction, distance = sensor.split(':')
                if float(distance) < obstacle_threshold:
                    # Calculate obstacle position relative to robot
                    obstacle_pos = self.calculate_obstacle_position(direction, float(distance))
                    if obstacle_pos:
                        self.obstacle_map[obstacle_pos] = time.time()
    
    def calculate_obstacle_position(self, direction, distance):
        """Calculate obstacle grid position based on sensor data"""
        x, y = self.current_position
        
        if direction == "FRONT":
            if self.current_orientation == 0:
                return (x + 1, y)
            elif self.current_orientation == 90:
                return (x, y + 1)
            elif self.current_orientation == 180:
                return (x - 1, y)
            elif self.current_orientation == 270:
                return (x, y - 1)
        # Add LEFT and RIGHT calculations similarly
        
        return None
    
    def execute_pickup_sequence(self, box_info):
        """Execute complete box pickup sequence"""
        print(f"Executing pickup for box {box_info['id']}")
        
        # Position robot for optimal pickup angle
        response = self.send_nav_command("POSITION", "PICKUP")
        if "SUCCESS" not in response:
            return False
        
        # Initialize arm to home position
        response = self.send_arm_command("HOME")
        if "SUCCESS" not in response:
            print("Arm home position failed")
            return False
        
        # Calculate pickup coordinates (adjust based on QR position)
        pickup_x = 150  # mm from robot center
        pickup_y = 0    # mm from robot center
        
        # Execute pickup sequence
        response = self.send_arm_command("PICK", str(pickup_x), str(pickup_y))
        if "SUCCESS" in response:
            # Confirm grip
            grip_status = self.send_arm_command("GRIP", "STATUS")
            if "GRIPPED" in grip_status:
                print("Box successfully picked up")
                
                # Move to transport position
                self.send_arm_command("TRANSPORT")
                
                # Update database
                cursor = self.db.cursor()
                cursor.execute('''
                    UPDATE boxes SET status = 'picked', pickup_time = ?
                    WHERE id = ?
                ''', (datetime.now(), box_info['id']))
                self.db.commit()
                
                return True
        
        print("Pickup failed")
        return False
    
    def execute_delivery_sequence(self, box_info):
        """Execute complete box delivery sequence"""
        print(f"Executing delivery for box {box_info['id']}")
        
        # Navigate to destination shelf
        shelf_pos = self.shelf_positions.get(box_info['destination'])
        if not shelf_pos:
            print(f"Unknown shelf: {box_info['destination']}")
            return False
        
        if not self.navigate_to_position(shelf_pos):
            print("Navigation to shelf failed")
            return False
        
        # Position for placement
        response = self.send_nav_command("POSITION", "PLACE")
        if "SUCCESS" not in response:
            return False
        
        # Calculate placement coordinates
        place_x = 200  # mm from robot center
        place_y = 0    # mm from robot center
        
        # Execute placement
        response = self.send_arm_command("PLACE", str(place_x), str(place_y))
        if "SUCCESS" in response:
            # Open gripper
            self.send_arm_command("GRIP", "OPEN")
            
            # Return to home position
            self.send_arm_command("HOME")
            
            # Update database
            cursor = self.db.cursor()
            cursor.execute('''
                UPDATE boxes SET status = 'delivered', delivery_time = ?
                WHERE id = ?
            ''', (datetime.now(), box_info['id']))
            self.db.commit()
            
            print(f"Box {box_info['id']} delivered successfully")
            return True
        
        print("Delivery failed")
        return False
    
    def process_box_task(self, box_qr_data):
        """Process complete box moving task"""
        box_info = self.parse_box_qr(box_qr_data)
        if not box_info:
            print("Invalid box QR code")
            return False
        
        # Add to database
        cursor = self.db.cursor()
        cursor.execute('''
            INSERT OR REPLACE INTO boxes 
            (id, destination_shelf, source_position, weight)
            VALUES (?, ?, ?, ?)
        ''', (box_info['id'], box_info['destination'], 
              str(self.current_position), box_info['weight']))
        self.db.commit()
        
        # Execute pickup
        if self.execute_pickup_sequence(box_info):
            # Execute delivery
            if self.execute_delivery_sequence(box_info):
                self.log_event("TASK_COMPLETE", f"Successfully moved box {box_info['id']}")
                return True
            else:
                self.log_event("DELIVERY_FAILED", f"Failed to deliver box {box_info['id']}", False)
        else:
            self.log_event("PICKUP_FAILED", f"Failed to pick up box {box_info['id']}", False)
        
        return False
    
    def main_loop(self):
        """Main robot operation loop"""
        print("Starting main robot loop...")
        
        while True:
            try:
                if self.robot_busy:
                    time.sleep(1)
                    continue
                
                # Scan for QR codes
                qr_codes = self.scan_qr_codes()
                
                for qr in qr_codes:
                    if qr['type'] == 'box':
                        print(f"Found box QR: {qr['data']}")
                        self.robot_busy = True
                        
                        success = self.process_box_task(qr['data'])
                        
                        if success:
                            print("Task completed successfully")
                        else:
                            print("Task failed")
                        
                        # Return to home position
                        home_pos = self.shelf_positions['HOME']
                        self.navigate_to_position(home_pos)
                        
                        self.robot_busy = False
                        break
                    
                    elif qr['type'] == 'floor_marker':
                        # Update position based on floor marker
                        self.update_position_from_qr(qr['data'])
                
                time.sleep(0.5)  # Main loop delay
                
            except KeyboardInterrupt:
                print("Shutting down robot...")
                break
            except Exception as e:
                print(f"Error in main loop: {e}")
                self.robot_busy = False
                time.sleep(1)
    
    def update_position_from_qr(self, floor_qr):
        """Update robot position based on floor QR marker"""
        # Format: FLOOR_X10_Y15
        if floor_qr.startswith('FLOOR_'):
            parts = floor_qr.split('_')
            if len(parts) >= 3:
                x = int(parts[1][1:])  # Remove 'X'
                y = int(parts[2][1:])  # Remove 'Y'
                self.current_position = (x, y)
                print(f"Position updated to {self.current_position}")
    
    def emergency_stop(self):
        """Emergency stop all robot operations"""
        self.send_nav_command("STOP")
        self.send_arm_command("STOP")
        self.robot_busy = False
        print("EMERGENCY STOP ACTIVATED")
    
    def get_status_report(self):
        """Get comprehensive robot status report"""
        cursor = self.db.cursor()
        
        # Get pending tasks
        cursor.execute("SELECT COUNT(*) FROM boxes WHERE status = 'pending'")
        pending_tasks = cursor.fetchone()[0]
        
        # Get completed tasks
        cursor.execute("SELECT COUNT(*) FROM boxes WHERE status = 'delivered'")
        completed_tasks = cursor.fetchone()[0]
        
        # Get recent logs
        cursor.execute('''
            SELECT * FROM robot_log 
            ORDER BY timestamp DESC LIMIT 5
        ''')
        recent_logs = cursor.fetchall()
        
        status = {
            'current_position': self.current_position,
            'current_orientation': self.current_orientation,
            'robot_busy': self.robot_busy,
            'pending_tasks': pending_tasks,
            'completed_tasks': completed_tasks,
            'obstacle_count': len(self.obstacle_map),
            'recent_logs': recent_logs
        }
        
        return status

if __name__ == "__main__":
    try:
        robot = RobotMasterController()
        
        # Start main operation loop
        robot.main_loop()
        
    except Exception as e:
        print(f"Failed to start robot: {e}")
    finally:
        # Cleanup
        if 'robot' in locals():
            robot.camera.release()
            robot.nav_uart.close()
            robot.arm_uart.close()
            robot.db.close()