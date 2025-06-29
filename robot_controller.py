#!/usr/bin/env python3
"""
Robot Controller with QR Code Integration
Combines QR code scanning with robotic arm control and AMR navigation
"""

from flask import Flask, render_template_string, request, redirect, url_for, Response, jsonify
import serial
import time
import cv2
import sqlite3
import threading
import qrcode
from qr_code_generator import RobotQRGenerator
import os
import numpy as np
from PIL import Image
import json

# Configuration
ARM_SERIAL_PORT = '/dev/ttyACM1'  # Primary port for arm ESP32
ARM_SERIAL_PORT_ALT = '/dev/ttyACM0'  # Alternative port for arm ESP32
NAV_SERIAL_PORT = '/dev/ttyUSB1'  # Primary port for navigation ESP32
NAV_SERIAL_PORT_ALT = '/dev/ttyUSB0'  # Alternative port for navigation ESP32
BAUDRATE = 9600
DB_PATH = 'robot_tasks.db'
# Camera configuration - can be overridden with environment variables
CAMERA_DEVICE = os.environ.get('CAMERA_DEVICE', '0')  # Default to camera index 0
# Standard box dimensions in cm
BOX_WIDTH = 6.5
BOX_HEIGHT = 3.5
BOX_DEPTH = 9.0
# QR code size in cm (assuming it covers one face of the box)
QR_CODE_REAL_WIDTH = 6.5  # cm
QR_CODE_REAL_HEIGHT = 3.5  # cm
# Focal length will be calibrated during runtime
FOCAL_LENGTH = None
# Known distance for calibration (in cm)
KNOWN_DISTANCE = 20.0  # cm
# Known width in pixels at the known distance (will be calibrated)
KNOWN_WIDTH_PIXELS = None

# Commands for the ESP32 arm
ARM_COMMANDS = {
    'Enable Arm': 'z',
    'Disable Arm': 'x',
    'Base +': 'e',  # Correct direction for base
    'Base -': 'q',  # Correct direction for base
    'Shoulder +': 'd',  # Changed from 'a' to 'd' to match correct direction
    'Shoulder -': 'a',  # Changed from 'd' to 'a' to match correct direction
    'Elbow +': 's',  # Changed from 'w' to 's' to match correct direction
    'Elbow -': 'w',  # Changed from 's' to 'w' to match correct direction
    'Gripper Open': 'i',
    'Gripper Close': 'o'
}

# Commands for the ESP32 navigation
NAV_COMMANDS = {
    'Forward': 'w',
    'Backward': 's',
    'Left': 'a',
    'Right': 'd',
    'Rotate CW': 'q',
    'Rotate CCW': 'e',
    'Stop': 'c',
    'Enable Motion': 'z',
    'Disable Motion': 'x'
}

# Change the SEQUENCES declaration to be an empty dictionary initially
SEQUENCES = {}

app = Flask(__name__)

# Initialize serial connection to ESP32 for arm control
arm_serial_available = False
try:
    print(f"Trying to connect to arm ESP32 on {ARM_SERIAL_PORT}...")
    arm_ser = serial.Serial(ARM_SERIAL_PORT, BAUDRATE, timeout=1)
    # Disable DTR and RTS to prevent ESP32 from resetting
    arm_ser.setDTR(False)
    arm_ser.setRTS(False)
    time.sleep(0.5)  # Short pause to stabilize connection
    arm_serial_available = True
    print(f"Connected to arm ESP32 on {ARM_SERIAL_PORT}")
except Exception as e:
    print(f"Could not open serial port {ARM_SERIAL_PORT}: {e}")
    try:
        print(f"Trying alternative port {ARM_SERIAL_PORT_ALT} for arm ESP32...")
        arm_ser = serial.Serial(ARM_SERIAL_PORT_ALT, BAUDRATE, timeout=1)
        # Disable DTR and RTS to prevent ESP32 from resetting
        arm_ser.setDTR(False)
        arm_ser.setRTS(False)
        time.sleep(0.5)  # Short pause to stabilize connection
        arm_serial_available = True
        print(f"Connected to arm ESP32 on {ARM_SERIAL_PORT_ALT}")
    except Exception as e2:
        print(f"Could not open alternative serial port {ARM_SERIAL_PORT_ALT}: {e2}")
        print("Arm ESP32 connection failed on both ports")
        arm_serial_available = False

# Initialize serial connection to ESP32 for navigation
nav_serial_available = False
try:
    print(f"Trying to connect to navigation ESP32 on {NAV_SERIAL_PORT}...")
    nav_ser = serial.Serial(NAV_SERIAL_PORT, BAUDRATE, timeout=1)
    # Disable DTR and RTS to prevent ESP32 from resetting
    nav_ser.setDTR(False)
    nav_ser.setRTS(False)
    time.sleep(0.5)  # Short pause to stabilize connection
    nav_serial_available = True
    print(f"Connected to navigation ESP32 on {NAV_SERIAL_PORT}")
except Exception as e:
    print(f"Could not open serial port {NAV_SERIAL_PORT}: {e}")
    try:
        print(f"Trying alternative port {NAV_SERIAL_PORT_ALT} for navigation ESP32...")
        nav_ser = serial.Serial(NAV_SERIAL_PORT_ALT, BAUDRATE, timeout=1)
        # Disable DTR and RTS to prevent ESP32 from resetting
        nav_ser.setDTR(False)
        nav_ser.setRTS(False)
        time.sleep(0.5)  # Short pause to stabilize connection
        nav_serial_available = True
        print(f"Connected to navigation ESP32 on {NAV_SERIAL_PORT_ALT}")
    except Exception as e2:
        print(f"Could not open alternative serial port {NAV_SERIAL_PORT_ALT}: {e2}")
        print("Navigation ESP32 connection failed on both ports")
        nav_serial_available = False

# Shared variable for latest IR correction value
latest_correction = "N/A"

# Initialize QR code generator
qr_generator = RobotQRGenerator(db_path=DB_PATH)

# Camera capture for QR code scanning
camera = None
camera_thread = None
camera_running = False
last_qr_data = None
last_qr_bbox = None  # To store bounding box coordinates

# Robot state tracking
robot_state = {
    "status": "idle",        # idle, moving, picking, placing, navigating
    "carrying_box": None,    # ID of box being carried, or None
    "battery": 100,          # Battery percentage
    "position": "home",      # Current position identifier
    "last_action": "",       # Last action performed
    "error": None,           # Any error state
    "nav_status": "stopped", # Navigation status: stopped, moving, etc.
    "ir_correction": "N/A",  # IR correction value from navigation ESP32
    "target_shelf": None,    # Target shelf for navigation
    "back_position": None    # Current back position being used
}

def serial_reader():
    """Thread function to continuously read from serial ports"""
    global latest_correction
    if not nav_serial_available or not arm_serial_available:
        return
        
    while True:
        try:
            # Read from arm ESP32
            if arm_ser.in_waiting:
                try:
                    line = arm_ser.readline().decode(errors='ignore').strip()
                    if line.startswith("Sensor States:"):
                        try:
                            # Extract IR values
                            ir_values = []
                            parts = line.split()
                            for part in parts:
                                if part.startswith("IR") and ":" in part:
                                    value = int(part.split(":")[1])
                                    ir_values.append(value)
                            
                            if len(ir_values) == 5:  # Ensure we have all 5 IR sensor values
                                # Calculate correction value directly
                                # Using weighted average: -2, -1, 0, 1, 2 from left to right
                                weights = [-2, -1, 0, 1, 2]
                                active_count = sum(ir_values)
                                
                                if active_count == 0:
                                    # No line detected - send 0 as correction
                                    correction_value = 0
                                else:
                                    # Calculate weighted position
                                    position_sum = sum(weights[i] * ir_values[i] for i in range(5))
                                    correction_value = int(position_sum / active_count)
                                
                                # Send correction value with 'C' prefix that navigation ESP32 understands
                                correction_cmd = f"C{correction_value}\n"
                                nav_ser.write(correction_cmd.encode())
                                print(f"Calculated correction: {correction_value} from IR: {ir_values}")
                                
                                # Update latest correction for UI
                                latest_correction = str(correction_value)
                                robot_state["ir_correction"] = latest_correction
                        except Exception as e:
                            print(f"Error parsing IR data: {e}")
                except Exception as e:
                    print(f"Error reading from arm serial: {e}")
                
            # Read from navigation ESP32
            if nav_ser.in_waiting:
                try:
                    line = nav_ser.readline().decode(errors='ignore').strip()
                    if line.startswith("CORRECTION:"):
                        latest_correction = line.split(":", 1)[1].strip()
                        robot_state["ir_correction"] = latest_correction
                        print(f"Received IR correction: {latest_correction}")  # Add debug print
                except Exception as e:
                    print(f"Error reading from navigation serial: {e}")
                    
        except Exception as e:
            print(f"Serial reader error: {e}")
            time.sleep(1)  # Prevent tight loop in case of errors
        
        time.sleep(0.01)  # Small delay to prevent CPU overuse

# Start background thread to read serial for IR correction
if nav_serial_available:
    nav_thread = threading.Thread(target=serial_reader, daemon=True)
    nav_thread.start()

def get_camera():
    """Initialize camera if not already done"""
    global camera
    print("Attempting to get camera...")
    
    if camera is not None:
        # Test if existing camera is still working
        try:
            print("Testing existing camera connection...")
            ret, _ = camera.read()
            if ret:
                print("Existing camera connection is working")
                return camera
            else:
                print("Existing camera connection failed, attempting to reconnect...")
                release_camera()
        except Exception as e:
            print(f"Error with existing camera: {e}")
            release_camera()
    
    # List of possible camera devices to try
    # On Raspberry Pi, USB cameras typically show up as /dev/video0, /dev/video1, etc.
    camera_devices = [
        '/dev/video0',
        '/dev/video1',
        '/dev/video2',
        0,  # Also try numeric indices as fallback
        1,
        2
    ]
    
    cam = None
    for device in camera_devices:
        try:
            print(f"Trying camera device: {device}")
            cam = cv2.VideoCapture(device)
            
            if not cam.isOpened():
                print(f"Failed to open camera at {device}")
                continue
                
            print(f"Camera opened at {device}, testing connection...")
            # Test multiple frames to ensure stable connection
            stable = True
            for i in range(3):
                ret, test_frame = cam.read()
                if not ret or test_frame is None:
                    print(f"Failed to read test frame {i+1}/3")
                    stable = False
                    break
                else:
                    print(f"Successfully read test frame {i+1}/3")
                time.sleep(0.1)
            
            if stable:
                camera = cam
                # Try to set resolution, but don't fail if not supported
                print("Setting camera resolution...")
                
                try:
                    # First try to get the current resolution
                    current_width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
                    current_height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
                    print(f"Current camera resolution: {current_width}x{current_height}")
                    
                    # Try to set to 640x480 if different
                    if current_width != 640 or current_height != 480:
                        print("Attempting to set resolution to 640x480...")
                        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        
                        # Verify the new resolution
                        actual_width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
                        actual_height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
                        print(f"New camera resolution: {actual_width}x{actual_height}")
                except Exception as e:
                    print(f"Warning: Could not set camera resolution: {e}")
                
                # Try to set some additional properties for better performance
                try:
                    # Set autofocus off (if supported)
                    camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                    # Set focus to infinity (if supported)
                    camera.set(cv2.CAP_PROP_FOCUS, 0)
                    # Try to disable auto exposure (if supported)
                    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
                except Exception as e:
                    print(f"Note: Some camera properties not supported: {e}")
                
                print(f"Successfully connected to camera at {device}")
                return camera
            else:
                print(f"Camera at {device} connection not stable")
                try:
                    if cam:
                        cam.release()
                except Exception as e:
                    print(f"Warning: Error releasing camera: {e}")
        except Exception as e:
            print(f"Error with camera at {device}: {e}")
            try:
                if cam:
                    cam.release()
            except Exception as release_error:
                print(f"Warning: Error releasing camera: {release_error}")
    
    print("Error: Could not find a working camera. Please check USB connection and permissions.")
    print("On Raspberry Pi, you may need to:")
    print("1. Check if the camera is properly connected (try unplugging and plugging back in)")
    print("2. Verify camera permissions (ls -l /dev/video*)")
    print("3. Add your user to the 'video' group (sudo usermod -a -G video $USER)")
    camera = None
    return camera

def release_camera():
    """Release camera resources"""
    global camera, camera_running
    camera_running = False
    if camera:
        camera.release()
        camera = None

def calculate_distance(pixel_width):
    """Calculate distance to QR code based on apparent size"""
    global FOCAL_LENGTH, KNOWN_WIDTH_PIXELS
    
    # If focal length is not calibrated yet, use a default approximation
    if FOCAL_LENGTH is None:
        # Approximate focal length based on camera resolution
        # This is a rough estimate and should be calibrated properly
        FOCAL_LENGTH = 650  # Default value for 640x480 resolution
        print(f"Using default focal length: {FOCAL_LENGTH}")
    
    # Calculate distance using the formula: distance = (real_width * focal_length) / pixel_width
    distance = (QR_CODE_REAL_WIDTH * FOCAL_LENGTH) / pixel_width
    return distance

def calibrate_camera(pixel_width, known_distance=KNOWN_DISTANCE):
    """Calibrate camera focal length using a known distance and QR code size"""
    global FOCAL_LENGTH, KNOWN_WIDTH_PIXELS
    
    # Calculate focal length using the formula: F = (P * D) / W
    # Where F is focal length, P is pixel width, D is known distance, W is real width
    FOCAL_LENGTH = (pixel_width * known_distance) / QR_CODE_REAL_WIDTH
    KNOWN_WIDTH_PIXELS = pixel_width
    
    print(f"Camera calibrated: Focal length = {FOCAL_LENGTH}, Known width in pixels = {KNOWN_WIDTH_PIXELS}")
    return FOCAL_LENGTH

def scan_qr_codes():
    """Thread function to continuously scan for QR codes"""
    global camera_running, last_qr_data, last_qr_bbox
    
    # Initialize camera
    camera = get_camera()
    if not camera:
        print("Failed to initialize camera for QR scanning")
        return
    
    qr_detector = cv2.QRCodeDetector()
    camera_running = True
    last_frame_time = time.time()
    frame_interval = 1.0 / 30  # Target 30 FPS
    consecutive_failures = 0
    max_failures = 5  # Maximum number of consecutive failures before reconnecting
    
    while camera_running:
        current_time = time.time()
        if current_time - last_frame_time < frame_interval:
            time.sleep(0.001)  # Small sleep to prevent CPU overuse
            continue
            
        # Read frame
        try:
            ret, frame = camera.read()
            if not ret or frame is None:
                consecutive_failures += 1
                print(f"Failed to read frame ({consecutive_failures}/{max_failures})")
                if consecutive_failures >= max_failures:
                    print("Too many consecutive failures, attempting to reconnect camera...")
                    release_camera()
                    camera = get_camera()
                    if not camera:
                        print("Failed to reconnect camera, stopping QR scanning")
                        camera_running = False
                        break
                    consecutive_failures = 0
                continue
            
            # Reset failure counter on successful frame read
            consecutive_failures = 0
            last_frame_time = current_time
                
            # Try to detect QR code
            try:
                data, bbox, _ = qr_detector.detectAndDecode(frame)
                if data:
                    print(f"QR Code detected: {data}")
                    # Only update if data is different or significant time has passed
                    if data != last_qr_data or (current_time - last_frame_time > 1.0):
                        last_qr_data = data
                        # Process the QR code data in the main thread
                        processing_thread = threading.Thread(target=process_qr_code, args=(data,))
                        processing_thread.daemon = True
                        processing_thread.start()
                
                # Update bounding box information if QR code is detected
                if bbox is not None and len(bbox) > 0:
                    points = bbox[0]
                    if len(points) == 4:
                        x_coords = [p[0] for p in points]
                        y_coords = [p[1] for p in points]
                        x = min(x_coords)
                        y = min(y_coords)
                        width = max(x_coords) - x
                        height = max(y_coords) - y
                        
                        distance = calculate_distance(width)
                        
                        last_qr_bbox = {
                            "x": int(x),
                            "y": int(y),
                            "width": int(width),
                            "height": int(height),
                            "distance": round(distance, 2)
                        }
                
            except Exception as e:
                print(f"QR detection error: {e}")
                # Don't count QR detection errors as camera failures
                time.sleep(0.1)  # Add small delay on error
                
        except Exception as e:
            print(f"Camera error: {e}")
            consecutive_failures += 1
            if consecutive_failures >= max_failures:
                print("Too many consecutive failures, attempting to reconnect camera...")
                release_camera()
                camera = get_camera()
                if not camera:
                    print("Failed to reconnect camera, stopping QR scanning")
                    camera_running = False
                    break
                consecutive_failures = 0
            time.sleep(0.1)  # Add small delay on error

def select_sequence_for_box(box_info):
    """Select appropriate sequence based on box properties"""
    # Always use the standard sequence regardless of box properties
    return 'standard_sequence'

def update_robot_state(status=None, box_id=None, position=None, action=None, error=None, **kwargs):
    """Update the robot's current state"""
    global robot_state
    
    if status:
        robot_state["status"] = status
    if box_id is not None:  # Could be None to clear the box
        robot_state["carrying_box"] = box_id
    if position:
        robot_state["position"] = position
    if action:
        robot_state["last_action"] = action
    if error is not None:  # Could be None to clear the error
        robot_state["error"] = error
    
    # Handle additional state parameters
    if "target_shelf" in kwargs:
        robot_state["target_shelf"] = kwargs["target_shelf"]
    if "back_position" in kwargs:
        robot_state["back_position"] = kwargs["back_position"]
    if "nav_status" in kwargs:
        robot_state["nav_status"] = kwargs["nav_status"]
    
    # Simulate battery usage
    if status in ["moving", "picking", "placing", "navigating"]:
        robot_state["battery"] = max(0, robot_state["battery"] - 0.5)
    
    print(f"Robot state updated: {robot_state}")

def process_qr_code(qr_data):
    """Process QR code data and trigger appropriate actions"""
    try:
        print(f"Processing QR code: {qr_data}")
        
        # If the robot is already busy, don't process another QR code
        if robot_state["status"] != "idle":
            print(f"Robot is busy ({robot_state['status']}), ignoring QR code")
            return
            
        if qr_data.startswith('BOX_') or qr_data == "BOX_TEST":
            # Extract box information
            if qr_data == "BOX_TEST":
                box_id = "TEST"
                # Create a dummy box_info for testing
                box_info = ("TEST", "pending", "source", "SHELF_A", "1", None, None, 1.0, 0, None)
            else:
                parts = qr_data.split('_')
                box_id = parts[1]
                
                # Connect to database to get box information
                try:
                    conn = sqlite3.connect(DB_PATH)
                    cursor = conn.cursor()
                    cursor.execute("SELECT * FROM boxes WHERE id = ?", (box_id,))
                    box_info = cursor.fetchone()
                    conn.close()
                except Exception as db_error:
                    print(f"Database error: {db_error}")
                    # Create a dummy box_info if database fails
                    box_info = (box_id, "pending", "source", "SHELF_A", "1", None, None, 1.0, 0, None)
            
            # Look for custom pickup sequence first, fall back to default if not found
            pickup_sequence = 'custom_pickup' if 'custom_pickup' in SEQUENCES else 'pick_and_store_back'
            
            # Step 1: Pick up the box and move to back
            update_robot_state(status="picking", action=f"Picking up box: {box_id}")
            execute_sequence(pickup_sequence)
            
            # Determine which back position to use based on available slots
            # For now, using a simple round-robin approach
            back_position = (int(box_id) % 3) + 1 if box_id.isdigit() else 1
            
            # Look for custom store sequence first, fall back to default if not found
            custom_store_sequence = f'custom_store_{back_position}'
            back_sequence = custom_store_sequence if custom_store_sequence in SEQUENCES else f'store_back_position_{back_position}'
            
            # Store the box at the selected back position
            update_robot_state(status="storing", action=f"Storing box at back position {back_position}")
            execute_sequence(back_sequence)
            
            # Look for custom home sequence first, fall back to default if not found
            home_sequence = 'custom_home' if 'custom_home' in SEQUENCES else 'return_to_home'
            
            # Return arm to home position
            update_robot_state(status="homing", action="Returning to home position")
            execute_sequence(home_sequence)
            
            # Add a small delay to ensure the arm is fully settled
            print("Waiting for arm to settle before starting navigation...")
            time.sleep(1.0)
            
            # Step 2: Navigate to destination
            destination = box_info[3] if box_info and len(box_info) > 3 else "SHELF_A"
            shelf_letter = destination[-1].upper() if destination.startswith("SHELF_") else "A"
            
            # Start navigation to shelf
            navigate_to_shelf(destination, back_position, box_id)
            
        elif qr_data.startswith('SHELF_'):
            print(f"Detected shelf: {qr_data}")
            # Update current position
            update_robot_state(position=qr_data)
            
            # If we're in navigation mode, check if this is our target
            if robot_state["status"] == "navigating":
                target_shelf = robot_state.get("target_shelf")
                if target_shelf and qr_data == target_shelf:
                    print(f"Reached target shelf: {qr_data}")
                    # Stop navigation
                    send_nav_command("Stop")
                    send_nav_command("Disable Motion")
                    
                    # Continue with box placement
                    complete_shelf_placement()
                else:
                    # Not our target shelf, adjust navigation
                    adjust_navigation(qr_data, target_shelf)
            
        elif qr_data.startswith('FLOOR_'):
            print(f"Detected floor marker: {qr_data}")
            update_robot_state(position=qr_data)
            # Use floor markers for additional navigation reference
            if robot_state["status"] == "navigating":
                adjust_navigation(qr_data, robot_state.get("target_shelf"))
        
        else:
            print(f"Unknown QR code format: {qr_data}")
            update_robot_state(error=f"Unknown QR code format: {qr_data}")
            
    except Exception as e:
        print(f"Error processing QR code: {str(e)}")
        update_robot_state(error=f"QR processing error: {str(e)}")
        # Reset to idle state after error
        update_robot_state(status="idle")

def continuous_line_monitor():
    """Continuously monitor IR values and make small adjustments to keep on the line.
    This function is designed to run in a separate thread."""
    print("Starting continuous line monitor")
    
    # Pattern detection for shelf (without relying on QR)
    consecutive_zero_readings = 0
    max_consecutive_zeros = 5  # Threshold to identify a shelf
    last_non_zero_reading = None
    
    # Shelf counting variables
    shelves_passed = 0
    shelf_detection_cooldown = 0  # Cooldown to prevent multiple detections of the same shelf
    cooldown_period = 20  # Number of iterations before allowing another shelf detection
    shelf_mapping = {0: "SHELF_A", 1: "SHELF_B", 2: "SHELF_C"}  # Map count to shelf names
    
    # Only run while navigation is active
    while robot_state["status"] == "navigating":
        try:
            # Get current IR correction value
            ir_correction = robot_state.get("ir_correction", "0")
            
            # Update cooldown counter if active
            if shelf_detection_cooldown > 0:
                shelf_detection_cooldown -= 1
            
            # Special handling for all IR sensors reporting zero (completely lost)
            if ir_correction == "0" or ir_correction == "0.0":
                # Increment consecutive zero counter
                consecutive_zero_readings += 1
                print(f"All IR sensors zero - count: {consecutive_zero_readings}/{max_consecutive_zeros}")
                
                # If we've reached our threshold, consider it a shelf detection
                if consecutive_zero_readings >= max_consecutive_zeros and shelf_detection_cooldown == 0:
                    print("Pattern detected: Multiple consecutive zero readings - likely at shelf")
                    
                    # Increment shelf count
                    shelves_passed += 1
                    print(f"Shelf detected! Total shelves passed: {shelves_passed}")
                    
                    # Set cooldown to prevent multiple detections of the same shelf
                    shelf_detection_cooldown = cooldown_period
                    
                    # Get the target shelf from state
                    target_shelf = robot_state.get("target_shelf")
                    
                    # Determine current shelf based on count
                    if shelves_passed <= 3:  # We only have 3 shelves (A, B, C)
                        current_shelf = shelf_mapping[shelves_passed - 1]
                        update_robot_state(action=f"Detected {current_shelf} (shelf #{shelves_passed})")
                        
                        # Check if we've reached our target shelf
                        if target_shelf and current_shelf == target_shelf:
                            print(f"Reached target shelf: {target_shelf}")
                            update_robot_state(position=target_shelf)
                            
                            # Stop movement
                            send_nav_command("Stop")
                            time.sleep(0.5)
                            
                            # Stop navigation motors
                            send_nav_command("Disable Motion")
                            
                            # Continue with box placement
                            complete_shelf_placement()
                            return  # Exit the line monitor as navigation is complete
                        else:
                            # Not at target shelf yet, continue moving
                            print(f"Passed {current_shelf}, continuing to {target_shelf}")
                            send_nav_command("Forward")
                            consecutive_zero_readings = 0  # Reset counter to continue navigation
                    else:
                        # We've passed more shelves than expected - something is wrong
                        print("Error: Passed more shelves than expected (A, B, C)")
                        send_nav_command("Stop")
                        update_robot_state(
                            error="Navigation error: Passed more shelves than expected",
                            action="Stopped due to navigation error"
                        )
                        return  # Exit the monitor
                
                # Standard rotation pattern for finding the line
                elif consecutive_zero_readings == 1:
                    # On first zero, try moving forward a bit
                    send_nav_command("Forward")
                    time.sleep(0.5)
                    send_nav_command("Stop")
                    time.sleep(0.2)
                elif consecutive_zero_readings == 3:
                    # After a few zeros, start rotating to search
                    # Rotate in direction of last known line position
                    if last_non_zero_reading and last_non_zero_reading > 0:
                        send_nav_command("Rotate CW")
                    else:
                        send_nav_command("Rotate CCW")
                    time.sleep(1.0)
                    send_nav_command("Stop")
            else:
                # Reset counter when we get a non-zero reading
                consecutive_zero_readings = 0
                
                # Normal IR correction handling
                try:
                    # Convert correction value to int instead of float
                    correction_value = int(ir_correction)
                    line_detected = True
                    
                    # Store the last non-zero reading
                    if correction_value != 0:
                        last_non_zero_reading = correction_value
                except ValueError:
                    # If we can't convert to int, it might be "N/A" or some other error value
                    correction_value = 0
                    line_detected = False
            
            # Define thresholds for correction
            small_threshold = 1    # Small deviation
            medium_threshold = 2   # Medium deviation
            large_threshold = 3    # Large deviation
            lost_threshold = 4     # Completely lost the line
            
            # Check if we've completely lost the line
            if not line_detected or abs(correction_value) >= lost_threshold:
                # We've lost the line completely - initiate recovery
                print(f"Line lost! Correction value: {ir_correction}. Starting recovery procedure.")
                update_robot_state(action="Line lost - initiating recovery")
                
                # Stop movement
                send_nav_command("Stop")
                time.sleep(0.5)
                
                # Back up slightly to try to find the line again
                send_nav_command("Backward")
                time.sleep(1.0)
                send_nav_command("Stop")
                time.sleep(0.5)
                
                # Check if we found the line by backing up
                ir_correction = robot_state.get("ir_correction", "0")
                try:
                    correction_value = int(ir_correction)
                    if abs(correction_value) < lost_threshold:
                        print(f"Line found after backing up. Correction: {correction_value}")
                        # Found the line, align and continue
                        adjust_navigation(None, robot_state.get("target_shelf"))
                        send_nav_command("Forward")
                        continue
                except ValueError:
                    pass
                
                # If we're still here, we didn't find the line by backing up
                # Try a search pattern - first rotate clockwise
                print("Searching for line by rotating CW...")
                send_nav_command("Rotate CW")
                
                # Search for up to 5 seconds
                search_start_time = time.time()
                found_during_search = False
                
                while time.time() - search_start_time < 5.0:
                    # Check every 0.5 seconds
                    time.sleep(0.5)
                    
                    # Check if we found the line
                    ir_correction = robot_state.get("ir_correction", "0")
                    try:
                        correction_value = int(ir_correction)
                        if abs(correction_value) < lost_threshold:
                            found_during_search = True
                            print(f"Line found during CW rotation! Correction: {correction_value}")
                            break
                    except ValueError:
                        pass
                
                # Stop rotation
                send_nav_command("Stop")
                time.sleep(0.5)
                
                # If we didn't find the line, try counter-clockwise
                if not found_during_search:
                    print("Line not found in CW rotation. Trying CCW...")
                    send_nav_command("Rotate CCW")
                    
                    # Search for up to 8 seconds (longer to cover more area)
                    search_start_time = time.time()
                    while time.time() - search_start_time < 8.0:
                        # Check every 0.5 seconds
                        time.sleep(0.5)
                        
                        # Check if we found the line
                        ir_correction = robot_state.get("ir_correction", "0")
                        try:
                            correction_value = int(ir_correction)
                            if abs(correction_value) < lost_threshold:
                                found_during_search = True
                                print(f"Line found during CCW rotation! Correction: {correction_value}")
                                break
                        except ValueError:
                            pass
                    
                    # Stop rotation
                    send_nav_command("Stop")
                    time.sleep(0.5)
                
                # If we found the line during search, align and continue
                if found_during_search:
                    print("Line found during search. Realigning...")
                    adjust_navigation(None, robot_state.get("target_shelf"))
                    send_nav_command("Forward")
                    update_robot_state(action="Line recovered - resuming navigation")
                else:
                    # If we still couldn't find the line, report failure
                    print("Could not recover line after extensive search")
                    update_robot_state(
                        error="Line lost and could not be recovered",
                        action="Navigation failed - line lost"
                    )
                    # Don't stop navigation completely - let the user decide what to do
                    # The robot will remain stopped until further commands
            
            # Make proportional adjustments based on deviation
            elif abs(correction_value) >= large_threshold:
                # Large deviation - stop and perform full realignment
                print(f"Large deviation detected: {correction_value}. Performing full realignment.")
                # Use the adjust_navigation function for full realignment
                adjust_navigation(None, robot_state.get("target_shelf"))
                
            elif abs(correction_value) >= medium_threshold:
                # Medium deviation - make a correction without stopping
                if correction_value > 0:
                    # Too far right, adjust left
                    print(f"Medium right deviation: {correction_value}. Adjusting left.")
                    send_nav_command("Left")
                    time.sleep(0.2)  # Brief correction
                    send_nav_command("Forward")  # Resume forward
                else:
                    # Too far left, adjust right
                    print(f"Medium left deviation: {correction_value}. Adjusting right.")
                    send_nav_command("Right")
                    time.sleep(0.2)  # Brief correction
                    send_nav_command("Forward")  # Resume forward
                    
            elif abs(correction_value) >= small_threshold:
                # Small deviation - make a very brief correction
                if correction_value > 0:
                    # Slightly right, minor left adjustment
                    print(f"Small right deviation: {correction_value}. Minor left adjustment.")
                    send_nav_command("Left")
                    time.sleep(0.1)  # Very brief correction
                    send_nav_command("Forward")  # Resume forward
                else:
                    # Slightly left, minor right adjustment
                    print(f"Small left deviation: {correction_value}. Minor right adjustment.")
                    send_nav_command("Right")
                    time.sleep(0.1)  # Very brief correction
                    send_nav_command("Forward")  # Resume forward
            
        except Exception as e:
            print(f"Error in continuous line monitor: {e}")
            time.sleep(1.0)  # Longer delay on error
        
        # Sleep to prevent tight loop and allow other processes to run
        time.sleep(0.5)
    
    print("Continuous line monitor stopped")

def navigate_to_shelf(target_shelf, back_position, box_id):
    """Handle navigation to target shelf with continuous feedback"""
    print(f"Starting navigation to {target_shelf}")
    
    # Convert target shelf name to a shelf index for our counting system
    target_shelf_letter = target_shelf[-1].upper() if target_shelf and len(target_shelf) > 0 else "A"
    shelf_indices = {"A": "SHELF_A", "B": "SHELF_B", "C": "SHELF_C"}
    
    if target_shelf_letter not in shelf_indices:
        print(f"Invalid shelf letter: {target_shelf_letter}. Defaulting to Shelf A")
        target_shelf = "SHELF_A"
    else:
        # Ensure target_shelf has the correct format for our shelf mapping
        target_shelf = shelf_indices[target_shelf_letter]
    
    # Update robot state with navigation target
    update_robot_state(
        status="navigating",
        action=f"Moving to {target_shelf}",
        target_shelf=target_shelf,
        back_position=back_position,
        carrying_box=box_id,
        nav_status="moving"
    )
    
    try:
        # Enable navigation with more explicit logging
        print("Enabling navigation motors...")
        response = send_nav_command("Enable Motion")
        print(f"Navigation enable response: {response}")
        
        # Longer delay after enabling motion
        time.sleep(1.0)
        
        # Check if we're already on a line by checking IR correction value
        ir_correction = robot_state.get("ir_correction", "0")
        try:
            correction_value = float(ir_correction)
            line_detected = abs(correction_value) < 10  # Assuming large values mean no line detected
        except ValueError:
            correction_value = 0
            line_detected = False
            
        if not line_detected:
            # Implement a search pattern to find the line
            print("No line detected. Starting search pattern...")
            update_robot_state(action="Searching for line")
            
            # First try moving forward a bit to find the line
            send_nav_command("Forward")
            time.sleep(1.0)
            send_nav_command("Stop")
            time.sleep(0.5)
            
            # Check if we found the line
            ir_correction = robot_state.get("ir_correction", "0")
            try:
                correction_value = float(ir_correction)
                line_detected = abs(correction_value) < 10
            except ValueError:
                line_detected = False
                
            # If still not found, try rotating to find the line
            if not line_detected:
                print("Line not found. Rotating to search...")
                
                # Try rotating clockwise first
                send_nav_command("Rotate CW")
                
                # Search for up to 5 seconds
                search_start_time = time.time()
                while time.time() - search_start_time < 5.0:
                    # Check every 0.5 seconds
                    time.sleep(0.5)
                    
                    # Check if we found the line
                    ir_correction = robot_state.get("ir_correction", "0")
                    try:
                        correction_value = float(ir_correction)
                        if abs(correction_value) < 10:
                            line_detected = True
                            print(f"Line found during rotation! Correction: {correction_value}")
                            break
                    except ValueError:
                        pass
                
                # Stop rotation
                send_nav_command("Stop")
                time.sleep(0.5)
                
                # If still not found, try the other direction
                if not line_detected:
                    print("Line not found in CW rotation. Trying CCW...")
                    
                    # Try rotating counter-clockwise
                    send_nav_command("Rotate CCW")
                    
                    # Search for up to 5 seconds
                    search_start_time = time.time()
                    while time.time() - search_start_time < 5.0:
                        # Check every 0.5 seconds
                        time.sleep(0.5)
                        
                        # Check if we found the line
                        ir_correction = robot_state.get("ir_correction", "0")
                        try:
                            correction_value = float(ir_correction)
                            if abs(correction_value) < 10:
                                line_detected = True
                                print(f"Line found during rotation! Correction: {correction_value}")
                                break
                        except ValueError:
                            pass
                    
                    # Stop rotation
                    send_nav_command("Stop")
                    time.sleep(0.5)
        
        # If we found the line or were already on it, start following it
        if line_detected:
            print("Line detected. Starting line following...")
            update_robot_state(action="Following line to target")
            
            # First align with the line
            adjust_navigation(None, target_shelf)
            
            # Start moving forward with explicit logging
            print("Starting forward movement...")
            response = send_nav_command("Forward")
            print(f"Forward command response: {response}")
            
            # Start continuous line monitoring in a separate thread
            monitor_thread = threading.Thread(target=continuous_line_monitor)
            monitor_thread.daemon = True
            monitor_thread.start()
            
        else:
            # If we couldn't find the line, notify and stop
            print("Could not find line after search pattern")
            send_nav_command("Stop")
            update_robot_state(
                error="Could not find line for navigation",
                action="Navigation failed - no line detected"
            )
            
        # Navigation will continue until a shelf QR code is detected
        # The process_qr_code function will handle detection and stopping
    except Exception as e:
        print(f"Navigation error: {e}")
        # Stop navigation on error
        send_nav_command("Stop")
        send_nav_command("Disable Motion")
        update_robot_state(
            status="idle",
            error=f"Navigation error: {str(e)}",
            nav_status="stopped",
            target_shelf=None
        )

def adjust_navigation(current_marker, target_shelf):
    """Adjust navigation based on detected markers and IR feedback"""
    if not target_shelf:
        print("No target shelf set, cannot adjust navigation")
        return
        
    try:
        # Get current IR correction value
        ir_correction = robot_state.get("ir_correction", "0")
        try:
            correction_value = int(ir_correction)
        except ValueError:
            correction_value = 0
        
        # Parse shelf letters for position comparison
        current_letter = current_marker[-1] if current_marker else None
        target_letter = target_shelf[-1] if target_shelf else None
        
        print(f"Adjusting navigation: Current={current_letter}, Target={target_letter}, IR={correction_value}")
        
        # Define the threshold for considering the line centered
        center_threshold = 1
        
        # Check if the line is significantly off-center
        if abs(correction_value) >= center_threshold:
            # Stop forward motion first
            send_nav_command("Stop")
            time.sleep(0.2)  # Short pause
            
            # Move backward a bit to reposition
            print("Moving backward to reposition")
            send_nav_command("Backward")
            update_robot_state(action=f"Moving backward to reposition (IR: {correction_value})")
            time.sleep(0.5)  # Move backward for half a second
            
            # Stop backward motion
            send_nav_command("Stop")
            time.sleep(0.2)  # Short pause
            
            # Rotate to align with the line
            if correction_value > 0:
                # Too far right, rotate counter-clockwise (left)
                print(f"Rotating CCW to center line (IR: {correction_value})")
                send_nav_command("Rotate CCW")
                update_robot_state(action=f"Rotating CCW to center line (IR: {correction_value})")
            elif correction_value < 0:
                # Too far left, rotate clockwise (right)
                print(f"Rotating CW to center line (IR: {correction_value})")
                send_nav_command("Rotate CW")
                update_robot_state(action=f"Rotating CW to center line (IR: {correction_value})")
            
            # Rotate for a duration proportional to the correction value
            rotation_time = min(abs(correction_value) * 0.2, 1.0)  # Limit to 1 second max
            time.sleep(rotation_time)
            
            # Stop rotation
            send_nav_command("Stop")
            time.sleep(0.2)  # Short pause
            
            # Resume forward motion if we're not at the target
            if current_marker != target_shelf:
                print("Resuming forward motion after alignment")
                send_nav_command("Forward")
                update_robot_state(action="Resuming forward after alignment")
        else:
            # Line is centered, proceed with normal navigation
            if current_letter and target_letter:
                if current_letter < target_letter:
                    # Need to go further
                    print("Target is ahead, moving forward")
                    send_nav_command("Forward")
                    update_robot_state(action="Moving forward to target")
                elif current_letter > target_letter:
                    # Went too far, need to back up
                    print("Passed target, moving backward")
                    send_nav_command("Backward")
                    update_robot_state(action="Moving backward to target")
                elif current_letter == target_letter:
                    # At the target, stop
                    print("At target shelf, stopping")
                    send_nav_command("Stop")
                    update_robot_state(action="Arrived at target shelf")
            else:
                # No marker information, just try to stay on the line
                send_nav_command("Forward")
                update_robot_state(action="Following line forward")
            
    except Exception as e:
        print(f"Navigation adjustment error: {e}")
        update_robot_state(error=f"Navigation adjustment error: {str(e)}")

def complete_shelf_placement():
    """Complete the box placement after reaching the target shelf"""
    # Get stored state information
    back_position = robot_state.get("back_position")
    target_shelf = robot_state.get("target_shelf")
    
    if not (back_position and target_shelf):
        print("Missing required state information for shelf placement")
        update_robot_state(
            error="Missing state information",
            status="idle",
            nav_status="stopped"
        )
        return
    
    # Extract the shelf letter from the target shelf name
    # Format could be either "SHELF_X" or just "X"
    if target_shelf.startswith("SHELF_") and len(target_shelf) > 6:
        shelf_letter = target_shelf[-1].lower()
    else:
        shelf_letter = target_shelf.lower()
    
    # Validate shelf letter
    if shelf_letter not in ['a', 'b', 'c']:
        print(f"Invalid shelf letter: {shelf_letter}. Defaulting to shelf A")
        shelf_letter = 'a'
    
    try:
        # Look for custom pick sequence first, fall back to default if not found
        custom_pick_sequence = f'custom_pick_back_{back_position}'
        pick_sequence = custom_pick_sequence if custom_pick_sequence in SEQUENCES else f'pick_from_back_{back_position}'
        
        # Pick up box from back position
        update_robot_state(
            status="retrieving",
            action=f"Retrieving box from back position {back_position}",
            nav_status="stopped"
        )
        execute_sequence(pick_sequence)
        
        # Look for custom placement sequence first, fall back to default if not found
        custom_place_sequence = f'custom_place_shelf_{shelf_letter}'
        place_sequence = custom_place_sequence if custom_place_sequence in SEQUENCES else f'place_on_shelf_{shelf_letter}'
        
        # Place on the correct shelf
        update_robot_state(
            status="placing",
            action=f"Placing box on shelf {shelf_letter.upper()}"
        )
        execute_sequence(place_sequence)
        
        # Reset state
        update_robot_state(
            status="idle",
            action="Completed box placement",
            target_shelf=None,
            back_position=None,
            carrying_box=None,
            nav_status="stopped"
        )
        
    except Exception as e:
        print(f"Error during shelf placement: {e}")
        update_robot_state(
            error=f"Placement error: {str(e)}",
            status="idle",
            nav_status="stopped"
        )

def send_command(cmd_label):
    """Send a command to the ESP32 arm controller"""
    if not arm_serial_available:
        print(f"ERROR: Arm serial port not available, can't send '{cmd_label}'")
        return "Arm serial port not available"
    
    cmd = ARM_COMMANDS.get(cmd_label)
    if not cmd:
        print(f"ERROR: Unknown command: '{cmd_label}'")
        return f"Unknown command: {cmd_label}"
    
    response = ""
    try:
        print(f"Sending command '{cmd_label}' (byte: '{cmd}')")
        arm_ser.write(cmd.encode())
        time.sleep(0.1)
        
        # Read response if available
        response_data = ""
        while arm_ser.in_waiting:
            response_data += arm_ser.readline().decode(errors='ignore')
            
        if response_data:
            print(f"Received response: {response_data.strip()}")
            response = response_data
        else:
            print("No response received")
            response = "No response"
    except Exception as e:
        error_msg = f"Error: {str(e)}"
        print(f"ERROR sending command '{cmd_label}': {error_msg}")
        response = error_msg
    
    return response

def send_nav_command(cmd_label):
    """Send a command to the ESP32 navigation controller"""
    if not nav_serial_available:
        print(f"ERROR: Navigation serial port not available, can't send '{cmd_label}'")
        return "Navigation serial port not available"
    
    cmd = NAV_COMMANDS.get(cmd_label)
    if not cmd:
        print(f"ERROR: Unknown navigation command: '{cmd_label}'")
        return f"Unknown command: {cmd_label}"
    
    response = ""
    try:
        print(f"Sending navigation command '{cmd_label}' (byte: '{cmd}')")
        nav_ser.write((cmd + '\n').encode())
        
        # Wait a bit longer for response
        time.sleep(0.2)
        
        # Read response if available
        response_data = ""
        while nav_ser.in_waiting:
            response_data += nav_ser.readline().decode(errors='ignore')
            
        if response_data:
            print(f"Received navigation response: {response_data.strip()}")
            response = response_data
        else:
            print("No navigation response received")
            response = "No response"
    except Exception as e:
        error_msg = f"Error: {str(e)}"
        print(f"ERROR sending navigation command '{cmd_label}': {error_msg}")
        response = error_msg
    
    return response

def send_correction(correction_value):
    """Send a correction value to the navigation controller"""
    if not nav_serial_available:
        return "Navigation serial port not available"
    
    response = ""
    try:
        nav_ser.write((str(correction_value) + '\n').encode())
        time.sleep(0.1)
        while nav_ser.in_waiting:
            response += nav_ser.readline().decode(errors='ignore')
    except Exception as e:
        response = f"Error: {str(e)}"
    
    return response

def execute_sequence(sequence_name):
    """Execute a predefined sequence of commands"""
    if sequence_name not in SEQUENCES:
        update_robot_state(error=f"Unknown sequence: {sequence_name}")
        return f"Unknown sequence: {sequence_name}"
    
    update_robot_state(status="executing", action=f"Starting sequence: {sequence_name}")
    
    try:
        sequence = SEQUENCES[sequence_name]
        print(f"Starting sequence '{sequence_name}' with {len(sequence)} commands")
        
        # Count commands by type for verification
        command_counts = {}
        for cmd, _ in sequence:
            command_counts[cmd] = command_counts.get(cmd, 0) + 1
        print(f"Command distribution: {command_counts}")
        
        # Execute each command in sequence
        for i, (cmd, delay) in enumerate(sequence):
            print(f"Executing command {i+1}/{len(sequence)}: {cmd} with delay {delay}")
            
            # Check if it's an arm or navigation command
            if cmd in NAV_COMMANDS:
                response = send_nav_command(cmd)
            else:
                response = send_command(cmd)
                
            if "Error" in response:
                update_robot_state(error=f"Command failed: {cmd} - {response}")
                print(f"ERROR: Command failed: {cmd} - {response}")
                return f"Sequence {sequence_name} failed: {response}"
            
            # Wait for the specified delay
            print(f"Waiting for {delay} seconds")
            time.sleep(delay)
            
            # Add a small pause between commands for stability
            if i < len(sequence) - 1:
                time.sleep(0.1)
        
        print(f"Sequence '{sequence_name}' completed successfully")
        update_robot_state(status="idle", action=f"Completed sequence: {sequence_name}")
        return f"Sequence {sequence_name} completed"
    except Exception as e:
        error_msg = f"Sequence error: {str(e)}"
        print(f"ERROR: {error_msg}")
        update_robot_state(error=error_msg)
        return f"Sequence {sequence_name} failed: {str(e)}"

def analyze_sequence(sequence_name):
    """Analyze a sequence structure and return debug information"""
    if sequence_name not in SEQUENCES:
        return {"error": f"Unknown sequence: {sequence_name}"}
    
    sequence = SEQUENCES[sequence_name]
    
    # Count commands by type
    command_counts = {}
    for cmd, _ in sequence:
        command_counts[cmd] = command_counts.get(cmd, 0) + 1
    
    # Check for symmetry in movement commands
    movement_pairs = {
        'Base +': 'Base -',
        'Shoulder +': 'Shoulder -',
        'Elbow +': 'Elbow -',
        'Gripper Open': 'Gripper Close'
    }
    
    symmetry_check = {}
    for cmd1, cmd2 in movement_pairs.items():
        count1 = command_counts.get(cmd1, 0)
        count2 = command_counts.get(cmd2, 0)
        symmetry_check[f"{cmd1}/{cmd2}"] = {
            "forward": count1,
            "reverse": count2,
            "balanced": count1 == count2,
            "difference": abs(count1 - count2)
        }
    
    # Analyze sequence structure
    total_steps = len(sequence)
    total_time = sum(delay for _, delay in sequence)
    
    # Check for repeated patterns using Python's list multiplication
    is_multiplied = False
    multiplication_factor = 0
    for pattern_length in range(1, len(sequence) // 2 + 1):
        for start in range(min(10, len(sequence) - pattern_length)):  # Check first few positions
            pattern = sequence[start:start+pattern_length]
            # Try to find if this pattern is repeated
            for factor in range(2, 100):  # Reasonable upper limit
                if start + pattern_length * factor > len(sequence):
                    break
                if sequence[start:start+pattern_length*factor] == pattern * factor:
                    is_multiplied = True
                    multiplication_factor = factor
                    break
            if is_multiplied:
                break
        if is_multiplied:
            break
    
    return {
        "name": sequence_name,
        "total_steps": total_steps,
        "total_time": total_time,
        "command_counts": command_counts,
        "symmetry_check": symmetry_check,
        "uses_multiplication": is_multiplied,
        "multiplication_factor": multiplication_factor if is_multiplied else 0
    }

# HTML templates
MAIN_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control System</title>
    <style>
        body { font-family: Arial; text-align: center; margin: 0; padding: 20px; }
        .container { max-width: 1000px; margin: 0 auto; }
        .mode-selector { margin-bottom: 30px; }
        button { padding: 10px 20px; margin: 5px; font-size: 16px; cursor: pointer; }
        .mode-button { width: 200px; height: 60px; font-size: 18px; }
        .active-mode { background-color: #4CAF50; color: white; }
        .help-button { background-color: #ffcc00; color: #333; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Control System</h1>
        
        <div class="mode-selector">
            <h2>Select Operation Mode</h2>
            <a href="/manual"><button class="mode-button {% if mode == 'manual' %}active-mode{% endif %}">Arm Control</button></a>
            <a href="/navigation"><button class="mode-button {% if mode == 'navigation' %}active-mode{% endif %}">Navigation Control</button></a>
            <a href="/auto"><button class="mode-button {% if mode == 'auto' %}active-mode{% endif %}">Automatic Mode</button></a>
            <a href="/manual_sequence"><button class="mode-button {% if mode == 'sequence' %}active-mode{% endif %}">Sequence Control</button></a>
            <a href="/sequence_recorder"><button class="mode-button {% if mode == 'recorder' %}active-mode{% endif %}">Sequence Recorder</button></a>
            <a href="/status"><button class="mode-button {% if mode == 'status' %}active-mode{% endif %}">Robot Status</button></a>
            <a href="/sequence_analysis"><button class="mode-button {% if mode == 'analysis' %}active-mode{% endif %}">Sequence Analysis</button></a>
            <a href="/debug_serial"><button class="mode-button {% if mode == 'debug' %}active-mode{% endif %}">Debug Serial</button></a>
            <a href="/sequence_help"><button class="mode-button help-button">Custom Sequence Help</button></a>
        </div>
        
        <div class="current-mode">
            <h3>Current Mode: {{ mode|capitalize }}</h3>
        </div>
    </div>
</body>
</html>
"""

MANUAL_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Manual Arm Control</title>
    <style>
        body { font-family: Arial; text-align: center; }
        button { width: 150px; height: 50px; margin: 10px; font-size: 18px; }
        .row { margin-bottom: 20px; }
        .back-button { background-color: #f0f0f0; }
    </style>
</head>
<body>
    <h1>ESP32 Arm Manual Control</h1>
    <a href="/"><button class="back-button">Back to Main Menu</button></a>
    
    <form method="post">
        <div class="row">
            <button name="cmd" value="Enable Arm">Enable Arm</button>
            <button name="cmd" value="Disable Arm">Disable Arm</button>
        </div>
        <div class="row">
            <button name="cmd" value="Base +">Base +</button>
            <button name="cmd" value="Base -">Base -</button>
        </div>
        <div class="row">
            <button name="cmd" value="Shoulder +">Shoulder +</button>
            <button name="cmd" value="Shoulder -">Shoulder -</button>
        </div>
        <div class="row">
            <button name="cmd" value="Elbow +">Elbow +</button>
            <button name="cmd" value="Elbow -">Elbow -</button>
        </div>
        <div class="row">
            <button name="cmd" value="Gripper Open">Gripper Open</button>
            <button name="cmd" value="Gripper Close">Gripper Close</button>
        </div>
    </form>
    
    {% if response %}
    <div>
        <h3>ESP32 Response:</h3>
        <pre>{{ response }}</pre>
    </div>
    {% endif %}
</body>
</html>
"""

NAV_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>AMR Control GUI</title>
    <style>
        body { font-family: Arial; text-align: center; }
        button { width: 120px; height: 40px; margin: 8px; font-size: 18px; }
        .row { margin-bottom: 16px; }
        input[type=number] { width: 80px; font-size: 18px; }
        .irbox { margin: 30px auto; background: #222; color: #0f0; padding: 12px 0; border-radius: 8px; width: 320px; font-size: 22px; }
        .back-button { background-color: #f0f0f0; }
    </style>
</head>
<body>
    <h1>AMR Motion Control</h1>
    <a href="/"><button class="back-button">Back to Main Menu</button></a>
    
    <form method="post">
        <div class="row">
            <button name="cmd" value="Enable Motion">Enable Motion</button>
            <button name="cmd" value="Disable Motion">Disable Motion</button>
        </div>
        <div class="row">
            <button name="cmd" value="Forward">Forward</button>
        </div>
        <div class="row">
            <button name="cmd" value="Left">Left</button>
            <button name="cmd" value="Stop">Stop</button>
            <button name="cmd" value="Right">Right</button>
        </div>
        <div class="row">
            <button name="cmd" value="Backward">Backward</button>
        </div>
        <div class="row">
            <button name="cmd" value="Rotate CW">Rotate CW</button>
            <button name="cmd" value="Rotate CCW">Rotate CCW</button>
        </div>
    </form>
    <form method="post" style="margin-top:30px;">
        <label>Correction value (e.g. -2, 3): </label>
        <input type="number" name="correction" required>
        <button type="submit" name="send_correction" value="1">Send Correction</button>
    </form>
    <div class="irbox">
        <b>Latest IR Correction:</b> {{ latest_correction }}
    </div>
    {% if response %}
    <div>
        <h3>ESP32 Response:</h3>
        <pre>{{ response }}</pre>
    </div>
    {% endif %}
</body>
</html>
"""

AUTO_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Automatic Mode</title>
    <style>
        body { font-family: Arial; text-align: center; }
        .container { display: flex; flex-direction: column; align-items: center; }
        .video-container { width: 640px; height: 480px; margin: 20px; border: 1px solid #ccc; position: relative; }
        .video-overlay { position: absolute; top: 0; left: 0; width: 640px; height: 480px; z-index: 10; pointer-events: none; }
        button { width: 200px; height: 50px; margin: 10px; font-size: 18px; }
        .back-button { background-color: #f0f0f0; }
        .control-panel { margin: 20px; }
        .qr-data { margin: 20px; padding: 10px; background-color: #f0f0f0; border-radius: 5px; }
        .camera-config { margin: 20px; padding: 10px; background-color: #f0f0f0; border-radius: 5px; }
        .detection-box { position: absolute; border: 3px solid #00ff00; display: none; }
        .detection-text { position: absolute; background-color: rgba(0, 255, 0, 0.7); color: white; padding: 5px; font-size: 14px; display: none; }
        .error { color: red; }
        .success { color: green; }
        #status { padding: 10px; margin: 10px; border-radius: 5px; }
        .loading { cursor: wait; opacity: 0.7; }
    </style>
    <script>
        let cameraStarted = false;
        
        function updateStatus(message, isError = false) {
            const statusElement = document.getElementById('status');
            statusElement.className = isError ? 'error' : 'success';
            statusElement.innerText = message;
        }
        
        function setLoading(loading) {
            document.body.className = loading ? 'loading' : '';
            const buttons = document.getElementsByTagName('button');
            for (let button of buttons) {
                button.disabled = loading;
            }
        }
        
        function startCamera() {
            setLoading(true);
            updateStatus('Starting camera...');
            
            fetch('/start_camera')
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        cameraStarted = true;
                        document.getElementById('camera_img').src = "/video_feed";
                        updateStatus(data.status);
                    } else {
                        updateStatus(data.status, true);
                    }
                })
                .catch(error => {
                    updateStatus('Error starting camera: ' + error, true);
                })
                .finally(() => {
                    setLoading(false);
                });
        }
        
        function stopCamera() {
            setLoading(true);
            updateStatus('Stopping camera...');
            
            fetch('/stop_camera')
                .then(response => response.json())
                .then(data => {
                    cameraStarted = false;
                    document.getElementById('camera_img').src = "";
                    updateStatus(data.status);
                    
                    // Clear QR detection
                    document.getElementById('qr_data').innerText = 'None';
                    document.getElementById('detection-box').style.display = 'none';
                    document.getElementById('detection-text').style.display = 'none';
                })
                .catch(error => {
                    updateStatus('Error stopping camera: ' + error, true);
                })
                .finally(() => {
                    setLoading(false);
                });
        }
        
        function setCamera() {
            const cameraDevice = document.getElementById('camera_device').value;
            if (!cameraDevice) {
                updateStatus('Please enter a camera device', true);
                return;
            }
            
            setLoading(true);
            updateStatus('Setting camera device...');
            
            fetch('/set_camera', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ camera_device: cameraDevice }),
            })
            .then(response => response.json())
            .then(data => {
                updateStatus(data.status);
                if (cameraStarted) {
                    // Restart camera if it was running
                    stopCamera().then(() => startCamera());
                }
            })
            .catch(error => {
                updateStatus('Error setting camera: ' + error, true);
            })
            .finally(() => {
                setLoading(false);
            });
        }
        
        function calibrateCamera() {
            const knownDistance = document.getElementById('known_distance').value;
            if (!knownDistance) {
                updateStatus('Please enter a known distance in cm', true);
                return;
            }
            
            setLoading(true);
            updateStatus('Calibrating camera...');
            
            fetch('/calibrate_camera', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ known_distance: parseFloat(knownDistance) }),
            })
            .then(response => response.json())
            .then(data => {
                updateStatus(data.status);
            })
            .catch(error => {
                updateStatus('Error calibrating camera: ' + error, true);
            })
            .finally(() => {
                setLoading(false);
            });
        }
        
        function checkQrStatus() {
            if (!cameraStarted) return;
            
            fetch('/qr_status')
                .then(response => response.json())
                .then(data => {
                    if (data.qr_data) {
                        document.getElementById('qr_data').innerText = data.qr_data;
                        
                        // Show detection box if coordinates are available
                        if (data.qr_bbox) {
                            const box = document.getElementById('detection-box');
                            box.style.left = data.qr_bbox.x + 'px';
                            box.style.top = data.qr_bbox.y + 'px';
                            box.style.width = data.qr_bbox.width + 'px';
                            box.style.height = data.qr_bbox.height + 'px';
                            box.style.display = 'block';
                            
                            const text = document.getElementById('detection-text');
                            text.style.left = data.qr_bbox.x + 'px';
                            text.style.top = (data.qr_bbox.y + data.qr_bbox.height + 5) + 'px';
                            text.innerText = data.qr_data;
                            text.style.display = 'block';
                        }
                    } else {
                        document.getElementById('qr_data').innerText = 'None';
                        document.getElementById('detection-box').style.display = 'none';
                        document.getElementById('detection-text').style.display = 'none';
                    }
                })
                .catch(error => {
                    console.error('Error checking QR status:', error);
                });
            
            setTimeout(checkQrStatus, 1000);
        }
        
        window.onload = function() {
            checkQrStatus();
        };
    </script>
</head>
<body>
    <h1>Automatic Box Handling Mode</h1>
    <a href="/"><button class="back-button">Back to Main Menu</button></a>
    
    <div class="container">
        <div class="video-container">
            <img id="camera_img" src="" width="640" height="480">
            <div class="video-overlay">
                <div id="detection-box" class="detection-box"></div>
                <div id="detection-text" class="detection-text"></div>
            </div>
        </div>
        
        <div class="control-panel">
            <button onclick="startCamera()">Start Camera</button>
            <button onclick="stopCamera()">Stop Camera</button>
            <div class="camera-config">
                <h3>Camera Configuration</h3>
                <input type="text" id="camera_device" placeholder="Camera device (e.g., 0, 1, /dev/video0)">
                <button onclick="setCamera()">Set Camera</button>
                
                <h3>Distance Calibration</h3>
                <input type="number" id="known_distance" placeholder="Known distance (cm)">
                <button onclick="calibrateCamera()">Calibrate</button>
            </div>
            <p id="status" class="success">Camera inactive</p>
        </div>
        
        <div class="qr-data">
            <h3>Last Detected QR Code:</h3>
            <p id="qr_data">None</p>
        </div>
    </div>
</body>
</html>
"""

def generate_camera_frames():
    """Generate frames from camera for streaming"""
    global camera_running
    
    # Initialize camera
    camera = get_camera()
    if not camera:
        print("Failed to initialize camera for video feed")
        return
        
    qr_detector = cv2.QRCodeDetector()
    consecutive_failures = 0
    max_failures = 5
    frame_interval = 1.0 / 30  # Target 30 FPS
    last_frame_time = time.time()
    
    while True:
        try:
            current_time = time.time()
            if current_time - last_frame_time < frame_interval:
                time.sleep(0.001)  # Small sleep to prevent CPU overuse
                continue
                
            success, frame = camera.read()
            if not success or frame is None:
                consecutive_failures += 1
                print(f"Failed to read frame for video feed ({consecutive_failures}/{max_failures})")
                if consecutive_failures >= max_failures:
                    print("Too many consecutive failures, attempting to reconnect camera...")
                    release_camera()
                    camera = get_camera()
                    if not camera:
                        print("Failed to reconnect camera, stopping video feed")
                        break
                    consecutive_failures = 0
                time.sleep(0.1)
                continue
            
            # Reset failure counter on successful frame read
            consecutive_failures = 0
            last_frame_time = current_time
            
            # Try to detect QR codes in the frame for visualization
            try:
                data, bbox, _ = qr_detector.detectAndDecode(frame)
                if data and bbox is not None and len(bbox) > 0:
                    points = bbox[0]
                    if len(points) == 4:
                        # Draw green polygon around QR code
                        try:
                            cv2.polylines(frame, [np.int32(points)], True, (0, 255, 0), 2)
                            
                            # Calculate distance
                            x_coords = [p[0] for p in points]
                            y_coords = [p[1] for p in points]
                            x = min(x_coords)
                            y = min(y_coords)
                            width = max(x_coords) - x
                            
                            distance = calculate_distance(width)
                            
                            # Add text label with distance
                            text = f"{data} - {round(distance, 2)}cm"
                            
                            # Draw background for text
                            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                            cv2.rectangle(frame, 
                                        (int(x), int(y) - text_size[1] - 10),
                                        (int(x) + text_size[0], int(y)),
                                        (0, 255, 0), -1)
                            
                            # Draw text
                            cv2.putText(frame, text, (int(x), int(y) - 5), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                        except Exception as e:
                            print(f"Error drawing QR visualization: {e}")
            except Exception as e:
                print(f"Error detecting QR code in video frame: {e}")
                
            # Convert frame to JPEG
            try:
                ret, jpeg = cv2.imencode('.jpg', frame)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
                else:
                    print("Failed to encode frame to JPEG")
            except Exception as e:
                print(f"Error encoding frame: {e}")
                
        except Exception as e:
            print(f"Camera error in video feed: {e}")
            consecutive_failures += 1
            if consecutive_failures >= max_failures:
                print("Too many consecutive failures, attempting to reconnect camera...")
                release_camera()
                camera = get_camera()
                if not camera:
                    print("Failed to reconnect camera, stopping video feed")
                    break
                consecutive_failures = 0
            time.sleep(0.1)

@app.route('/')
def index():
    """Main page with mode selection"""
    return render_template_string(MAIN_HTML, mode="selection")

@app.route('/manual', methods=['GET', 'POST'])
def manual_control():
    """Manual arm control page"""
    response = ""
    if request.method == 'POST':
        cmd_label = request.form['cmd']
        response = send_command(cmd_label)
    
    return render_template_string(MANUAL_HTML, response=response)

@app.route('/navigation', methods=['GET', 'POST'])
def navigation_control():
    """Navigation control page"""
    response = ""
    global latest_correction
    
    if request.method == 'POST':
        if 'cmd' in request.form:
            cmd_label = request.form['cmd']
            response = send_nav_command(cmd_label)
            # Update robot state based on command
            if cmd_label == 'Stop' or cmd_label == 'Disable Motion':
                robot_state["nav_status"] = "stopped"
            elif cmd_label == 'Enable Motion':
                robot_state["nav_status"] = "enabled"
            else:
                robot_state["nav_status"] = "moving"
                
        elif 'send_correction' in request.form:
            correction = request.form.get('correction')
            if correction:
                response = send_correction(correction)
    
    return render_template_string(NAV_HTML, response=response, latest_correction=latest_correction)

@app.route('/auto')
def auto_control():
    """Automatic control page with camera feed"""
    return render_template_string(AUTO_HTML)

@app.route('/start_camera')
def start_camera():
    """Start the camera and QR code scanning"""
    global camera_thread, camera_running
    
    try:
        if camera_running:
            return jsonify({"status": "Camera already running", "success": True})
        
        # Try to initialize camera first
        camera = get_camera()
        if not camera:
            return jsonify({
                "status": "Failed to initialize camera. Please check if camera is connected and try again.",
                "success": False
            })
        
        # Start camera in a separate thread
        camera_thread = threading.Thread(target=scan_qr_codes)
        camera_thread.daemon = True
        camera_thread.start()
        
        return jsonify({"status": "Camera started successfully", "success": True})
    except Exception as e:
        print(f"Error starting camera: {e}")
        return jsonify({
            "status": f"Error starting camera: {str(e)}",
            "success": False
        })

@app.route('/stop_camera')
def stop_camera():
    """Stop the camera and QR code scanning"""
    global camera_running
    
    if camera_running:
        camera_running = False
        if camera_thread:
            camera_thread.join(timeout=1.0)
        release_camera()
        return jsonify({"status": "Camera stopped"})
    else:
        return jsonify({"status": "Camera not running"})

@app.route('/video_feed')
def video_feed():
    """Video streaming route for the camera feed"""
    try:
        # Try to get camera first
        camera = get_camera()
        if not camera:
            # If camera initialization fails, return an error image
            error_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_img, "Camera not available", (160, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            _, jpeg = cv2.imencode('.jpg', error_img)
            return Response(b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n',
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        # If camera is available, return the video stream
        return Response(generate_camera_frames(),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    except Exception as e:
        print(f"Error in video feed route: {e}")
        return "Video feed error", 500

@app.route('/qr_status')
def qr_status():
    """Return the last detected QR code data"""
    distance_info = ""
    if last_qr_bbox and "distance" in last_qr_bbox:
        distance_info = f" (Distance: {last_qr_bbox['distance']} cm)"
    
    # Create a copy of last_qr_bbox with NumPy values converted to standard Python types
    bbox_json = None
    if last_qr_bbox:
        bbox_json = {}
        for key, value in last_qr_bbox.items():
            # Convert NumPy types to standard Python types
            if hasattr(value, "item"):  # Check if it's a NumPy type
                bbox_json[key] = value.item()
            else:
                bbox_json[key] = value
    
    return jsonify({
        "qr_data": last_qr_data + distance_info if last_qr_data else None,
        "qr_bbox": bbox_json
    })

@app.route('/set_camera', methods=['POST'])
def set_camera():
    """Set the camera device"""
    global CAMERA_DEVICE, camera, camera_running
    
    # Stop the camera if it's running
    if camera_running:
        camera_running = False
        if camera_thread:
            camera_thread.join(timeout=1.0)
        release_camera()
    
    # Get the new camera device from the request
    data = request.json
    new_camera_device = data.get('camera_device')
    
    if new_camera_device:
        CAMERA_DEVICE = new_camera_device
        print(f"Camera device set to: {CAMERA_DEVICE}")
        return jsonify({"status": f"Camera device set to: {CAMERA_DEVICE}"})
    else:
        return jsonify({"status": "No camera device specified"})

@app.route('/execute_sequence/<sequence_name>')
def api_execute_sequence(sequence_name):
    """API endpoint to execute a predefined sequence"""
    result = execute_sequence(sequence_name)
    return jsonify({"result": result})

@app.route('/force_sequence/<sequence_name>')
def force_sequence(sequence_name):
    """Force execution of a sequence for testing purposes"""
    if sequence_name not in SEQUENCES:
        return jsonify({"error": f"Unknown sequence: {sequence_name}"})
    
    # Execute the sequence
    result = execute_sequence(sequence_name)
    return jsonify({"result": result})

@app.route('/manual_sequence')
def manual_sequence_page():
    """Page for manually triggering automation sequences"""
    sequences_html = ""
    for seq_name in SEQUENCES.keys():
        sequences_html += f'<button onclick="runSequence(\'{seq_name}\')">{seq_name}</button><br>'
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Manual Sequence Execution</title>
        <style>
            body {{ font-family: Arial; text-align: center; }}
            button {{ width: 200px; height: 50px; margin: 10px; font-size: 18px; }}
            .back-button {{ background-color: #f0f0f0; }}
            .result {{ margin: 20px; padding: 10px; background-color: #f0f0f0; border-radius: 5px; }}
            .emergency {{ background-color: #ff6666; color: white; font-weight: bold; }}
        </style>
        <script>
            function runSequence(name) {{
                document.getElementById('status').innerText = "Running sequence: " + name;
                fetch('/execute_sequence/' + name)
                    .then(response => response.json())
                    .then(data => {{
                        document.getElementById('status').innerText = data.result;
                    }});
            }}
            
            function forceSequence(name) {{
                if (confirm("Are you sure you want to force execution of " + name + "?")) {{
                    document.getElementById('status').innerText = "Forcing sequence: " + name;
                    fetch('/force_sequence/' + name)
                        .then(response => response.json())
                        .then(data => {{
                            document.getElementById('status').innerText = data.result || data.error;
                        }});
                }}
            }}
        </script>
    </head>
    <body>
        <h1>Manual Sequence Execution</h1>
        <a href="/"><button class="back-button">Back to Main Menu</button></a>
        
        <div>
            <h2>Available Sequences</h2>
            {sequences_html}
        </div>
        
        <div>
            <h2>Force Execution (Emergency)</h2>
            <p>Use these buttons to force sequence execution regardless of robot state</p>
            <button class="emergency" onclick="forceSequence('standard_sequence')">Force Standard Sequence</button><br>
            <button class="emergency" onclick="forceSequence('home_position')">Force Home Position</button>
        </div>
        
        <div class="result">
            <h3>Status:</h3>
            <p id="status">Ready</p>
        </div>
    </body>
    </html>
    """
    return html

@app.route('/robot_status')
def robot_status():
    """Return the current robot status"""
    return jsonify(robot_state)

@app.route('/status')
def status_page():
    """Page showing robot status"""
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Status</title>
        <style>
            body {{ font-family: Arial; text-align: center; }}
            .container {{ max-width: 800px; margin: 0 auto; }}
            .status-panel {{ margin: 20px; padding: 20px; background-color: #f0f0f0; border-radius: 5px; text-align: left; }}
            .status-item {{ margin: 10px 0; }}
            .back-button {{ background-color: #f0f0f0; width: 150px; height: 40px; }}
            .error {{ color: red; }}
        </style>
        <script>
            function updateStatus() {{
                fetch('/robot_status')
                    .then(response => response.json())
                    .then(data => {{
                        document.getElementById('status').innerText = data.status;
                        document.getElementById('box').innerText = data.carrying_box || "None";
                        document.getElementById('battery').innerText = data.battery + "%";
                        document.getElementById('position').innerText = data.position;
                        document.getElementById('action').innerText = data.last_action;
                        document.getElementById('nav-status').innerText = data.nav_status;
                        document.getElementById('ir-correction').innerText = data.ir_correction;
                        
                        if (data.error) {{
                            document.getElementById('error').innerText = data.error;
                            document.getElementById('error').style.display = "block";
                        }} else {{
                            document.getElementById('error').style.display = "none";
                        }}
                        
                        // Update battery indicator color
                        const batteryLevel = document.getElementById('battery-level');
                        batteryLevel.style.width = data.battery + "%";
                        if (data.battery > 70) {{
                            batteryLevel.style.backgroundColor = "green";
                        }} else if (data.battery > 30) {{
                            batteryLevel.style.backgroundColor = "orange";
                        }} else {{
                            batteryLevel.style.backgroundColor = "red";
                        }}
                    }});
                setTimeout(updateStatus, 1000);
            }}
            
            window.onload = function() {{
                updateStatus();
            }};
        </script>
    </head>
    <body>
        <div class="container">
            <h1>Robot Status</h1>
            <a href="/"><button class="back-button">Back to Main Menu</button></a>
            
            <div class="status-panel">
                <div class="status-item">
                    <strong>Status:</strong> <span id="status">Loading...</span>
                </div>
                <div class="status-item">
                    <strong>Carrying Box:</strong> <span id="box">None</span>
                </div>
                <div class="status-item">
                    <strong>Battery:</strong> <span id="battery">100%</span>
                    <div style="width: 100%; background-color: #ddd; height: 20px; border-radius: 5px;">
                        <div id="battery-level" style="width: 100%; background-color: green; height: 20px; border-radius: 5px;"></div>
                    </div>
                </div>
                <div class="status-item">
                    <strong>Position:</strong> <span id="position">Unknown</span>
                </div>
                <div class="status-item">
                    <strong>Last Action:</strong> <span id="action">None</span>
                </div>
                <div class="status-item">
                    <strong>Navigation Status:</strong> <span id="nav-status">stopped</span>
                </div>
                <div class="status-item">
                    <strong>IR Correction:</strong> <span id="ir-correction">N/A</span>
                </div>
                <div class="status-item error" id="error" style="display: none;">
                    <strong>Error:</strong> <span id="error-message">None</span>
                </div>
            </div>
        </div>
    </body>
    </html>
    """
    return html

@app.route('/calibrate_camera', methods=['POST'])
def api_calibrate_camera():
    """API endpoint to manually calibrate camera with known distance"""
    data = request.json
    known_distance = data.get('known_distance', KNOWN_DISTANCE)
    
    if last_qr_bbox and "width" in last_qr_bbox:
        focal_length = calibrate_camera(last_qr_bbox["width"], known_distance)
        return jsonify({"status": "Camera calibrated", "focal_length": focal_length})
    else:
        return jsonify({"status": "No QR code detected for calibration"})

@app.route('/analyze_sequence/<sequence_name>')
def api_analyze_sequence(sequence_name):
    """API endpoint to analyze a sequence structure"""
    result = analyze_sequence(sequence_name)
    return jsonify(result)

@app.route('/sequence_analysis')
def sequence_analysis_page():
    """Page showing analysis of all sequences"""
    results = {}
    for seq_name in SEQUENCES.keys():
        results[seq_name] = analyze_sequence(seq_name)
    
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Sequence Analysis</title>
        <style>
            body { font-family: Arial; text-align: center; }
            .container { max-width: 1200px; margin: 0 auto; }
            .back-button { background-color: #f0f0f0; width: 150px; height: 40px; margin-bottom: 20px; }
            .sequence { margin: 20px 0; padding: 15px; background-color: #f5f5f5; border-radius: 8px; text-align: left; }
            .sequence h3 { margin-top: 0; }
            .stats { margin: 10px 0; }
            .symmetry { margin: 15px 0; }
            .balanced { color: green; }
            .unbalanced { color: red; font-weight: bold; }
            .details { margin-top: 10px; }
            .multiplication { margin-top: 10px; font-style: italic; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Sequence Analysis</h1>
            <a href="/"><button class="back-button">Back to Main Menu</button></a>
            
            <div id="sequences">
    """
    
    for seq_name, analysis in results.items():
        html += f"""
                <div class="sequence">
                    <h3>{seq_name}</h3>
                    <div class="stats">
                        <strong>Total Steps:</strong> {analysis['total_steps']}<br>
                        <strong>Total Time:</strong> {analysis['total_time']} seconds
                    </div>
                    
                    <div class="symmetry">
                        <strong>Movement Symmetry:</strong><br>
        """
        
        for pair, check in analysis['symmetry_check'].items():
            balance_class = "balanced" if check['balanced'] else "unbalanced"
            html += f"""
                        <div class="{balance_class}">
                            {pair}: {check['forward']} vs {check['reverse']} 
                            ({check['difference']} difference)
                        </div>
            """
        
        html += """
                    </div>
                    
                    <div class="details">
                        <strong>Command Distribution:</strong><br>
        """
        
        for cmd, count in analysis['command_counts'].items():
            html += f"                        {cmd}: {count}<br>"
        
        if analysis['uses_multiplication']:
            html += f"""
                    <div class="multiplication">
                        This sequence uses pattern multiplication (factor: {analysis['multiplication_factor']})
                    </div>
            """
        
        html += """
                    </div>
                </div>
        """
    
    html += """
            </div>
        </div>
    </body>
    </html>
    """
    
    return html

def cleanup():
    """Clean up resources before exit"""
    if camera_running:
        release_camera()
    if arm_serial_available:
        arm_ser.close()
    if nav_serial_available:
        nav_ser.close()
    qr_generator.close()

# Add this function after the other utility functions
def debug_serial_connections():
    """Check the status of serial connections and return diagnostic information"""
    arm_status = {
        "available": arm_serial_available,
        "port": ARM_SERIAL_PORT if arm_serial_available else "Not connected",
        "in_waiting": arm_ser.in_waiting if arm_serial_available else "N/A"
    }
    
    nav_status = {
        "available": nav_serial_available,
        "port": NAV_SERIAL_PORT if nav_serial_available else "Not connected",
        "in_waiting": nav_ser.in_waiting if nav_serial_available else "N/A"
    }
    
    # Try to send a test message to navigation ESP32 if available
    nav_test_result = "Not attempted"
    if nav_serial_available:
        try:
            print("Sending test message to navigation ESP32...")
            nav_ser.write(b"TEST\n")
            time.sleep(0.5)
            response = ""
            while nav_ser.in_waiting:
                response += nav_ser.readline().decode(errors='ignore')
            nav_test_result = f"Response: {response}" if response else "No response received"
        except Exception as e:
            nav_test_result = f"Error: {str(e)}"
    
    return {
        "arm_serial": arm_status,
        "nav_serial": nav_status,
        "latest_correction": latest_correction,
        "nav_test_result": nav_test_result
    }

# Add this route after the other routes
@app.route('/debug_serial')
def debug_serial_page():
    """Page showing debug information for serial connections"""
    debug_info = debug_serial_connections()
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Serial Connection Debug</title>
        <style>
            body {{ font-family: Arial; text-align: center; }}
            .container {{ max-width: 800px; margin: 0 auto; }}
            .debug-panel {{ margin: 20px; padding: 20px; background-color: #f0f0f0; border-radius: 5px; text-align: left; }}
            .status-item {{ margin: 10px 0; }}
            .back-button {{ background-color: #f0f0f0; width: 150px; height: 40px; }}
            .available {{ color: green; font-weight: bold; }}
            .unavailable {{ color: red; font-weight: bold; }}
            .test-button {{ margin: 20px; }}
        </style>
        <script>
            function refreshDebug() {{
                location.reload();
            }}
            
            function sendTestIR() {{
                fetch('/send_test_ir')
                    .then(response => response.json())
                    .then(data => {{
                        document.getElementById('test-result').innerText = data.result;
                    }});
            }}
        </script>
    </head>
    <body>
        <div class="container">
            <h1>Serial Connection Debug</h1>
            <a href="/"><button class="back-button">Back to Main Menu</button></a>
            
            <div class="debug-panel">
                <h2>Arm Serial Status</h2>
                <div class="status-item">
                    <strong>Available:</strong> 
                    <span class="{('available' if debug_info['arm_serial']['available'] else 'unavailable')}">
                        {debug_info['arm_serial']['available']}
                    </span>
                </div>
                <div class="status-item">
                    <strong>Port:</strong> {debug_info['arm_serial']['port']}
                </div>
                <div class="status-item">
                    <strong>Bytes in buffer:</strong> {debug_info['arm_serial']['in_waiting']}
                </div>
                
                <h2>Navigation Serial Status</h2>
                <div class="status-item">
                    <strong>Available:</strong> 
                    <span class="{('available' if debug_info['nav_serial']['available'] else 'unavailable')}">
                        {debug_info['nav_serial']['available']}
                    </span>
                </div>
                <div class="status-item">
                    <strong>Port:</strong> {debug_info['nav_serial']['port']}
                </div>
                <div class="status-item">
                    <strong>Bytes in buffer:</strong> {debug_info['nav_serial']['in_waiting']}
                </div>
                <div class="status-item">
                    <strong>Latest Correction:</strong> {debug_info['latest_correction']}
                </div>
                <div class="status-item">
                    <strong>Test Result:</strong> {debug_info['nav_test_result']}
                </div>
                
                <div class="test-button">
                    <button onclick="sendTestIR()">Send Test IR Signal</button>
                    <button onclick="refreshDebug()">Refresh</button>
                    <div id="test-result"></div>
                </div>
            </div>
        </div>
    </body>
    </html>
    """
    
    return html

@app.route('/send_test_ir')
def send_test_ir():
    """Send a test IR signal to the navigation ESP32"""
    result = "Not attempted"
    if nav_serial_available:
        try:
            # Send a test correction value with the 'C' prefix that navigation ESP32 understands
            test_correction = "C2\n"  # Positive 2 correction (move right)
            nav_ser.write(test_correction.encode())
            print(f"Sent test correction: {test_correction.strip()}")
            
            # Wait for response
            time.sleep(0.5)
            response = ""
            while nav_ser.in_waiting:
                response += nav_ser.readline().decode(errors='ignore')
            
            result = f"Response: {response}" if response else "No response received"
            
            # Also update the latest correction value for UI
            latest_correction = "2"
            robot_state["ir_correction"] = latest_correction
        except Exception as e:
            result = f"Error: {str(e)}"
    else:
        result = "Navigation serial port not available"
    
    return jsonify({"result": result})

# Add these global variables after the other globals
SEQUENCES_FILE = 'sequences.json'
recording_commands = []
is_recording = False

def save_sequences_to_file():
    """Save all sequences to a JSON file"""
    try:
        with open(SEQUENCES_FILE, 'w') as f:
            json.dump(SEQUENCES, f, indent=4)
        return True
    except Exception as e:
        print(f"Error saving sequences: {e}")
        return False

def load_sequences_from_file():
    """Load sequences from JSON file if it exists"""
    global SEQUENCES
    
    # Define default sequences that will be used if no JSON file exists
    default_sequences = {
        'pick_and_store_back': [
            ('Enable Arm', 1.0),     # Enable the arm first
            ('Elbow +', 0.5),         # First base movement 
            ('Elbow +', 0.5),         # Second base movement
            ('Elbow +', 0.5),  
            ('Gripper Open', 1.0), 
            ('Shoulder +', 0.5),     # Lower arm toward box
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     
            ('Shoulder +', 0.5),     # Complete lowering
            ('Gripper Close', 1.0),  # Grip box
            ('Shoulder -', 0.5),     # Lift box
            ('Shoulder -', 0.5),     
            ('Shoulder -', 0.5),     
            ('Shoulder -', 0.5),     
            ('Shoulder -', 0.5),     
            ('Shoulder -', 0.5),     
            ('Shoulder -', 0.5),     
            ('Shoulder -', 0.5),     
            ('Shoulder -', 0.5),     
            ('Shoulder -', 0.5),     # Complete lifting
            ('Elbow +', 0.5),         # Turn to back
            ('Elbow +', 0.5),         
            ('Elbow +', 0.5),         
            ('Elbow +', 0.5),         
            ('Elbow +', 0.5),         
            ('Elbow +', 0.5),         
            ('Elbow +', 0.5),         
            ('Elbow +', 0.5),         
            ('Elbow +', 0.5),  
            ('Elbow +', 0.5),
        ],

        'store_back_position_1': [  # Position 1 sequence
            # Move elbow to position 1 - exactly 38 steps down
        ] + [('Base +', 0.5) for _ in range(38)] + [
            ('Elbow -', 0.5),         # Reverse turn 1
            ('Elbow -', 0.5),         # Reverse turn 2
            ('Elbow -', 0.5),  
            # Release box
            ('Gripper Open', 1.0),   # Release box
            # Return elbow exactly 30 steps up
        ] + [('Base -', 0.5) for _ in range(38)],

        'store_back_position_2': [  # Position 2 sequence
            # Move elbow to position 2 - exactly 48 steps down
        ] + [('Base +', 0.5) for _ in range(48)] + [
            # Release box
            ('Gripper Open', 1.0),   # Release box
            # Return elbow exactly 40 steps up
        ] + [('Base -', 0.5) for _ in range(48)],

        'store_back_position_3': [  # Position 3 sequence
            # Move elbow to position 3 - exactly 55 steps down
        ] + [('Base +', 0.5) for _ in range(55)] + [
            # Release box
            ('Gripper Open', 1.0),   # Release box
            # Return elbow exactly 50 steps up
        ] + [('Base -', 0.5) for _ in range(55)],

        'return_to_home': [
            # Return base to center - exact reverse of turn to back
            ('Elbow -', 0.5),         # Reverse turn 1
            ('Elbow -', 0.5),         # Reverse turn 2
            ('Elbow -', 0.5),         # Reverse turn 3
            ('Elbow -', 0.5),         # Reverse turn 4
            ('Elbow -', 0.5),         # Reverse turn 5
            ('Elbow -', 0.5),         # Reverse turn 6
            ('Elbow -', 0.5),         # Reverse turn 7
            ('Elbow -', 0.5),         # Reverse turn 8
            ('Elbow -', 0.5),         # Reverse turn 9
            # Only disable the arm, not the entire robot
            ('Disable Arm', 0.5),    # Disable arm only, navigation can still work
        ],

        'pick_from_back_1': [  # Pick from first back position (30 iterations)
            ('Enable Arm', 1.0),
            # Turn to back - same as initial sequence
            ('Elbow +', 0.5),         # Turn 1
            ('Elbow +', 0.5),         # Turn 2
            ('Elbow +', 0.5),         # Turn 3
            ('Elbow +', 0.5),         # Turn 4
            ('Elbow +', 0.5),         # Turn 5
            ('Elbow +', 0.5),         # Turn 6
            ('Elbow +', 0.5),         # Turn 7
            ('Elbow +', 0.5),         # Turn 8
            ('Elbow +', 0.5),         # Turn 9
            # Move elbow to position 1 - exactly 30 steps
        ] + [('Base +', 0.5) for _ in range(30)] + [
            # Grip box
            ('Gripper Close', 1.0),  # Grip box
            # Return elbow - exactly 30 steps
        ] + [('Base -', 0.5) for _ in range(30)] + [
            # Return base to front - reverse of turn
            ('Elbow -', 0.5),         # Return turn 1
            ('Elbow -', 0.5),         # Return turn 2
            ('Elbow -', 0.5),         # Return turn 3
            ('Elbow -', 0.5),         # Return turn 4
            ('Elbow -', 0.5),         # Return turn 5
            ('Elbow -', 0.5),         # Return turn 6
            ('Elbow -', 0.5),         # Return turn 7
            ('Elbow -', 0.5),         # Return turn 8
            ('Elbow -', 0.5),         # Return turn 9
        ],

        'pick_from_back_2': [  # Pick from second back position (40 iterations)
            ('Enable Arm', 1.0),
            # Turn to back - same as initial sequence
            ('Elbow +', 0.5),         # Turn 1
            ('Elbow +', 0.5),         # Turn 2
            ('Elbow +', 0.5),         # Turn 3
            ('Elbow +', 0.5),         # Turn 4
            ('Elbow +', 0.5),         # Turn 5
            ('Elbow +', 0.5),         # Turn 6
            ('Elbow +', 0.5),         # Turn 7
            ('Elbow +', 0.5),         # Turn 8
            ('Elbow +', 0.5),         # Turn 9
            # Move elbow to position 2 - exactly 40 steps
        ] + [('Base +', 0.5) for _ in range(40)] + [
            # Grip box
            ('Gripper Close', 1.0),  # Grip box
            # Return elbow - exactly 40 steps
        ] + [('Base -', 0.5) for _ in range(40)] + [
            # Return base to front - reverse of turn
            ('Elbow -', 0.5),         # Return turn 1
            ('Elbow -', 0.5),         # Return turn 2
            ('Elbow -', 0.5),         # Return turn 3
            ('Elbow -', 0.5),         # Return turn 4
            ('Elbow -', 0.5),         # Return turn 5
            ('Elbow -', 0.5),         # Return turn 6
            ('Elbow -', 0.5),         # Return turn 7
            ('Elbow -', 0.5),         # Return turn 8
            ('Elbow -', 0.5),         # Return turn 9
        ],

        'pick_from_back_3': [  # Pick from third back position (50 iterations)
            ('Enable Arm', 1.0),
            # Turn to back - same as initial sequence
            ('Elbow +', 0.5),         # Turn 1
            ('Elbow +', 0.5),         # Turn 2
            ('Elbow +', 0.5),         # Turn 3
            ('Elbow +', 0.5),         # Turn 4
            ('Elbow +', 0.5),         # Turn 5
            ('Elbow +', 0.5),         # Turn 6
            ('Elbow +', 0.5),         # Turn 7
            ('Elbow +', 0.5),         # Turn 8
            ('Elbow +', 0.5),         # Turn 9
            # Move elbow to position 3 - exactly 50 steps
        ] + [('Base +', 0.5) for _ in range(50)] + [
            # Grip box
            ('Gripper Close', 1.0),  # Grip box
            # Return elbow - exactly 50 steps
        ] + [('Base -', 0.5) for _ in range(50)] + [
            # Return base to front - reverse of turn
            ('Elbow -', 0.5),         # Return turn 1
            ('Elbow -', 0.5),         # Return turn 2
            ('Elbow -', 0.5),         # Return turn 3
            ('Elbow -', 0.5),         # Return turn 4
            ('Elbow -', 0.5),         # Return turn 5
            ('Elbow -', 0.5),         # Return turn 6
            ('Elbow -', 0.5),         # Return turn 7
            ('Elbow -', 0.5),         # Return turn 8
            ('Elbow -', 0.5),         # Return turn 9
        ],

        'place_on_shelf_a': [
            # Move to shelf A
            ('Elbow -', 0.5),         # Turn to shelf A
            ('Elbow -', 0.5),
            ('Elbow -', 0.5),
            ('Shoulder +', 0.5),     # Position for shelf A
            ('Shoulder +', 0.5),
            ('Shoulder +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Gripper Open', 1.0),   # Release box
            # Return to neutral - exact reverse of positioning
            ('Base -', 0.5),        # Return elbow
            ('Base -', 0.5),
            ('Base -', 0.5),
            ('Shoulder -', 0.5),     # Return shoulder
            ('Shoulder -', 0.5),
            ('Shoulder -', 0.5),
            # Return base to center
            ('Elbow +', 0.5),         # Return base
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Disable Arm', 0.5),
        ],

        'place_on_shelf_b': [
            # Move to shelf B
            ('Elbow -', 0.5),         # Turn to shelf B
            ('Elbow -', 0.5),
            ('Elbow -', 0.5),
            ('Elbow -', 0.5),
            ('Shoulder +', 0.5),     # Position for shelf B
            ('Shoulder +', 0.5),
            ('Shoulder +', 0.5),
            ('Shoulder +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Gripper Open', 1.0),   # Release box
            # Return to neutral - exact reverse of positioning
            ('Base -', 0.5),        # Return elbow
            ('Base -', 0.5),
            ('Base -', 0.5),
            ('Base -', 0.5),
            ('Shoulder -', 0.5),     # Return shoulder
            ('Shoulder -', 0.5),
            ('Shoulder -', 0.5),
            ('Shoulder -', 0.5),
            # Return base to center
            ('Elbow +', 0.5),         # Return base
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Disable Arm', 0.5),
        ],

        'place_on_shelf_c': [
            # Move to shelf C
            ('Elbow -', 0.5),         # Turn to shelf C
            ('Elbow -', 0.5),
            ('Elbow -', 0.5),
            ('Elbow -', 0.5),
            ('Elbow -', 0.5),
            ('Shoulder +', 0.5),     # Position for shelf C
            ('Shoulder +', 0.5),
            ('Shoulder +', 0.5),
            ('Shoulder +', 0.5),
            ('Shoulder +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Gripper Open', 1.0),   # Release box
            # Return to neutral - exact reverse of positioning
            ('Base -', 0.5),        # Return elbow
            ('Base -', 0.5),
            ('Base -', 0.5),
            ('Base -', 0.5),
            ('Base -', 0.5),
            ('Shoulder -', 0.5),     # Return shoulder
            ('Shoulder -', 0.5),
            ('Shoulder -', 0.5),
            ('Shoulder -', 0.5),
            ('Shoulder -', 0.5),
            # Return base to center
            ('Elbow +', 0.5),         # Return base
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Disable Arm', 0.5),
        ],

        'move_forward_and_place': [
            # Enable navigation and move forward for 12 seconds
            ('Enable Motion', 1.0),    # Enable navigation motors
            ('Forward', 1.0),          # Move forward for 1 second
            ('Stop', 1.0),             # Stop movement
            ('Disable Motion', 1.0),   # Disable navigation motors
            
            # Now pick up box from back position 1 (using existing sequence)
            ('Enable Arm', 1.0),
            # Turn to back - same as initial sequence
            ('Elbow +', 0.5),         # Turn 1
            ('Elbow +', 0.5),         # Turn 2
            ('Elbow +', 0.5),         # Turn 3
            ('Elbow +', 0.5),         # Turn 4
            ('Elbow +', 0.5),         # Turn 5
            ('Elbow +', 0.5),         # Turn 6
            ('Elbow +', 0.5),         # Turn 7
            ('Elbow +', 0.5),         # Turn 8
            ('Elbow +', 0.5),         # Turn 9
            # Move elbow to position 1 - exactly 30 steps
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            # Grip box
            ('Gripper Close', 1.0),  # Grip box
            # Return elbow - exactly 30 steps
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            # Return base to front - reverse of turn
            ('Base +', 0.5),         # Return turn 1
            ('Base +', 0.5),         # Return turn 2
            ('Base +', 0.5),         # Return turn 3
            ('Base +', 0.5),         # Return turn 4
            ('Base +', 0.5),         # Return turn 5
            ('Base +', 0.5),         # Return turn 6
            ('Base +', 0.5),         # Return turn 7
            ('Base +', 0.5),         # Return turn 8
            ('Base +', 0.5),         # Return turn 9
            
            # Now place on shelf A (using existing sequence)
            # Move to shelf A
            ('Elbow -', 0.5),         # Turn to shelf A
            ('Elbow -', 0.5),
            ('Elbow -', 0.5),
            ('Shoulder +', 0.5),     # Position for shelf A
            ('Shoulder +', 0.5),
            ('Shoulder +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Base +', 0.5),
            ('Gripper Open', 1.0),   # Release box
            # Return to neutral - exact reverse of positioning
            ('Base -', 0.5),        # Return elbow
            ('Base -', 0.5),
            ('Base -', 0.5),
            ('Shoulder -', 0.5),     # Return shoulder
            ('Shoulder -', 0.5),
            ('Shoulder -', 0.5),
            # Return base to center
            ('Elbow +', 0.5),         # Return base
            ('Elbow +', 0.5),
            ('Elbow +', 0.5),
            ('Disable Arm', 0.5),    # Disable arm at the end
        ],
    }
    
    try:
        if os.path.exists(SEQUENCES_FILE):
            # Load existing sequences from the JSON file
            with open(SEQUENCES_FILE, 'r') as f:
                SEQUENCES.update(json.load(f))
            print(f"Loaded {len(SEQUENCES)} sequences from {SEQUENCES_FILE}")
            return True
        else:
            # First run - initialize with default sequences and save to file
            print(f"Sequences file {SEQUENCES_FILE} not found. Creating with default sequences.")
            SEQUENCES.update(default_sequences)
            save_sequences_to_file()
            return True
    except Exception as e:
        print(f"Error loading sequences: {e}")
        # If there's an error, still initialize with defaults
        SEQUENCES.update(default_sequences)
    return False

# Try to load sequences at startup
load_sequences_from_file()

@app.route('/sequence_recorder', methods=['GET', 'POST'])
def sequence_recorder():
    """Page for recording and managing sequences"""
    global recording_commands, is_recording, SEQUENCES
    
    message = ""
    editing_sequence = request.args.get('edit')
    test_result = None
    
    # If editing an existing sequence, load it into recording_commands
    if editing_sequence and editing_sequence in SEQUENCES and not is_recording:
        recording_commands = SEQUENCES[editing_sequence].copy()
        message = f"Loaded sequence '{editing_sequence}' for editing. Press Start Recording to modify it."
    
    if request.method == 'POST':
        if 'reorder_commands' in request.form:
            try:
                # Get the new order from the form data
                new_order = json.loads(request.form['reorder_commands'])
                # Create a new list with the reordered commands
                reordered_commands = []
                for item in new_order:
                    original_index = int(item['index'])
                    if 0 <= original_index < len(recording_commands):
                        command = recording_commands[original_index][0]
                        delay = float(item['delay'])
                        reordered_commands.append((command, delay))
                recording_commands = reordered_commands
                # Save changes if editing an existing sequence
                if editing_sequence:
                    SEQUENCES[editing_sequence] = recording_commands.copy()
                    save_sequences_to_file()
                message = "Command order updated successfully."
            except Exception as e:
                message = f"Error reordering commands: {str(e)}"
        
        elif 'start_recording' in request.form:
            is_recording = True
            message = "Recording started. Use the controls below to record commands."
        
        elif 'stop_recording' in request.form:
            is_recording = False
            message = f"Recording stopped. {len(recording_commands)} commands recorded."
        
        elif 'save_sequence' in request.form:
            sequence_name = request.form.get('sequence_name', '')
            if not sequence_name:
                message = "Error: Please provide a sequence name."
            elif not recording_commands:
                message = "Error: No commands recorded yet."
            else:
                SEQUENCES[sequence_name] = recording_commands.copy()
                save_sequences_to_file()
                message = f"Sequence '{sequence_name}' saved with {len(recording_commands)} commands."
        
        elif 'delete_sequence' in request.form:
            sequence_name = request.form.get('delete_name', '')
            if sequence_name in SEQUENCES:
                del SEQUENCES[sequence_name]
                save_sequences_to_file()
                message = f"Sequence '{sequence_name}' deleted."
            else:
                message = f"Sequence '{sequence_name}' not found."
        
        elif 'clear_commands' in request.form:
            recording_commands = []
            message = "Recorded commands cleared."
        
        elif 'remove_command' in request.form:
            try:
                index = int(request.form.get('command_index', -1))
                if 0 <= index < len(recording_commands):
                    removed_cmd = recording_commands.pop(index)
                    # Save changes if editing an existing sequence
                    if editing_sequence:
                        SEQUENCES[editing_sequence] = recording_commands.copy()
                        save_sequences_to_file()
                    message = f"Removed command: {removed_cmd[0]}"
                else:
                    message = "Invalid command index."
            except ValueError:
                message = "Invalid command index format."
        
        elif 'update_delay' in request.form:
            try:
                index = int(request.form.get('command_index', -1))
                new_delay = float(request.form.get('new_delay', 0.5))
                if 0 <= index < len(recording_commands):
                    cmd = recording_commands[index][0]
                    recording_commands[index] = (cmd, new_delay)
                    # Save changes if editing an existing sequence
                    if editing_sequence:
                        SEQUENCES[editing_sequence] = recording_commands.copy()
                        save_sequences_to_file()
                    message = f"Updated delay for command {index+1}: {cmd} to {new_delay}s"
                else:
                    message = "Invalid command index."
            except ValueError:
                message = "Invalid delay or index format."
        
        elif 'test_sequence' in request.form:
            sequence_name = request.form.get('test_name', '')
            if sequence_name in SEQUENCES:
                # Create a temporary sequence for testing
                temp_sequence = SEQUENCES[sequence_name].copy()
                
                # Execute the sequence
                result = "Testing sequence: " + sequence_name + "<br>"
                
                # Execute each command in the sequence
                for i, (cmd, delay) in enumerate(temp_sequence):
                    # Check if it's an arm or navigation command
                    if cmd in ARM_COMMANDS:
                        cmd_result = send_command(cmd)
                    elif cmd in NAV_COMMANDS:
                        cmd_result = send_nav_command(cmd)
                    else:
                        cmd_result = f"Unknown command: {cmd}"
                    
                    result += f"Step {i+1}: {cmd} - {cmd_result}<br>"
                    time.sleep(delay)  # Wait for the specified delay
                
                test_result = result
                message = f"Test completed for sequence '{sequence_name}'."
            else:
                message = f"Sequence '{sequence_name}' not found."
        
        elif 'test_current' in request.form:
            if recording_commands:
                # Execute the current recorded sequence
                result = "Testing current recorded sequence:<br>"
                
                # Execute each command in the sequence
                for i, (cmd, delay) in enumerate(recording_commands):
                    # Check if it's an arm or navigation command
                    if cmd in ARM_COMMANDS:
                        cmd_result = send_command(cmd)
                    elif cmd in NAV_COMMANDS:
                        cmd_result = send_nav_command(cmd)
                    else:
                        cmd_result = f"Unknown command: {cmd}"
                    
                    result += f"Step {i+1}: {cmd} - {cmd_result}<br>"
                    time.sleep(delay)  # Wait for the specified delay
                
                test_result = result
                message = "Test completed for current recorded sequence."
            else:
                message = "No commands recorded to test."
        
        elif 'arm_cmd' in request.form:
            cmd_label = request.form['arm_cmd']
            response = send_command(cmd_label)
            if is_recording:
                # Default delay of 0.5 seconds
                delay = float(request.form.get('delay', 0.5))
                recording_commands.append((cmd_label, delay))
                message = f"Recorded arm command: {cmd_label} with delay {delay}s"
        
        elif 'nav_cmd' in request.form:
            cmd_label = request.form['nav_cmd']
            response = send_nav_command(cmd_label)
            if is_recording:
                # Default delay of 0.5 seconds
                delay = float(request.form.get('delay', 0.5))
                recording_commands.append((cmd_label, delay))
                message = f"Recorded navigation command: {cmd_label} with delay {delay}s"
    
    # Create HTML for the page
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Sequence Recorder</title>
        <style>
            body { font-family: Arial; text-align: center; }
            .container { max-width: 1200px; margin: 0 auto; display: flex; flex-direction: column; }
            .back-button { background-color: #f0f0f0; width: 150px; height: 40px; margin-bottom: 20px; }
            .control-panel { display: flex; flex-wrap: wrap; justify-content: space-between; }
            .panel { flex: 1; margin: 10px; padding: 15px; background-color: #f5f5f5; border-radius: 8px; min-width: 300px; }
            .commands { display: flex; flex-direction: column; }
            button { margin: 5px; padding: 8px; cursor: pointer; }
            .arm-button { background-color: #d1e7dd; }
            .nav-button { background-color: #cfe2ff; }
            .record-button { background-color: #f8d7da; font-weight: bold; }
            .save-button { background-color: #d1e7dd; font-weight: bold; }
            .test-button { background-color: #ffc107; font-weight: bold; }
            .message { margin: 20px; padding: 10px; background-color: #ffeeba; border-radius: 5px; }
            .recording { color: red; font-weight: bold; animation: blink 1s infinite; }
            .recorded-commands { text-align: left; margin: 10px; padding: 10px; background-color: #f8f9fa; border-radius: 5px; max-height: 400px; overflow-y: auto; }
            .sequence-list { text-align: left; margin: 10px; max-height: 200px; overflow-y: auto; }
            .sequence-item { margin: 5px 0; padding: 5px; background-color: #e9ecef; border-radius: 3px; display: flex; justify-content: space-between; }
            .delete-btn { background-color: #dc3545; color: white; border: none; border-radius: 3px; cursor: pointer; }
            .edit-btn { background-color: #0d6efd; color: white; border: none; border-radius: 3px; cursor: pointer; }
            .test-btn { background-color: #ffc107; border: none; border-radius: 3px; cursor: pointer; }
            .command-item { display: flex; justify-content: space-between; align-items: center; margin-bottom: 5px; padding: 8px; background: white; border: 1px solid #ddd; cursor: move; }
            .command-item:hover { background: #f0f0f0; }
            .command-drag-handle { cursor: move; padding: 0 10px; color: #666; }
            .command-controls { display: flex; align-items: center; gap: 5px; }
            .remove-btn { background-color: #dc3545; color: white; border: none; border-radius: 3px; cursor: pointer; padding: 2px 5px; font-size: 12px; }
            .add-time-btn { background-color: #28a745; color: white; border: none; border-radius: 3px; cursor: pointer; padding: 2px 5px; font-size: 12px; }
            .test-results { text-align: left; margin: 10px; padding: 10px; background-color: #e9ecef; border-radius: 5px; max-height: 300px; overflow-y: auto; }
            input[type="text"], input[type="number"] { padding: 8px; margin: 5px; }
            @keyframes blink {
                0% { opacity: 1; }
                50% { opacity: 0.5; }
                100% { opacity: 1; }
            }
        </style>
        <script>
            function confirmDelete(name) {
                return confirm('Are you sure you want to delete sequence "' + name + '"?');
            }
            
            function confirmClear() {
                return confirm('Are you sure you want to clear all recorded commands?');
            }
            
            function confirmTest(name) {
                return confirm('Are you sure you want to test sequence "' + name + '"? The robot will execute all commands in the sequence.');
            }
            
            function updateDelay(value) {
                document.getElementById('arm_delay').value = value;
                document.getElementById('nav_delay').value = value;
                document.getElementById('delay_display').innerText = value + 's';
            }

            // Add time to a command's delay
            function addTime(index, amount) {
                const delayInput = document.querySelector(`input[name="new_delay"][data-index="${index}"]`);
                let currentDelay = parseFloat(delayInput.value);
                currentDelay += amount;
                if (currentDelay < 0.1) currentDelay = 0.1;
                delayInput.value = currentDelay.toFixed(1);
                document.querySelector(`form[data-index="${index}"]`).submit();
            }

            // Initialize drag and drop when the page loads
            document.addEventListener('DOMContentLoaded', function() {
                const commandsList = document.getElementById('recorded-commands-list');
                if (!commandsList) return;

                let draggedItem = null;
                let draggedIndex = null;

                // Add event listeners to all command items
                document.querySelectorAll('.command-item').forEach((item, index) => {
                    item.setAttribute('draggable', true);
                    
                    item.addEventListener('dragstart', function(e) {
                        draggedItem = item;
                        draggedIndex = index;
                        setTimeout(() => item.classList.add('dragging'), 0);
                    });

                    item.addEventListener('dragend', function() {
                        draggedItem = null;
                        draggedIndex = null;
                        item.classList.remove('dragging');
                    });

                    item.addEventListener('dragover', function(e) {
                        e.preventDefault();
                        if (this === draggedItem) return;
                        
                        const rect = this.getBoundingClientRect();
                        const midY = (rect.top + rect.bottom) / 2;
                        const mouseY = e.clientY;
                        
                        if (mouseY < midY) {
                            this.parentNode.insertBefore(draggedItem, this);
                        } else {
                            this.parentNode.insertBefore(draggedItem, this.nextSibling);
                        }
                    });

                    item.addEventListener('dragend', function() {
                        // Get the new order and submit it
                        const newOrder = Array.from(commandsList.children).map(item => {
                            return {
                                index: item.getAttribute('data-original-index'),
                                command: item.getAttribute('data-command'),
                                delay: item.querySelector('input[type="number"]').value
                            };
                        });
                        
                        // Create a hidden form to submit the new order
                        const form = document.createElement('form');
                        form.method = 'POST';
                        form.style.display = 'none';
                        
                        const input = document.createElement('input');
                        input.type = 'hidden';
                        input.name = 'reorder_commands';
                        input.value = JSON.stringify(newOrder);
                        
                        form.appendChild(input);
                        document.body.appendChild(form);
                        form.submit();
                    });
                });
            });
        </script>
    </head>
    <body>
        <div class="container">
            <h1>Sequence Recorder</h1>
            <a href="/"><button class="back-button">Back to Main Menu</button></a>
    """
    
    # Add message if any
    if message:
        html += f"""
            <div class="message">
                {message}
            </div>
        """
    
    # Recording status and controls
    html += f"""
            <div class="panel">
                <h2>Recording Controls</h2>
                <div>
                    <form method="post">
                        <button type="submit" name="start_recording" class="record-button">Start Recording</button>
                        <button type="submit" name="stop_recording" class="record-button">Stop Recording</button>
                        <button type="submit" name="clear_commands" class="record-button" onclick="return confirmClear()">Clear Commands</button>
                        <button type="submit" name="test_current" class="test-button" onclick="return confirm('Test current sequence? The robot will execute all recorded commands.')">Test Current Sequence</button>
                    </form>
                    <div>
                        Status: <span class="{'recording' if is_recording else ''}">
                            {'RECORDING' if is_recording else 'Not Recording'}
                        </span>
                    </div>
                </div>
                
                <h3>Command Delay</h3>
                <div>
                    <input type="range" min="0.1" max="3.0" step="0.1" value="0.5" 
                           oninput="updateDelay(this.value)" style="width: 200px;">
                    <div>Delay: <span id="delay_display">0.5s</span></div>
                </div>
                
                <h3>Recorded Commands ({len(recording_commands)})</h3>
                <div class="recorded-commands">
                    <div id="recorded-commands-list">
    """
    
    # Show recorded commands with remove buttons, delay editing, and drag handles
    for i, (cmd, delay) in enumerate(recording_commands):
        html += f"""
                    <div class="command-item" data-original-index="{i}" data-command="{cmd}">
                        <div class="command-drag-handle">☰</div>
                        <div>{i+1}. {cmd}</div>
                        <div class="command-controls">
                            <form method="post" style="display: inline;" data-index="{i}">
                                <input type="hidden" name="command_index" value="{i}">
                                <input type="number" name="new_delay" value="{delay}" min="0.1" max="10.0" step="0.1" style="width: 60px;">s
                                <button type="submit" name="update_delay" value="1" class="edit-btn" style="padding: 2px 5px; font-size: 12px;">Update</button>
                                <button type="submit" name="remove_command" value="1" class="remove-btn">Remove</button>
                            </form>
                            <button onclick="addTime({i}, 0.1)" class="add-time-btn">+0.1s</button>
                            <button onclick="addTime({i}, 0.5)" class="add-time-btn">+0.5s</button>
                        </div>
                    </div>
        """
    
    html += """
                    </div>
                </div>
    """
    
    html += """
                <h3>Save Sequence</h3>
                <form method="post">
    """
    
    # If editing, pre-fill the sequence name
    if editing_sequence:
        html += f"""
                    <input type="text" name="sequence_name" placeholder="Sequence name" value="{editing_sequence}" required>
        """
    else:
        html += """
                    <input type="text" name="sequence_name" placeholder="Sequence name" required>
        """
    
    html += """
                    <button type="submit" name="save_sequence" class="save-button">Save Sequence</button>
                </form>
            </div>
            
            <div class="control-panel">
                <div class="panel">
                    <h2>Arm Controls</h2>
                    <form method="post" class="commands">
                        <input type="hidden" id="arm_delay" name="delay" value="0.5">
                        <div>
                            <button type="submit" name="arm_cmd" value="Enable Arm" class="arm-button">Enable Arm</button>
                            <button type="submit" name="arm_cmd" value="Disable Arm" class="arm-button">Disable Arm</button>
                        </div>
                        <div>
                            <button type="submit" name="arm_cmd" value="Base +" class="arm-button">Base +</button>
                            <button type="submit" name="arm_cmd" value="Base -" class="arm-button">Base -</button>
                        </div>
                        <div>
                            <button type="submit" name="arm_cmd" value="Shoulder +" class="arm-button">Shoulder +</button>
                            <button type="submit" name="arm_cmd" value="Shoulder -" class="arm-button">Shoulder -</button>
                        </div>
                        <div>
                            <button type="submit" name="arm_cmd" value="Elbow +" class="arm-button">Elbow +</button>
                            <button type="submit" name="arm_cmd" value="Elbow -" class="arm-button">Elbow -</button>
                        </div>
                        <div>
                            <button type="submit" name="arm_cmd" value="Gripper Open" class="arm-button">Gripper Open</button>
                            <button type="submit" name="arm_cmd" value="Gripper Close" class="arm-button">Gripper Close</button>
                        </div>
                    </form>
                </div>
                
                <div class="panel">
                    <h2>Navigation Controls</h2>
                    <form method="post" class="commands">
                        <input type="hidden" id="nav_delay" name="delay" value="0.5">
                        <div>
                            <button type="submit" name="nav_cmd" value="Enable Motion" class="nav-button">Enable Motion</button>
                            <button type="submit" name="nav_cmd" value="Disable Motion" class="nav-button">Disable Motion</button>
                        </div>
                        <div>
                            <button type="submit" name="nav_cmd" value="Forward" class="nav-button">Forward</button>
                        </div>
                        <div>
                            <button type="submit" name="nav_cmd" value="Left" class="nav-button">Left</button>
                            <button type="submit" name="nav_cmd" value="Stop" class="nav-button">Stop</button>
                            <button type="submit" name="nav_cmd" value="Right" class="nav-button">Right</button>
                        </div>
                        <div>
                            <button type="submit" name="nav_cmd" value="Backward" class="nav-button">Backward</button>
                        </div>
                        <div>
                            <button type="submit" name="nav_cmd" value="Rotate CW" class="nav-button">Rotate CW</button>
                            <button type="submit" name="nav_cmd" value="Rotate CCW" class="nav-button">Rotate CCW</button>
                        </div>
                    </form>
                </div>
            </div>
            
            <div class="panel">
                <h2>Manage Sequences</h2>
                <div class="sequence-list">
    """
    
    # List all sequences with delete, edit and test buttons
    for seq_name in SEQUENCES:
        html += f"""
                    <div class="sequence-item">
                        <span>{seq_name} ({len(SEQUENCES[seq_name])} commands)</span>
                        <div>
                            <a href="/sequence_recorder?edit={seq_name}"><button class="edit-btn">Edit</button></a>
                            <form method="post" style="display: inline;" onsubmit="return confirmTest('{seq_name}')">
                                <input type="hidden" name="test_name" value="{seq_name}">
                                <button type="submit" name="test_sequence" class="test-btn">Test</button>
                            </form>
                            <form method="post" style="display: inline;" onsubmit="return confirmDelete('{seq_name}')">
                                <input type="hidden" name="delete_name" value="{seq_name}">
                                <button type="submit" name="delete_sequence" class="delete-btn">Delete</button>
                            </form>
                        </div>
                    </div>
        """
    
    html += """
                </div>
            </div>
    """
    
    # Add test results if any
    if test_result:
        html += f"""
            <div class="panel">
                <h2>Test Results</h2>
                <div class="test-results">
                    {test_result}
                </div>
            </div>
        """
    
    html += """
        </div>
    </body>
    </html>
    """
    
    # Add JavaScript for handling delay updates
    html += """
        <script>
            function addTime(index, amount) {
                const form = document.querySelector(`form[data-index="${index}"]`);
                const delayInput = form.querySelector('input[name="new_delay"]');
                let currentDelay = parseFloat(delayInput.value);
                currentDelay += amount;
                if (currentDelay < 0.1) currentDelay = 0.1;
                delayInput.value = currentDelay.toFixed(1);
                
                // Create and append hidden input for update_delay
                const updateInput = document.createElement('input');
                updateInput.type = 'hidden';
                updateInput.name = 'update_delay';
                updateInput.value = '1';
                form.appendChild(updateInput);
                
                // Submit the form
                form.submit();
            }
        </script>
    """
    
    return html

@app.route('/sequence_help')
def sequence_help_page():
    """Page showing help information about creating custom sequences"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Custom Sequence Help</title>
        <style>
            body { font-family: Arial; text-align: left; margin: 20px; }
            .container { max-width: 1000px; margin: 0 auto; }
            .back-button { background-color: #f0f0f0; width: 150px; height: 40px; margin-bottom: 20px; }
            h1, h2 { color: #333; }
            .section { margin: 20px 0; padding: 15px; background-color: #f5f5f5; border-radius: 8px; }
            .important { color: #cc0000; font-weight: bold; }
            pre { background-color: #eee; padding: 10px; border-radius: 5px; overflow-x: auto; }
            table { border-collapse: collapse; width: 100%; }
            th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
            th { background-color: #f2f2f2; }
            tr:nth-child(even) { background-color: #f9f9f9; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Custom Sequence Help</h1>
            <a href="/"><button class="back-button">Back to Main Menu</button></a>
            
            <div class="section">
                <h2>Creating Custom Sequences</h2>
                <p>
                    The robot controller now supports custom sequences that can replace the default ones.
                    You can create your own sequences with the Sequence Recorder and give them specific names
                    to replace the default sequences used in automation.
                </p>
            </div>
            
            <div class="section">
                <h2>Custom Sequence Names</h2>
                <p class="important">
                    To replace default sequences, use these specific names for your custom sequences:
                </p>
                <table>
                    <tr>
                        <th>Purpose</th>
                        <th>Default Name</th>
                        <th>Custom Name to Use</th>
                    </tr>
                    <tr>
                        <td>Initial pickup sequence</td>
                        <td>pick_and_store_back</td>
                        <td>custom_pickup</td>
                    </tr>
                    <tr>
                        <td>Store in back position 1</td>
                        <td>store_back_position_1</td>
                        <td>custom_store_1</td>
                    </tr>
                    <tr>
                        <td>Store in back position 2</td>
                        <td>store_back_position_2</td>
                        <td>custom_store_2</td>
                    </tr>
                    <tr>
                        <td>Store in back position 3</td>
                        <td>store_back_position_3</td>
                        <td>custom_store_3</td>
                    </tr>
                    <tr>
                        <td>Return to home position</td>
                        <td>return_to_home</td>
                        <td>custom_home</td>
                    </tr>
                    <tr>
                        <td>Pick from back position 1</td>
                        <td>pick_from_back_1</td>
                        <td>custom_pick_back_1</td>
                    </tr>
                    <tr>
                        <td>Pick from back position 2</td>
                        <td>pick_from_back_2</td>
                        <td>custom_pick_back_2</td>
                    </tr>
                    <tr>
                        <td>Pick from back position 3</td>
                        <td>pick_from_back_3</td>
                        <td>custom_pick_back_3</td>
                    </tr>
                    <tr>
                        <td>Place on shelf A</td>
                        <td>place_on_shelf_a</td>
                        <td>custom_place_shelf_a</td>
                    </tr>
                    <tr>
                        <td>Place on shelf B</td>
                        <td>place_on_shelf_b</td>
                        <td>custom_place_shelf_b</td>
                    </tr>
                    <tr>
                        <td>Place on shelf C</td>
                        <td>place_on_shelf_c</td>
                        <td>custom_place_shelf_c</td>
                    </tr>
                </table>
            </div>
            
            <div class="section">
                <h2>How It Works</h2>
                <p>
                    When the robot processes a QR code and needs to execute a sequence, it will:
                </p>
                <ol>
                    <li>First look for a custom sequence with the appropriate name</li>
                    <li>If the custom sequence exists, it will use that sequence</li>
                    <li>If no custom sequence is found, it will fall back to the default sequence</li>
                </ol>
                <p>
                    This allows you to create your own custom sequences while still maintaining compatibility
                    with the default automation flow.
                </p>
            </div>
            
            <div class="section">
                <h2>Steps to Create a Custom Sequence</h2>
                <ol>
                    <li>Go to the Sequence Recorder page</li>
                    <li>Create your sequence by recording commands</li>
                    <li>When saving, use one of the custom names from the table above</li>
                    <li>Test your sequence using the "Test" button before using it in automation</li>
                </ol>
            </div>
            
            <div class="section">
                <h2>Tips for Custom Sequences</h2>
                <ul>
                    <li>Make sure to start with "Enable Arm" and end with "Disable Arm" when appropriate</li>
                    <li>Test sequences thoroughly before using them in production</li>
                    <li>If a sequence doesn't work as expected, you can always delete it and the system will fall back to the default sequence</li>
                    <li>Back up your sequences.json file regularly to avoid losing your custom sequences</li>
                </ul>
            </div>
        </div>
    </body>
    </html>
    """
    return html

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    finally:
        cleanup() 