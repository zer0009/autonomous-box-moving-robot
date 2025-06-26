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
    'Base +': 'w',
    'Base -': 's',
    'Shoulder +': 'a',
    'Shoulder -': 'd',
    'Elbow +': 'q',
    'Elbow -': 'e',
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

# Predefined sequences for common tasks
SEQUENCES = {
    'pick_and_store_back': [
        ('Enable Arm', 1.0),     # Enable the arm first
        ('Base -', 0.1),         # First base movement 
        ('Base -', 0.1),         # Second base movement
        ('Base -', 0.1), 
        ('Base -', 0.1),   
        ('Gripper Open', 1.0), 
        ('Shoulder -', 0.1),     # Lower arm toward box
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),     
        ('Shoulder -', 0.1),
        ('Shoulder -', 0.1),
        ('Shoulder -', 0.1),

        ('Gripper Close', 1.0),  # Grip box
        ('Shoulder +', 0.5),     # Lift box
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),
        ('Shoulder +', 0.5),     
        ('Shoulder +', 0.5),
        ('Base -', 0.5),         # Turn to back
        ('Base -', 0.5),         
        ('Base -', 0.5),         
        ('Base -', 0.5),         
        ('Base -', 0.5),         
        ('Base -', 0.5),               # Complete turn to back
    ],

    'store_back_position_1': [  # 30 Elbow - iterations for position 1
        # Move elbow to position 1
        ('Elbow -', 0.1),
    ] * 34 + [
        ('Base +', 0.5),         # Reverse turn 1
        ('Base +', 0.5),         # Reverse turn 2
        ('Base +', 0.5),
        # Release box
        ('Gripper Open', 1.0),   # Release box
        # Return elbow exactly 26 steps (reduced from 30)
        ('Elbow +', 0.1),        # Return elbow
    ] * 32,

    'store_back_position_2': [  # 40 Elbow - iterations for position 2
        ('Base +', 0.5),         # Reverse turn 1
        ('Base +', 0.5),         # Reverse turn 2
        ('Base +', 0.5),
        # Move elbow to position 2
        ('Elbow -', 0.5),
    ] * 48 + [
        ('Base +', 0.5),         # Reverse turn 1
        ('Base +', 0.5),         # Reverse turn 2
        ('Base +', 0.5),
        # Release box
        ('Gripper Open', 1.0),   # Release box
        # Return elbow exactly 36 steps (reduced from 40)
        ('Elbow +', 0.5),        # Return elbow
    ] * 42,

    'store_back_position_3': [  # 50 Elbow - iterations for position 3
        ('Base +', 0.5),         # Reverse turn 1
        ('Base +', 0.5),         # Reverse turn 2
        ('Base +', 0.5),
        # Move elbow to position 3
        ('Elbow -', 0.5),
    ] * 55 + [
        # Release box
        ('Gripper Open', 1.0),   # Release box
        # Return elbow exactly 50 steps (reduced from 55)
        ('Elbow +', 0.5),        # Return elbow
    ] * 52,

    'return_to_home': [
        # Return base to center - exact reverse of turn to back
        ('Base +', 0.5), 
        ('Base +', 0.5), 
        ('Base +', 0.5), 
        ('Base +', 0.5), 
        ('Base +', 0.5), 
        ('Base +', 0.5), 
        ('Base +', 0.5), 
        ('Base +', 0.5), 
        ('Base +', 0.5), 
        ('Base +', 0.5)
        # Only disable the arm, not the entire robot
        ('Disable Arm', 0.5),    # Disable arm only, navigation can still work
    ],

    'pick_from_back_1': [  # Pick from first back position (30 iterations)
        ('Enable Arm', 1.0),
        # Turn to back - same as initial sequence
        ('Base -', 0.5),         # Turn 1
        ('Base -', 0.5),         # Turn 2
        ('Base -', 0.5),         # Turn 3
        ('Base -', 0.5),         # Turn 4
        ('Base -', 0.5),         # Turn 5
        ('Base -', 0.5),         # Turn 6
        ('Base -', 0.5),         # Turn 7
        ('Base -', 0.5),         # Turn 8
        ('Base -', 0.5),         # Turn 9
        # Move elbow to position 1 - exactly 30 steps
        ('Elbow -', 0.5),
    ] * 30 + [
        # Grip box
        ('Gripper Close', 1.0),  # Grip box
        # Return elbow - exactly 30 steps
        ('Elbow +', 0.5),        # Return elbow
    ] * 30 + [
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
    ],

    'pick_from_back_2': [  # Pick from second back position (40 iterations)
        ('Enable Arm', 1.0),
        # Turn to back - same as initial sequence
        ('Base -', 0.5),         # Turn 1
        ('Base -', 0.5),         # Turn 2
        ('Base -', 0.5),         # Turn 3
        ('Base -', 0.5),         # Turn 4
        ('Base -', 0.5),         # Turn 5
        ('Base -', 0.5),         # Turn 6
        ('Base -', 0.5),         # Turn 7
        ('Base -', 0.5),         # Turn 8
        ('Base -', 0.5),         # Turn 9
        # Move elbow to position 2 - exactly 40 steps
        ('Elbow -', 0.5),
    ] * 40 + [
        # Grip box
        ('Gripper Close', 1.0),  # Grip box
        # Return elbow - exactly 40 steps
        ('Elbow +', 0.5),        # Return elbow
    ] * 40 + [
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
    ],

    'pick_from_back_3': [  # Pick from third back position (50 iterations)
        ('Enable Arm', 1.0),
        # Turn to back - same as initial sequence
        ('Base -', 0.5),         # Turn 1
        ('Base -', 0.5),         # Turn 2
        ('Base -', 0.5),         # Turn 3
        ('Base -', 0.5),         # Turn 4
        ('Base -', 0.5),         # Turn 5
        ('Base -', 0.5),         # Turn 6
        ('Base -', 0.5),         # Turn 7
        ('Base -', 0.5),         # Turn 8
        ('Base -', 0.5),         # Turn 9
        # Move elbow to position 3 - exactly 50 steps
        ('Elbow -', 0.5),
    ] * 50 + [
        # Grip box
        ('Gripper Close', 1.0),  # Grip box
        # Return elbow - exactly 50 steps
        ('Elbow +', 0.5),        # Return elbow
    ] * 50 + [
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
    ],

    'place_on_shelf_a': [
        # Move to shelf A
        ('Base +', 0.5),         # Turn to shelf A
        ('Base +', 0.5),
        ('Base +', 0.5),
        ('Shoulder -', 0.5),     # Position for shelf A
        ('Shoulder -', 0.5),
        ('Shoulder -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Gripper Open', 1.0),   # Release box
        # Return to neutral - exact reverse of positioning
        ('Elbow +', 0.5),        # Return elbow
        ('Elbow +', 0.5),
        ('Elbow +', 0.5),
        ('Shoulder +', 0.5),     # Return shoulder
        ('Shoulder +', 0.5),
        ('Shoulder +', 0.5),
        # Return base to center
        ('Base -', 0.5),         # Return base
        ('Base -', 0.5),
        ('Base -', 0.5),
        ('Disable Arm', 0.5),
    ],

    'place_on_shelf_b': [
        # Move to shelf B
        ('Base +', 0.5),         # Turn to shelf B
        ('Base +', 0.5),
        ('Base +', 0.5),
        ('Base +', 0.5),
        ('Shoulder -', 0.5),     # Position for shelf B
        ('Shoulder -', 0.5),
        ('Shoulder -', 0.5),
        ('Shoulder -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Gripper Open', 1.0),   # Release box
        # Return to neutral - exact reverse of positioning
        ('Elbow +', 0.5),        # Return elbow
        ('Elbow +', 0.5),
        ('Elbow +', 0.5),
        ('Elbow +', 0.5),
        ('Shoulder +', 0.5),     # Return shoulder
        ('Shoulder +', 0.5),
        ('Shoulder +', 0.5),
        ('Shoulder +', 0.5),
        # Return base to center
        ('Base -', 0.5),         # Return base
        ('Base -', 0.5),
        ('Base -', 0.5),
        ('Base -', 0.5),
        ('Disable Arm', 0.5),
    ],

    'place_on_shelf_c': [
        # Move to shelf C
        ('Base +', 0.5),         # Turn to shelf C
        ('Base +', 0.5),
        ('Base +', 0.5),
        ('Base +', 0.5),
        ('Base +', 0.5),
        ('Shoulder -', 0.5),     # Position for shelf C
        ('Shoulder -', 0.5),
        ('Shoulder -', 0.5),
        ('Shoulder -', 0.5),
        ('Shoulder -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Elbow -', 0.5),
        ('Gripper Open', 1.0),   # Release box
        # Return to neutral - exact reverse of positioning
        ('Elbow +', 0.5),        # Return elbow
        ('Elbow +', 0.5),
        ('Elbow +', 0.5),
        ('Elbow +', 0.5),
        ('Elbow +', 0.5),
        ('Shoulder +', 0.5),     # Return shoulder
        ('Shoulder +', 0.5),
        ('Shoulder +', 0.5),
        ('Shoulder +', 0.5),
        ('Shoulder +', 0.5),
        # Return base to center
        ('Base -', 0.5),         # Return base
        ('Base -', 0.5),
        ('Base -', 0.5),
        ('Base -', 0.5),
        ('Base -', 0.5),
        ('Disable Arm', 0.5),
    ],
}

app = Flask(__name__)

# Initialize serial connection to ESP32 for arm control
arm_serial_available = False
try:
    print(f"Trying to connect to arm ESP32 on {ARM_SERIAL_PORT}...")
    arm_ser = serial.Serial(ARM_SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Wait for ESP32 to reset
    arm_serial_available = True
    print(f"Connected to arm ESP32 on {ARM_SERIAL_PORT}")
except Exception as e:
    print(f"Could not open serial port {ARM_SERIAL_PORT}: {e}")
    try:
        print(f"Trying alternative port {ARM_SERIAL_PORT_ALT} for arm ESP32...")
        arm_ser = serial.Serial(ARM_SERIAL_PORT_ALT, BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for ESP32 to reset
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
    time.sleep(2)  # Wait for ESP32 to reset
    nav_serial_available = True
    print(f"Connected to navigation ESP32 on {NAV_SERIAL_PORT}")
except Exception as e:
    print(f"Could not open serial port {NAV_SERIAL_PORT}: {e}")
    try:
        print(f"Trying alternative port {NAV_SERIAL_PORT_ALT} for navigation ESP32...")
        nav_ser = serial.Serial(NAV_SERIAL_PORT_ALT, BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for ESP32 to reset
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
                line = arm_ser.readline().decode(errors='ignore').strip()
                if line.startswith("Sensor States:"):
                    # Parse IR sensor states
                    try:
                        # Extract IR values
                        ir_values = []
                        parts = line.split()
                        for part in parts:
                            if part.startswith("IR") and ":" in part:
                                value = int(part.split(":")[1])
                                ir_values.append(value)
                        
                        if len(ir_values) == 5:  # Ensure we have all 5 IR sensor values
                            # Convert IR values to binary number
                            # Example: [1,1,1,1,1] becomes 31 (11111 in binary)
                            binary_value = 0
                            for i in range(5):
                                if ir_values[i]:  # If sensor reads 1
                                    binary_value |= (1 << (4-i))  # Set corresponding bit
                            
                            # Send binary value to navigation ESP32 without colon
                            ir_data = f"IR{binary_value}\n"
                            nav_ser.write(ir_data.encode())
                            print(f"Forwarded IR data to navigation: {ir_data.strip()} (binary: {bin(binary_value)[2:].zfill(5)})")
                    except Exception as e:
                        print(f"Error parsing IR data: {e}")
                
            # Read from navigation ESP32
            if nav_ser.in_waiting:
                line = nav_ser.readline().decode(errors='ignore').strip()
                if line.startswith("CORRECTION:"):
                    latest_correction = line.split(":", 1)[1].strip()
                    robot_state["ir_correction"] = latest_correction
                    print(f"Received IR correction: {latest_correction}")  # Add debug print
                    
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
    if camera is None:
        # Try different camera indices - expanded range to include higher indices
        for camera_index in [0, 1, 2]:
            try:
                print(f"Trying camera at index {camera_index}...")
                cam = cv2.VideoCapture(camera_index)
                if cam.isOpened():
                    ret, test_frame = cam.read()
                    if ret and test_frame is not None:
                        camera = cam
                        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        print(f"Connected to camera at index {camera_index}")
                        print(f"Camera resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                        return camera
                    else:
                        print(f"Camera at index {camera_index} opened but couldn't read frame")
                        cam.release()
                else:
                    print(f"Failed to open camera at index {camera_index}")
            except Exception as e:
                print(f"Error with camera at index {camera_index}: {e}")
        
        # If we get here, we failed to find a working camera
        print("Error: Could not find a working camera")
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
    camera = get_camera()
    if not camera:
        return
    
    qr_detector = cv2.QRCodeDetector()
    camera_running = True
    
    while camera_running:
        ret, frame = camera.read()
        if not ret:
            continue
            
        # Try to detect QR code
        try:
            data, bbox, _ = qr_detector.detectAndDecode(frame)
            if data:
                print(f"QR Code detected: {data}")
                # Always update last_qr_data and process it
                last_qr_data = data
                
                # Process the QR code data in the main thread
                # We'll use threading to avoid blocking the camera thread
                processing_thread = threading.Thread(target=process_qr_code, args=(data,))
                processing_thread.daemon = True
                processing_thread.start()
            
            # Even if no data is decoded, check if a QR code is detected visually
            # This is a workaround for cases where the QR code is seen but not decoded
            if bbox is not None and len(bbox) > 0:
                # Convert bbox to a more usable format for the web interface
                # OpenCV returns bbox as a 3D array with 4 points (corners)
                points = bbox[0]
                if len(points) == 4:
                    # Calculate the bounding rectangle that contains all points
                    x_coords = [p[0] for p in points]
                    y_coords = [p[1] for p in points]
                    x = min(x_coords)
                    y = min(y_coords)
                    width = max(x_coords) - x
                    height = max(y_coords) - y
                    
                    # Calculate distance based on QR code width
                    distance = calculate_distance(width)
                    
                    last_qr_bbox = {
                        "x": int(x),
                        "y": int(y),
                        "width": int(width),
                        "height": int(height),
                        "distance": round(distance, 2)  # Distance in cm
                    }
                    
                    # Draw bounding box on the frame for debugging
                    cv2.polylines(frame, [np.int32(points)], True, (0, 255, 0), 2)
                    
                    # If data is empty but we have a bounding box, try to manually trigger
                    # the robot for testing purposes
                    if not data and last_qr_data is None:
                        # Generate a test QR data for the detected box
                        test_data = "BOX_TEST"
                        cv2.putText(frame, f"{test_data} - {round(distance, 2)}cm", (int(x), int(y) - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # Only process if we haven't processed a box recently
                        if robot_state["status"] == "idle":
                            print("QR code detected visually but not decoded, using test data")
                            last_qr_data = test_data
                            processing_thread = threading.Thread(target=process_qr_code, args=(test_data,))
                            processing_thread.daemon = True
                            processing_thread.start()
                    else:
                        # Normal case with data
                        cv2.putText(frame, f"{data or 'Unknown'} - {round(distance, 2)}cm", (int(x), int(y) - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # If this is the first detection, try to calibrate the camera
                    if FOCAL_LENGTH is None:
                        calibrate_camera(width)
            else:
                # No QR code detected visually
                pass
                
        except Exception as e:
            print(f"QR detection error: {e}")
            
        time.sleep(0.1)  # Small delay to reduce CPU usage

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
            
            # Step 1: Pick up the box and move to back
            update_robot_state(status="picking", action=f"Picking up box: {box_id}")
            execute_sequence('pick_and_store_back')
            
            # Determine which back position to use based on available slots
            # For now, using a simple round-robin approach
            back_position = (int(box_id) % 3) + 1 if box_id.isdigit() else 1
            back_sequence = f'store_back_position_{back_position}'
            
            # Store the box at the selected back position
            update_robot_state(status="storing", action=f"Storing box at back position {back_position}")
            execute_sequence(back_sequence)
            
            # Return arm to home position
            update_robot_state(status="homing", action="Returning to home position")
            execute_sequence('return_to_home')
            
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

def navigate_to_shelf(target_shelf, back_position, box_id):
    """Handle navigation to target shelf with continuous feedback"""
    print(f"Starting navigation to {target_shelf}")
    
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
        
        # Start moving forward with explicit logging
        print("Starting forward movement...")
        response = send_nav_command("Forward")
        print(f"Forward command response: {response}")
        
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
            correction_value = float(ir_correction)
        except ValueError:
            correction_value = 0
        
        # Parse shelf letters for position comparison
        current_letter = current_marker[-1] if current_marker else None
        target_letter = target_shelf[-1] if target_shelf else None
        
        print(f"Adjusting navigation: Current={current_letter}, Target={target_letter}, IR={correction_value}")
        
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
                
        # Apply IR correction
        if correction_value > 1:
            # Too far right, adjust left
            print("Correcting left")
            send_nav_command("Left")
            update_robot_state(action=f"Correcting left (IR: {correction_value})")
        elif correction_value < -1:
            # Too far left, adjust right
            print("Correcting right")
            send_nav_command("Right")
            update_robot_state(action=f"Correcting right (IR: {correction_value})")
        
        # Resume forward motion if we're not at the target
        if current_marker != target_shelf:
            send_nav_command("Forward")
            
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
        
    shelf_letter = target_shelf[-1].lower()
    
    try:
        # Pick up box from back position
        update_robot_state(
            status="retrieving",
            action=f"Retrieving box from back position {back_position}",
            nav_status="stopped"
        )
        execute_sequence(f'pick_from_back_{back_position}')
        
        # Place on the correct shelf
        update_robot_state(
            status="placing",
            action=f"Placing box on shelf {shelf_letter.upper()}"
        )
        execute_sequence(f'place_on_shelf_{shelf_letter}')
        
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
            <a href="/status"><button class="mode-button {% if mode == 'status' %}active-mode{% endif %}">Robot Status</button></a>
            <a href="/sequence_analysis"><button class="mode-button {% if mode == 'analysis' %}active-mode{% endif %}">Sequence Analysis</button></a>
            <a href="/debug_serial"><button class="mode-button {% if mode == 'debug' %}active-mode{% endif %}">Debug Serial</button></a>
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
    </style>
    <script>
        function startCamera() {
            fetch('/start_camera')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status').innerText = data.status;
                    if (data.status === 'Camera started') {
                        document.getElementById('camera_img').src = "/video_feed";
                    }
                });
        }
        
        function stopCamera() {
            fetch('/stop_camera')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status').innerText = data.status;
                    document.getElementById('camera_img').src = "";
                });
        }
        
        function setCamera() {
            const cameraDevice = document.getElementById('camera_device').value;
            if (!cameraDevice) {
                alert('Please enter a camera device');
                return;
            }
            
            fetch('/set_camera', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ camera_device: cameraDevice }),
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerText = data.status;
                alert(data.status);
            });
        }
        
        function calibrateCamera() {
            const knownDistance = document.getElementById('known_distance').value;
            if (!knownDistance) {
                alert('Please enter a known distance in cm');
                return;
            }
            
            fetch('/calibrate_camera', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ known_distance: parseFloat(knownDistance) }),
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').innerText = data.status;
                alert(data.status);
            });
        }
        
        function checkQrStatus() {
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
                    }
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
            <p id="status">Camera inactive</p>
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
    camera = get_camera()
    if not camera:
        return
        
    qr_detector = cv2.QRCodeDetector()
    
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Try to detect QR codes in the frame for visualization
            try:
                data, bbox, _ = qr_detector.detectAndDecode(frame)
                if data and bbox is not None and len(bbox) > 0:
                    # Draw bounding box and text on the frame
                    points = bbox[0]
                    if len(points) == 4:
                        # Draw green polygon around QR code
                        cv2.polylines(frame, [np.int32(points)], True, (0, 255, 0), 2)
                        
                        # Calculate distance
                        x_coords = [p[0] for p in points]
                        y_coords = [p[1] for p in points]
                        x = min(x_coords)
                        y = min(y_coords)
                        width = max(x_coords) - x
                        height = max(y_coords) - y
                        
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
                # Just continue if QR detection fails
                pass
                
            # Add robot state information to the frame
            try:
                # Add status text at the bottom of the frame
                status_text = f"Status: {robot_state['status']} | Box: {robot_state['carrying_box'] or 'None'}"
                cv2.rectangle(frame, (0, frame.shape[0] - 30), (frame.shape[1], frame.shape[0]), (0, 0, 0), -1)
                cv2.putText(frame, status_text, (10, frame.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            except:
                pass
                
            # Convert to jpg for streaming
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

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
    
    if camera_running:
        return jsonify({"status": "Camera already running"})
    
    # Start camera in a separate thread
    camera_thread = threading.Thread(target=scan_qr_codes)
    camera_thread.daemon = True
    camera_thread.start()
    
    return jsonify({"status": "Camera started"})

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
    return Response(generate_camera_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

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
            # Send a test IR signal with all sensors active - without colon
            test_ir = "IR31\n"  # 31 = 11111 in binary
            nav_ser.write(test_ir.encode())
            print(f"Sent test IR signal: {test_ir.strip()}")
            
            # Wait for response
            time.sleep(0.5)
            response = ""
            while nav_ser.in_waiting:
                response += nav_ser.readline().decode(errors='ignore')
            
            result = f"Response: {response}" if response else "No response received"
        except Exception as e:
            result = f"Error: {str(e)}"
    else:
        result = "Navigation serial port not available"
    
    return jsonify({"result": result})

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    finally:
        cleanup() 