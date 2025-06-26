#!/usr/bin/env python3
"""
Robot Controller with QR Code Integration
Combines QR code scanning with robotic arm control
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
SERIAL_PORT = '/dev/ttyACM1'  # or '/dev/ttyACM0' if that's your ESP32 port
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
KNOWN_DISTANCE = 30.0  # cm
# Known width in pixels at the known distance (will be calibrated)
KNOWN_WIDTH_PIXELS = None

# Commands for the ESP32 arm
COMMANDS = {
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

# Predefined sequences for common tasks
SEQUENCES = {
    'standard_sequence': [
        ('Enable Arm', 1.0),     # Enable the arm first
        ('Base -', 0.5),         # First base movement 
        ('Base -', 0.5),         # Second base movement
        ('Base -', 0.5),  
        ('Gripper Open', 1.0), 
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box
        ('Shoulder -', 0.5),     # Lower arm toward box       # Extend arm further     # Move down to box
        ('Gripper Close', 1.0),  # Grip box - single command
        ('Shoulder +', 0.5),     # Start lifting box
        ('Shoulder +', 0.5),     # Continue lifting box
        ('Shoulder +', 0.5),     # Start lifting box
        ('Shoulder +', 0.5),     # Continue lifting box
        ('Shoulder +', 0.5),     # Start lifting box
        ('Shoulder +', 0.5), 
        ('Shoulder +', 0.5),     # Start lifting box
        ('Shoulder +', 0.5),     # Continue lifting box
        ('Shoulder +', 0.5),
        ('Shoulder +', 0.5),      # Start lifting box
        ('Base -', 0.5),         # First turn toward back
        ('Base -', 0.5),         # Second turn
        ('Base -', 0.5),         # Complete turn to robot back
        ('Elbow +', 0.5),        # Position over storage area
        ('Elbow +', 0.5),        # Position more precisely
        ('Gripper Open', 1.0),   # Release box - single command
        ('Shoulder +', 0.5),     # Move away from box
        ('Base +', 0.5),         # Start returning to position
        ('Base +', 0.5),         # Continue returning
        ('Base +', 0.5),         # Complete return to starting position
        ('Disable Arm', 0.5),    # Disable arm when done
    ],
    
    # Return to home position
    'home_position': [
        ('Enable Arm', 1.0),     # Enable the arm first
        ('Base +', 0.5),         # First base centering movement
        ('Base +', 0.5),         # Second base centering movement
        ('Base +', 0.5),         # Complete base centering
        ('Shoulder -', 0.5),     # First movement to raise arm
        ('Shoulder -', 0.5),     # Complete raising arm
        ('Elbow +', 0.5),        # First movement to fold arm
        ('Elbow +', 0.5),        # Complete folding arm
        ('Gripper Open', 1.0),   # Open gripper - single command
        ('Disable Arm', 0.5),    # Disable arm when done
    ],
}

app = Flask(__name__)

# Initialize serial connection to ESP32
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Wait for ESP32 to reset
    serial_available = True
except:
    print(f"Warning: Could not open serial port {SERIAL_PORT}")
    serial_available = False

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
    "status": "idle",        # idle, moving, picking, placing
    "carrying_box": None,    # ID of box being carried, or None
    "battery": 100,          # Battery percentage
    "position": "home",      # Current position identifier
    "last_action": "",       # Last action performed
    "error": None            # Any error state
}

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

def update_robot_state(status=None, box_id=None, position=None, action=None, error=None):
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
    
    # Simulate battery usage
    if status in ["moving", "picking", "placing"]:
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
            
            # Update robot state
            update_robot_state(status="picking", action=f"Picking up box: {box_id}")
            
            # Select the standard sequence
            sequence = 'standard_sequence'
            
            # Execute the selected sequence - ensure this runs even if there are errors elsewhere
            print(f"Executing sequence '{sequence}' for box {box_id}")
            result = execute_sequence(sequence)
            
            # Update robot state to indicate carrying the box
            update_robot_state(status="carrying", carrying_box=box_id)
            
            # Update robot state to idle after completing the sequence
            update_robot_state(status="idle", action=f"Completed processing box: {box_id}")
            
            print(f"Box {box_id} processed: {result}")
            
        elif qr_data.startswith('SHELF_'):
            print(f"Detected shelf: {qr_data}")
            update_robot_state(position=qr_data)
            # Could implement shelf-specific actions here
            
        elif qr_data.startswith('FLOOR_'):
            print(f"Detected floor marker: {qr_data}")
            update_robot_state(position=qr_data)
            # Could implement navigation actions here
            
        else:
            print(f"Unknown QR code format: {qr_data}")
            update_robot_state(error=f"Unknown QR code format: {qr_data}")
            
    except Exception as e:
        print(f"Error processing QR code: {str(e)}")
        update_robot_state(error=f"QR processing error: {str(e)}")
        # Reset to idle state after error
        update_robot_state(status="idle")

def send_command(cmd_label):
    """Send a command to the ESP32"""
    if not serial_available:
        return "Serial port not available"
    
    cmd = COMMANDS.get(cmd_label)
    if not cmd:
        return f"Unknown command: {cmd_label}"
    
    response = ""
    try:
        ser.write(cmd.encode())
        time.sleep(0.1)
        while ser.in_waiting:
            response += ser.readline().decode(errors='ignore')
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
        for cmd, delay in sequence:
            print(f"Executing: {cmd}")
            response = send_command(cmd)
            if "Error" in response:
                update_robot_state(error=f"Command failed: {cmd} - {response}")
                return f"Sequence {sequence_name} failed: {response}"
            time.sleep(delay)
        
        update_robot_state(status="idle", action=f"Completed sequence: {sequence_name}")
        return f"Sequence {sequence_name} completed"
    except Exception as e:
        update_robot_state(error=f"Sequence error: {str(e)}")
        return f"Sequence {sequence_name} failed: {str(e)}"

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
            <a href="/manual"><button class="mode-button {% if mode == 'manual' %}active-mode{% endif %}">Manual Control</button></a>
            <a href="/auto"><button class="mode-button {% if mode == 'auto' %}active-mode{% endif %}">Automatic Mode</button></a>
            <a href="/manual_sequence"><button class="mode-button {% if mode == 'sequence' %}active-mode{% endif %}">Sequence Control</button></a>
            <a href="/status"><button class="mode-button {% if mode == 'status' %}active-mode{% endif %}">Robot Status</button></a>
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
    """Manual control page"""
    response = ""
    if request.method == 'POST':
        cmd_label = request.form['cmd']
        response = send_command(cmd_label)
    
    return render_template_string(MANUAL_HTML, response=response)

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
    
    return jsonify({
        "qr_data": last_qr_data + distance_info if last_qr_data else None,
        "qr_bbox": last_qr_bbox
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

def cleanup():
    """Clean up resources before exit"""
    if camera_running:
        release_camera()
    if serial_available:
        ser.close()
    qr_generator.close()

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    finally:
        cleanup() 