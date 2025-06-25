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
    'Gripper Open': 'i',   # Open gripper fully (40mm)
    'Gripper Close': 'o',  # Close gripper to box width (35mm)
    'Gripper Precise': 'p' # Fine-tune gripper for perfect box fit
}

# Predefined sequences for common tasks
SEQUENCES = {
    'small_box': [
        ('Enable Arm', 1.0),        # Enable the arm first
        ('Base +', 0.8),            # Position base precisely for small box
        ('Shoulder +', 0.6),        # Lower arm carefully to box height
        ('Elbow -', 0.7),           # Extend arm precisely
        ('Gripper Open', 1.0),      # Open gripper fully (40mm)
        ('Elbow +', 0.15),          # Move slightly down to align with box
        ('Base +', 0.1),            # Fine adjust position
        ('Gripper Close', 0.8),     # Close gripper to 35mm (precise for 3.5cm box)
        ('Gripper Precise', 0.5),   # Fine-tune gripper for perfect fit
        ('Shoulder -', 0.6),        # Lift box carefully
        ('Base -', 1.2),            # Turn to robot back
        ('Elbow +', 0.5),           # Position over storage area
        ('Gripper Open', 1.0),      # Release box fully
        ('Shoulder +', 0.3),        # Move away from box
        ('Base +', 0.7),            # Return to starting position
        ('Disable Arm', 0.5),       # Disable arm when done
    ],
    
    'pick_up_box': [
        ('Enable Arm', 1.0),        # Enable the arm first
        ('Base +', 0.8),            # Position base precisely for small box
        ('Shoulder +', 0.6),        # Lower arm carefully to box height
        ('Elbow -', 0.7),           # Extend arm precisely
        ('Gripper Open', 1.0),      # Open gripper fully (40mm)
        ('Elbow +', 0.15),          # Move slightly down to align with box
        ('Base +', 0.1),            # Fine adjust position
        ('Gripper Close', 0.8),     # Close gripper to 35mm (precise for 3.5cm box)
        ('Gripper Precise', 0.5),   # Fine-tune gripper for perfect fit
        ('Shoulder -', 0.6),        # Lift box carefully
        ('Base -', 1.2),            # Turn to robot back
        ('Elbow +', 0.5),           # Position over storage area
        ('Gripper Open', 1.0),      # Release box fully
        ('Shoulder +', 0.3),        # Move away from box
        ('Base +', 0.7),            # Return to starting position
        ('Disable Arm', 0.5),       # Disable arm when done
    ],
    
    'light_box': [
        ('Enable Arm', 1.0),      # Enable the arm first
        ('Base +', 0.8),          # Position base precisely for small box
        ('Shoulder +', 0.6),      # Lower arm carefully to box height
        ('Elbow -', 0.7),         # Extend arm precisely
        ('Gripper Open', 1.0),    # Open gripper fully (40mm opening)
        ('Elbow +', 0.15),        # Move slightly down to align with box
        ('Base +', 0.1),          # Fine adjust position
        ('Gripper Close', 0.8),   # Close gripper to 35mm (precise for 3.5cm box)
        ('Shoulder -', 0.6),      # Lift box carefully
        ('Base -', 1.2),          # Turn to robot back
        ('Elbow +', 0.5),         # Position over storage area
        ('Gripper Open', 1.0),    # Release box fully
        ('Shoulder +', 0.3),      # Move away from box
        ('Base +', 0.7),          # Return to starting position
        ('Disable Arm', 0.5),     # Disable arm when done
    ],
    
    'heavy_box': [
        ('Enable Arm', 1.0),      # Enable the arm first
        ('Base +', 0.8),          # Position base precisely for small box
        ('Shoulder +', 0.6),      # Lower arm carefully to box height
        ('Elbow -', 0.7),         # Extend arm precisely
        ('Gripper Open', 1.0),    # Open gripper fully (40mm opening)
        ('Elbow +', 0.15),        # Move slightly down to align with box
        ('Base +', 0.1),          # Fine adjust position
        ('Gripper Close', 0.8),   # Close gripper to 35mm (precise for 3.5cm box)
        ('Shoulder -', 0.6),      # Lift box carefully
        ('Base -', 1.2),          # Turn to robot back
        ('Elbow +', 0.5),         # Position over storage area
        ('Gripper Open', 1.0),    # Release box fully
        ('Shoulder +', 0.3),      # Move away from box
        ('Base +', 0.7),          # Return to starting position
        ('Disable Arm', 0.5),     # Disable arm when done
    ],
    
    # Return to home position
    'home_position': [
        ('Enable Arm', 1.0),
        ('Base +', 0.5),      # Center the base
        ('Shoulder -', 0.5),  # Raise arm
        ('Elbow +', 0.5),     # Fold arm
        ('Gripper Open', 0.3), # Open gripper
        ('Disable Arm', 0.5),
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
        setup_camera()
    return camera

def setup_camera():
    """Setup camera with fallback options for USB cameras"""
    global camera
    
    # Try different camera indices - expanded range to include higher indices
    # First check standard indices 0-2
    for camera_index in [0, 1, 2]:
        try:
            print(f"Trying camera at index {camera_index}...")
            camera = cv2.VideoCapture(camera_index)
            if camera.isOpened():
                ret, test_frame = camera.read()
                if ret and test_frame is not None:
                    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    print(f"Connected to camera at index {camera_index}")
                    print(f"Camera resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                    return camera
                else:
                    print(f"Camera at index {camera_index} opened but couldn't read frame")
                    camera.release()
            else:
                print(f"Failed to open camera at index {camera_index}")
        except Exception as e:
            print(f"Error with camera at index {camera_index}: {e}")
    
    # Try device paths as fallback
    for device_path in ['/dev/video0', '/dev/video1', '/dev/video2']:
        try:
            print(f"Trying camera at path {device_path}...")
            camera = cv2.VideoCapture(device_path)
            if camera.isOpened():
                ret, test_frame = camera.read()
                if ret and test_frame is not None:
                    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    print(f"Connected to camera at path {device_path}")
                    print(f"Camera resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
                    return camera
                else:
                    print(f"Camera at path {device_path} opened but couldn't read frame")
                    camera.release()
            else:
                print(f"Failed to open camera at path {device_path}")
        except Exception as e:
            print(f"Error with camera at path {device_path}: {e}")
    
    print("Error: Could not open any camera")
    return None

def release_camera():
    """Release camera resources"""
    global camera, camera_running
    camera_running = False
    if camera:
        camera.release()
        camera = None

def scan_qr_codes():
    """Thread function to continuously scan for QR codes"""
    global camera_running, last_qr_data
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
            if data and data != last_qr_data:
                print(f"QR Code detected: {data}")
                last_qr_data = data
                # Process the QR code data in the main thread
                process_qr_code(data)
        except Exception as e:
            print(f"QR detection error: {e}")
            
        time.sleep(0.1)  # Small delay to reduce CPU usage

def select_sequence_for_box(box_info):
    """Select appropriate sequence based on box properties"""
    # For the specific 3.5x4x9 cm box, always use the small_box sequence
    return 'small_box'

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
    if qr_data.startswith('BOX_'):
        # Extract box information
        parts = qr_data.split('_')
        box_id = parts[1]
        
        # Connect to database to get box information
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM boxes WHERE id = ?", (box_id,))
        box_info = cursor.fetchone()
        
        if box_info:
            # Update box status to "processing"
            cursor.execute("UPDATE boxes SET status = ? WHERE id = ?", ("processing", box_id))
            conn.commit()
            
            # Update robot state
            update_robot_state(status="picking", action=f"Picking up box: {box_id}")
            
            # Select appropriate sequence based on box properties
            sequence = select_sequence_for_box(box_info)
            
            # Execute the selected sequence
            print(f"Executing sequence '{sequence}' for box {box_id}")
            result = execute_sequence(sequence)
            
            # Update robot state to indicate carrying the box
            update_robot_state(status="carrying", carrying_box=box_id)
            
            # Update box status to "delivered"
            cursor.execute("UPDATE boxes SET status = ? WHERE id = ?", ("delivered", box_id))
            conn.commit()
            
            print(f"Box {box_id} processed: {result}")
        else:
            print(f"Unknown box ID: {box_id}")
            update_robot_state(error=f"Unknown box ID: {box_id}")
            
        conn.close()
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
        .video-container { width: 640px; height: 480px; margin: 20px; border: 1px solid #ccc; }
        button { width: 200px; height: 50px; margin: 10px; font-size: 18px; }
        .back-button { background-color: #f0f0f0; }
        .control-panel { margin: 20px; }
        .qr-data { margin: 20px; padding: 10px; background-color: #f0f0f0; border-radius: 5px; }
        .camera-config { margin: 20px; padding: 10px; background-color: #f0f0f0; border-radius: 5px; }
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
        
        function checkQrStatus() {
            fetch('/qr_status')
                .then(response => response.json())
                .then(data => {
                    if (data.qr_data) {
                        document.getElementById('qr_data').innerText = data.qr_data;
                    }
                });
            setTimeout(checkQrStatus, 1000);
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
        </div>
        
        <div class="control-panel">
            <button onclick="startCamera()">Start Camera</button>
            <button onclick="stopCamera()">Stop Camera</button>
            <div class="camera-config">
                <h3>Camera Configuration</h3>
                <input type="text" id="camera_device" placeholder="Camera device (e.g., 0, 1, /dev/video0)">
                <button onclick="setCamera()">Set Camera</button>
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
        
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
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
    return jsonify({"qr_data": last_qr_data})

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
        </script>
    </head>
    <body>
        <h1>Manual Sequence Execution</h1>
        <a href="/"><button class="back-button">Back to Main Menu</button></a>
        
        <div>
            <h2>Available Sequences</h2>
            {sequences_html}
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