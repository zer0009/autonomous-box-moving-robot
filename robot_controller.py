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
    'pick_up_box': [
        ('Base +', 0.5),      # Position base
        ('Shoulder +', 0.5),  # Lower arm
        ('Elbow -', 0.5),     # Extend arm
        ('Gripper Open', 0.5),# Open gripper
        ('Elbow +', 0.2),     # Move slightly down to box
        ('Gripper Close', 1), # Grip box
        ('Shoulder -', 0.5),  # Lift box
        ('Base -', 1),        # Turn to robot back
        ('Elbow +', 0.5),     # Position over storage area
        ('Gripper Open', 0.5),# Release box
        ('Shoulder +', 0.3),  # Move away from box
        ('Base +', 0.5),      # Return to starting position
    ]
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

def get_camera():
    """Initialize camera if not already done"""
    global camera
    if camera is None:
        camera = cv2.VideoCapture(0)  # Use default camera
        if not camera.isOpened():
            print("Error: Could not open camera")
            return None
    return camera

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
        conn.close()
        
        if box_info:
            print(f"Processing box {box_id}")
            # Execute the pick up box sequence
            execute_sequence('pick_up_box')
        else:
            print(f"Unknown box ID: {box_id}")

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
        return f"Unknown sequence: {sequence_name}"
    
    sequence = SEQUENCES[sequence_name]
    for cmd, delay in sequence:
        print(f"Executing: {cmd}")
        send_command(cmd)
        time.sleep(delay)
    
    return f"Sequence {sequence_name} completed"

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

@app.route('/execute_sequence/<sequence_name>')
def api_execute_sequence(sequence_name):
    """API endpoint to execute a predefined sequence"""
    result = execute_sequence(sequence_name)
    return jsonify({"result": result})

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