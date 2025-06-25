#!/usr/bin/env python3
"""
ESP32 Arm Controller Bridge
Connects the Flask web interface to the robot master controller
"""

from flask import Flask, render_template_string, request, redirect, url_for
import serial
import time
import threading
import logging
import argparse
import os

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("arm_bridge.log", encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Default serial port
DEFAULT_PORT = '/dev/ttyACM1'  # or '/dev/ttyACM0' if that's your ESP32 port
BAUDRATE = 9600

# Command mappings
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

app = Flask(__name__)
ser = None
serial_lock = threading.Lock()

# HTML template for the control interface
HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Arm Control GUI</title>
    <style>
        body { font-family: Arial; text-align: center; }
        button { width: 150px; height: 50px; margin: 10px; font-size: 18px; }
        .row { margin-bottom: 20px; }
    </style>
</head>
<body>
    <h1>ESP32 Arm Control</h1>
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

def init_serial(port):
    """Initialize serial connection to ESP32"""
    global ser
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for ESP32 to reset
        logger.info(f"Connected to ESP32 on {port}")
        return True
    except Exception as e:
        logger.error(f"Failed to connect to ESP32: {e}")
        return False

@app.route('/', methods=['GET', 'POST'])
def index():
    """Handle web interface requests"""
    response = ""
    if request.method == 'POST':
        cmd_label = request.form['cmd']
        cmd = ARM_COMMANDS.get(cmd_label)
        if cmd:
            response = send_command(cmd, cmd_label)
    return render_template_string(HTML, response=response)

def send_command(cmd, label="Command"):
    """Send command to ESP32 and return response"""
    global ser
    if not ser:
        return "ERROR: ESP32 not connected"
        
    try:
        with serial_lock:
            ser.write(cmd.encode())
            logger.info(f"Sent command: {label} ({cmd})")
            time.sleep(0.1)
            
            response = ""
            start_time = time.time()
            while time.time() - start_time < 1.0:  # Wait up to 1 second for response
                if ser.in_waiting:
                    line = ser.readline().decode(errors='ignore')
                    response += line
                else:
                    time.sleep(0.1)
                    
            return response if response else f"Command sent: {label}"
    except Exception as e:
        logger.error(f"Error sending command: {e}")
        return f"ERROR: {str(e)}"

def send_command_to_robot_controller(cmd):
    """Send command directly to the robot master controller"""
    # This function would integrate with the robot_master_controller.py
    # For now, it just logs the command
    logger.info(f"Would send to robot controller: {cmd}")
    return "OK"

def main():
    """Main function to run the bridge"""
    parser = argparse.ArgumentParser(description='ESP32 Arm Controller Bridge')
    parser.add_argument('--port', type=str, default=DEFAULT_PORT, help='Serial port for ESP32')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Host to run the server on')
    parser.add_argument('--port-web', type=int, default=8080, help='Port to run the web server on')
    args = parser.parse_args()
    
    if init_serial(args.port):
        print(f"Starting web server on http://{args.host}:{args.port_web}")
        app.run(host=args.host, port=args.port_web, debug=False)
    else:
        print("Failed to initialize serial connection. Exiting.")
        return 1
        
    return 0

if __name__ == '__main__':
    exit(main()) 