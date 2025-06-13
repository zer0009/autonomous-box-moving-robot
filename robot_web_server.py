#!/usr/bin/env python3
"""
Box-Moving Robot - Web Interface
Flask web server to monitor and control the robot via mobile devices
"""

from flask import Flask, render_template, request, jsonify, send_from_directory
import sqlite3
import os
import time
import json
import threading
import socket
import logging
from datetime import datetime

app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot_secret_key'
app.config['DATABASE'] = 'robot_tasks.db'
app.config['QR_CODE_DIR'] = 'generated_qr_codes'

# Robot connection status
robot_status = {
    "connected": False,
    "last_update": None,
    "position": (0, 0),
    "orientation": 0,
    "robot_busy": False,
    "pending_tasks": 0,
    "completed_tasks": 0
}

# Robot command queue for remote control
command_queue = []

# Ensure directories exist
os.makedirs('static', exist_ok=True)
os.makedirs('templates', exist_ok=True)
os.makedirs(app.config['QR_CODE_DIR'], exist_ok=True)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("robot_server.log", encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

def get_db_connection():
    """Create a database connection"""
    conn = sqlite3.connect(app.config['DATABASE'])
    conn.row_factory = sqlite3.Row
    return conn

@app.route('/')
def index():
    """Main dashboard page"""
    try:
        conn = get_db_connection()
        
        # Check if created_time column exists
        cursor = conn.cursor()
        cursor.execute("PRAGMA table_info(boxes)")
        columns = [column[1] for column in cursor.fetchall()]
        has_created_time = 'created_time' in columns
        
        # Get pending tasks
        if has_created_time:
            pending_tasks = conn.execute('''
                SELECT * FROM boxes WHERE status != 'delivered'
                ORDER BY created_time DESC
            ''').fetchall()
        else:
            # Fallback if created_time doesn't exist
            pending_tasks = conn.execute('''
                SELECT * FROM boxes WHERE status != 'delivered'
                ORDER BY pickup_time DESC
            ''').fetchall()
        
        # Get recent completed tasks
        completed_tasks = conn.execute('''
            SELECT * FROM boxes WHERE status = 'delivered'
            ORDER BY delivery_time DESC LIMIT 5
        ''').fetchall()
        
        # Get recent log events
        logs = conn.execute('''
            SELECT * FROM robot_log
            ORDER BY timestamp DESC LIMIT 10
        ''').fetchall()
        
        conn.close()
        
        return render_template('index.html', 
                               pending_tasks=pending_tasks,
                               completed_tasks=completed_tasks,
                               logs=logs,
                               robot_status=robot_status)
    except Exception as e:
        logger.error(f"Error in index route: {e}")
        return render_template('error.html', error=str(e))

@app.route('/qr_codes')
def qr_codes():
    """QR code management page"""
    try:
        conn = get_db_connection()
        
        # Check if qr_codes table exists
        cursor = conn.cursor()
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='qr_codes'")
        table_exists = cursor.fetchone() is not None
        
        if table_exists:
            # Check if created_time column exists
            cursor.execute("PRAGMA table_info(qr_codes)")
            columns = [column[1] for column in cursor.fetchall()]
            has_created_time = 'created_time' in columns
            
            if has_created_time:
                qr_codes = conn.execute('''
                    SELECT * FROM qr_codes
                    ORDER BY created_time DESC
                ''').fetchall()
            else:
                qr_codes = conn.execute('''
                    SELECT * FROM qr_codes
                ''').fetchall()
        else:
            # Create the table
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS qr_codes (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    code_type TEXT,
                    content TEXT,
                    filename TEXT,
                    created_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            conn.commit()
            qr_codes = []
            
        conn.close()
        
        return render_template('qr_codes.html', qr_codes=qr_codes)
    except Exception as e:
        logger.error(f"Error in qr_codes route: {e}")
        return render_template('error.html', error=str(e))

@app.route('/control')
def control():
    """Robot control page"""
    return render_template('control.html', robot_status=robot_status)

@app.route('/api/status')
def api_status():
    """API endpoint to get robot status"""
    try:
        conn = get_db_connection()
        
        # Get pending and completed counts
        pending = conn.execute('SELECT COUNT(*) FROM boxes WHERE status != "delivered"').fetchone()[0]
        completed = conn.execute('SELECT COUNT(*) FROM boxes WHERE status = "delivered"').fetchone()[0]
        
        conn.close()
        
        status = {
            "connected": robot_status["connected"],
            "last_update": robot_status["last_update"],
            "position": robot_status["position"],
            "orientation": robot_status["orientation"],
            "robot_busy": robot_status["robot_busy"],
            "pending_tasks": pending,
            "completed_tasks": completed
        }
        
        return jsonify(status)
    except Exception as e:
        logger.error(f"Error in api_status: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/api/send_command', methods=['POST'])
def api_send_command():
    """API endpoint to send commands to robot"""
    try:
        data = request.get_json()
        if not data or 'command' not in data:
            return jsonify({"error": "Invalid command format"}), 400
        
        command = data['command']
        params = data.get('params', {})
        
        # Add command to queue
        command_queue.append({
            "command": command,
            "params": params,
            "timestamp": time.time(),
            "status": "pending"
        })
        
        return jsonify({"status": "Command added to queue", "queue_position": len(command_queue)})
    except Exception as e:
        logger.error(f"Error in api_send_command: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/api/logs')
def api_logs():
    """API endpoint to get robot logs"""
    try:
        conn = get_db_connection()
        limit = request.args.get('limit', 20, type=int)
        
        logs = conn.execute('''
            SELECT * FROM robot_log
            ORDER BY timestamp DESC LIMIT ?
        ''', (limit,)).fetchall()
        
        conn.close()
        
        # Convert to list of dicts
        logs_list = [{
            "timestamp": row['timestamp'],
            "event_type": row['event_type'],
            "description": row['description'],
            "position": row['position'],
            "success": bool(row['success'])
        } for row in logs]
        
        return jsonify({"logs": logs_list})
    except Exception as e:
        logger.error(f"Error in api_logs: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/api/tasks')
def api_tasks():
    """API endpoint to get task list"""
    try:
        conn = get_db_connection()
        status = request.args.get('status', 'all')
        limit = request.args.get('limit', 20, type=int)
        
        # Check if created_time column exists
        cursor = conn.cursor()
        cursor.execute("PRAGMA table_info(boxes)")
        columns = [column[1] for column in cursor.fetchall()]
        has_created_time = 'created_time' in columns
        
        if status == 'pending':
            if has_created_time:
                query = '''SELECT * FROM boxes WHERE status != 'delivered' 
                          ORDER BY created_time DESC LIMIT ?'''
            else:
                query = '''SELECT * FROM boxes WHERE status != 'delivered' 
                          ORDER BY pickup_time DESC LIMIT ?'''
        elif status == 'completed':
            query = '''SELECT * FROM boxes WHERE status = 'delivered' 
                      ORDER BY delivery_time DESC LIMIT ?'''
        else:
            if has_created_time:
                query = '''SELECT * FROM boxes ORDER BY created_time DESC LIMIT ?'''
            else:
                query = '''SELECT * FROM boxes ORDER BY pickup_time DESC LIMIT ?'''
            
        tasks = conn.execute(query, (limit,)).fetchall()
        conn.close()
        
        # Convert to list of dicts
        tasks_list = [{
            "id": row['id'],
            "status": row['status'],
            "source_position": row['source_position'],
            "destination_shelf": row['destination_shelf'],
            "destination_section": row['destination_section'] if 'destination_section' in row.keys() else None,
            "stack_position": row['stack_position'] if 'stack_position' in row.keys() else 0,
            "pickup_time": row['pickup_time'],
            "delivery_time": row['delivery_time'],
            "weight": row['weight'],
            "created_time": row['created_time'] if has_created_time and 'created_time' in row.keys() else None
        } for row in tasks]
        
        return jsonify({"tasks": tasks_list})
    except Exception as e:
        logger.error(f"Error in api_tasks: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/api/update_robot_status', methods=['POST'])
def update_robot_status():
    """API endpoint for robot to update its status (called by robot)"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "Invalid data format"}), 400
        
        robot_status["connected"] = True
        robot_status["last_update"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        robot_status["position"] = data.get("current_position", robot_status["position"])
        robot_status["orientation"] = data.get("current_orientation", robot_status["orientation"])
        robot_status["robot_busy"] = data.get("robot_busy", robot_status["robot_busy"])
        robot_status["pending_tasks"] = data.get("pending_tasks", robot_status["pending_tasks"])
        robot_status["completed_tasks"] = data.get("completed_tasks", robot_status["completed_tasks"])
        
        # Check for pending commands
        if command_queue:
            return jsonify({
                "status": "ok",
                "has_commands": True,
                "commands": command_queue[0]  # Send the oldest command
            })
        else:
            return jsonify({
                "status": "ok",
                "has_commands": False
            })
    except Exception as e:
        logger.error(f"Error in update_robot_status: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/api/command_result', methods=['POST'])
def command_result():
    """API endpoint for robot to report command execution result"""
    try:
        data = request.get_json()
        if not data or 'command_id' not in data:
            return jsonify({"error": "Invalid result format"}), 400
            
        # Remove the command from queue
        if command_queue:
            command_queue.pop(0)
            
        return jsonify({"status": "ok"})
    except Exception as e:
        logger.error(f"Error in command_result: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/qr/<filename>')
def serve_qr(filename):
    """Serve QR code images"""
    return send_from_directory(app.config['QR_CODE_DIR'], filename)

@app.route('/qr_viewer/<filename>')
def qr_viewer(filename):
    """Mobile-friendly QR code viewer for printing"""
    return render_template('qr_viewer.html', filename=filename)

@app.route('/api/shelf_sections')
def api_shelf_sections():
    """API endpoint to get shelf sections data"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        
        sections = cursor.execute('''
            SELECT shelf_id, section_id, capacity, occupied, last_updated
            FROM shelf_sections
            ORDER BY shelf_id, section_id
        ''').fetchall()
        
        conn.close()
        
        # Convert to list of dicts
        sections_list = [{
            "shelf_id": row['shelf_id'],
            "section_id": row['section_id'],
            "capacity": row['capacity'],
            "occupied": row['occupied'],
            "utilization": (row['occupied'] / row['capacity']) * 100 if row['capacity'] > 0 else 0,
            "last_updated": row['last_updated']
        } for row in sections]
        
        # Group by shelf
        shelves = {}
        for section in sections_list:
            shelf_id = section['shelf_id']
            if shelf_id not in shelves:
                shelves[shelf_id] = []
            shelves[shelf_id].append(section)
        
        return jsonify({
            "sections": sections_list,
            "shelves": shelves
        })
    except Exception as e:
        logger.error(f"Error in api_shelf_sections: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/shelves')
def shelves():
    """Shelf management page"""
    return render_template('shelves.html')

def connection_monitor():
    """Monitor thread to check robot connection status"""
    while True:
        if robot_status["last_update"]:
            last_update_time = datetime.strptime(robot_status["last_update"], "%Y-%m-%d %H:%M:%S")
            time_diff = (datetime.now() - last_update_time).total_seconds()
            
            # Mark as disconnected if no update in 30 seconds
            if time_diff > 30:
                robot_status["connected"] = False
                
        time.sleep(5)

# Create basic HTML templates for the app
def create_templates():
    """Create template files if they don't exist"""
    # Create templates directory if it doesn't exist
    if not os.path.exists('templates'):
        os.makedirs('templates')
    
    # Create qr_viewer.html for easy printing
    with open('templates/qr_viewer.html', 'w', encoding='utf-8') as f:
        f.write('''
<!DOCTYPE html>
<html>
<head>
    <title>QR Code Viewer</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
            margin: 0;
            padding: 20px;
            box-sizing: border-box;
            font-family: Arial, sans-serif;
        }
        .qr-container {
            text-align: center;
            max-width: 100%;
        }
        .qr-image {
            max-width: 100%;
            height: auto;
            margin-bottom: 20px;
        }
        .button-container {
            margin-top: 20px;
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            justify-content: center;
        }
        .button {
            background-color: #007bff;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            text-decoration: none;
            font-size: 16px;
        }
        .button.print {
            background-color: #28a745;
        }
        .button.back {
            background-color: #6c757d;
        }
        @media print {
            .button-container {
                display: none;
            }
        }
    </style>
</head>
<body>
    <div class="qr-container">
        <img src="/qr/{{ filename }}" alt="QR Code" class="qr-image">
        <div class="button-container">
            <button class="button print" onclick="window.print()">Print QR Code</button>
            <a href="/qr_codes" class="button back">Back to QR Codes</a>
        </div>
    </div>
</body>
</html>
        ''')
    
    # Create index.html
    with open('templates/index.html', 'w', encoding='utf-8') as f:
        f.write('''
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control System</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/css/bootstrap.min.css">
    <style>
        .status-indicator {
            display: inline-block;
            width: 15px;
            height: 15px;
            border-radius: 50%;
            margin-right: 5px;
        }
        .status-connected { background-color: #28a745; }
        .status-disconnected { background-color: #dc3545; }
        .shelf-section {
            width: 80px;
            height: 80px;
            border: 1px solid #ccc;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            margin: 5px;
            text-align: center;
            font-weight: bold;
            position: relative;
        }
        .section-capacity {
            position: absolute;
            top: 5px;
            right: 5px;
            font-size: 0.8em;
            font-weight: normal;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-dark">
        <div class="container">
            <a class="navbar-brand" href="/">Box-Moving Robot</a>
            <button class="navbar-toggler" type="button" data-bs-toggle="collapse" data-bs-target="#navbarNav">
                <span class="navbar-toggler-icon"></span>
            </button>
            <div class="collapse navbar-collapse" id="navbarNav">
                <ul class="navbar-nav">
                    <li class="nav-item">
                        <a class="nav-link active" href="/">Dashboard</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link" href="/control">Control</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link" href="/qr_codes">QR Codes</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link" href="/shelves">Shelves</a>
                    </li>
                </ul>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <div class="row">
            <div class="col-md-6">
                <div class="card mb-4">
                    <div class="card-header">
                        Robot Status
                        {% if robot_status.connected %}
                        <span class="status-indicator status-connected"></span> Connected
                        {% else %}
                        <span class="status-indicator status-disconnected"></span> Disconnected
                        {% endif %}
                    </div>
                    <div class="card-body">
                        <p><strong>Position:</strong> {{ robot_status.position }}</p>
                        <p><strong>Orientation:</strong> {{ robot_status.orientation }}°</p>
                        <p><strong>Status:</strong> {{ "Busy" if robot_status.robot_busy else "Idle" }}</p>
                        <p><strong>Tasks:</strong> {{ robot_status.pending_tasks }} pending, {{ robot_status.completed_tasks }} completed</p>
                        {% if robot_status.carrying_boxes is defined %}
                        <p><strong>Carrying:</strong> {{ robot_status.carrying_boxes }} / {{ robot_status.max_box_capacity or 2 }} boxes</p>
                        {% endif %}
                        <p><strong>Last Update:</strong> {{ robot_status.last_update or "Never" }}</p>
                    </div>
                </div>
            </div>
            
            <div class="col-md-6">
                <div class="card mb-4">
                    <div class="card-header">Pending Tasks</div>
                    <div class="card-body">
                        <table class="table table-sm">
                            <thead>
                                <tr>
                                    <th>Box ID</th>
                                    <th>Destination</th>
                                    <th>Section</th>
                                    <th>Weight</th>
                                    <th>Status</th>
                                </tr>
                            </thead>
                            <tbody>
                                {% for task in pending_tasks %}
                                <tr>
                                    <td>{{ task.id }}</td>
                                    <td>{{ task.destination_shelf }}</td>
                                    <td>{{ task.destination_section }}</td>
                                    <td>{{ task.weight }} kg</td>
                                    <td>{{ task.status }}</td>
                                </tr>
                                {% else %}
                                <tr>
                                    <td colspan="5" class="text-center">No pending tasks</td>
                                </tr>
                                {% endfor %}
                            </tbody>
                        </table>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="row">
            <div class="col-md-6">
                <div class="card mb-4">
                    <div class="card-header">Recent Completed Tasks</div>
                    <div class="card-body">
                        <table class="table table-sm">
                            <thead>
                                <tr>
                                    <th>Box ID</th>
                                    <th>Destination</th>
                                    <th>Section</th>
                                    <th>Completion Time</th>
                                </tr>
                            </thead>
                            <tbody>
                                {% for task in completed_tasks %}
                                <tr>
                                    <td>{{ task.id }}</td>
                                    <td>{{ task.destination_shelf }}</td>
                                    <td>{{ task.destination_section }}</td>
                                    <td>{{ task.delivery_time }}</td>
                                </tr>
                                {% else %}
                                <tr>
                                    <td colspan="4" class="text-center">No completed tasks</td>
                                </tr>
                                {% endfor %}
                            </tbody>
                        </table>
                    </div>
                </div>
            </div>
            
            <div class="col-md-6">
                <div class="card mb-4">
                    <div class="card-header">Recent Logs</div>
                    <div class="card-body">
                        <table class="table table-sm">
                            <thead>
                                <tr>
                                    <th>Time</th>
                                    <th>Event</th>
                                    <th>Description</th>
                                </tr>
                            </thead>
                            <tbody>
                                {% for log in logs %}
                                <tr class="{{ 'table-danger' if not log.success else '' }}">
                                    <td>{{ log.timestamp.split(' ')[1] if ' ' in log.timestamp else log.timestamp }}</td>
                                    <td>{{ log.event_type }}</td>
                                    <td>{{ log.description }}</td>
                                </tr>
                                {% else %}
                                <tr>
                                    <td colspan="3" class="text-center">No logs available</td>
                                </tr>
                                {% endfor %}
                            </tbody>
                        </table>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <script src="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/js/bootstrap.bundle.min.js"></script>
    <script>
        // Auto-refresh the page every 30 seconds
        setTimeout(function() {
            location.reload();
        }, 30000);
    </script>
</body>
</html>
        ''')
    
    # Create control.html
    with open('templates/control.html', 'w', encoding='utf-8') as f:
        f.write('''
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/css/bootstrap.min.css">
    <style>
        .control-btn {
            width: 80px;
            height: 80px;
            margin: 5px;
            font-size: 24px;
        }
        #robotCanvas {
            border: 1px solid #ccc;
            background-color: #f8f9fa;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-dark">
        <div class="container">
            <a class="navbar-brand" href="/">Box-Moving Robot</a>
            <button class="navbar-toggler" type="button" data-bs-toggle="collapse" data-bs-target="#navbarNav">
                <span class="navbar-toggler-icon"></span>
            </button>
            <div class="collapse navbar-collapse" id="navbarNav">
                <ul class="navbar-nav">
                    <li class="nav-item">
                        <a class="nav-link" href="/">Dashboard</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link active" href="/control">Control</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link" href="/qr_codes">QR Codes</a>
                    </li>
                </ul>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <div class="row">
            <div class="col-md-6">
                <div class="card mb-4">
                    <div class="card-header">Robot Control</div>
                    <div class="card-body">
                        <div class="text-center mb-4">
                            <div>
                                <button class="btn btn-primary control-btn" onclick="sendCommand('move', {direction: 'forward'})">UP</button>
                            </div>
                            <div>
                                <button class="btn btn-primary control-btn" onclick="sendCommand('turn', {direction: 'left'})">LEFT</button>
                                <button class="btn btn-danger control-btn" onclick="sendCommand('stop')">STOP</button>
                                <button class="btn btn-primary control-btn" onclick="sendCommand('turn', {direction: 'right'})">RIGHT</button>
                            </div>
                            <div>
                                <button class="btn btn-primary control-btn" onclick="sendCommand('move', {direction: 'backward'})">DOWN</button>
                            </div>
                        </div>
                        
                        <div class="d-grid gap-2 mb-4">
                            <button class="btn btn-success" onclick="sendCommand('home')">Return to Home</button>
                            <button class="btn btn-warning" onclick="sendCommand('emergency_stop')">EMERGENCY STOP</button>
                        </div>
                    </div>
                </div>
                <div class="card mb-4">
                    <div class="card-header">Command Response</div>
                    <div class="card-body">
                        <pre id="commandResponse">No commands sent yet</pre>
                    </div>
                </div>
            </div>
            
            <div class="col-md-6">
                <div class="card mb-4">
                    <div class="card-header">Robot Position</div>
                    <div class="card-body">
                        <canvas id="robotCanvas" width="300" height="300"></canvas>
                        
                        <div class="mt-3">
                            <p><strong>Position:</strong> <span id="positionText">(0, 0)</span></p>
                            <p><strong>Orientation:</strong> <span id="orientationText">0°</span></p>
                            <p><strong>Status:</strong> <span id="statusText">Unknown</span></p>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <script src="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/js/bootstrap.bundle.min.js"></script>
    <script>
        // Function to send commands to robot
        function sendCommand(command, params = {}) {
            fetch('/api/send_command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    command: command,
                    params: params
                }),
            })
            .then(response => response.json())
            .then(data => {
                document.getElementById('commandResponse').textContent = JSON.stringify(data, null, 2);
            })
            .catch((error) => {
                document.getElementById('commandResponse').textContent = 'Error: ' + error;
            });
        }
        
        // Function to update robot status
        function updateRobotStatus() {
            fetch('/api/status')
            .then(response => response.json())
            .then(data => {
                document.getElementById('positionText').textContent = data.position;
                document.getElementById('orientationText').textContent = data.orientation + '°';
                document.getElementById('statusText').textContent = data.robot_busy ? 'Busy' : 'Idle';
                
                // Draw robot on canvas
                drawRobot(data.position, data.orientation);
            })
            .catch(error => console.error('Error fetching status:', error));
        }
        
        // Function to draw robot on canvas
        function drawRobot(position, orientation) {
            const canvas = document.getElementById('robotCanvas');
            const ctx = canvas.getContext('2d');
            const gridSize = 20; // Grid size from robot controller
            
            // Clear canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Draw grid
            const cellSize = canvas.width / gridSize;
            ctx.strokeStyle = '#ddd';
            ctx.lineWidth = 1;
            
            for (let i = 0; i <= gridSize; i++) {
                // Vertical lines
                ctx.beginPath();
                ctx.moveTo(i * cellSize, 0);
                ctx.lineTo(i * cellSize, canvas.height);
                ctx.stroke();
                
                // Horizontal lines
                ctx.beginPath();
                ctx.moveTo(0, i * cellSize);
                ctx.lineTo(canvas.width, i * cellSize);
                ctx.stroke();
            }
            
            // Draw robot
            const x = position[0] * cellSize + cellSize/2;
            const y = position[1] * cellSize + cellSize/2;
            const radius = cellSize * 0.4;
            
            // Robot body
            ctx.fillStyle = '#007bff';
            ctx.beginPath();
            ctx.arc(x, y, radius, 0, Math.PI * 2);
            ctx.fill();
            
            // Direction indicator
            const radians = orientation * Math.PI / 180;
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(
                x + Math.cos(radians) * radius,
                y + Math.sin(radians) * radius
            );
            ctx.stroke();
            
            // Draw shelves
            const shelves = [
                { name: 'A', position: [18, 5] },
                { name: 'B', position: [18, 10] },
                { name: 'C', position: [18, 15] },
                { name: 'HOME', position: [1, 1] }
            ];
            
            shelves.forEach(shelf => {
                const shelfX = shelf.position[0] * cellSize + cellSize/2;
                const shelfY = shelf.position[1] * cellSize + cellSize/2;
                
                ctx.fillStyle = shelf.name === 'HOME' ? '#28a745' : '#dc3545';
                ctx.fillRect(shelfX - cellSize/3, shelfY - cellSize/3, cellSize/1.5, cellSize/1.5);
                
                ctx.fillStyle = '#ffffff';
                ctx.font = '10px Arial';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(shelf.name, shelfX, shelfY);
            });
        }
        
        // Update status initially and then every 3 seconds
        updateRobotStatus();
        setInterval(updateRobotStatus, 3000);
    </script>
</body>
</html>
        ''')
    
    # Create qr_codes.html
    with open('templates/qr_codes.html', 'w', encoding='utf-8') as f:
        f.write('''
<!DOCTYPE html>
<html>
<head>
    <title>QR Code Management</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/css/bootstrap.min.css">
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-dark">
        <div class="container">
            <a class="navbar-brand" href="/">Box-Moving Robot</a>
            <button class="navbar-toggler" type="button" data-bs-toggle="collapse" data-bs-target="#navbarNav">
                <span class="navbar-toggler-icon"></span>
            </button>
            <div class="collapse navbar-collapse" id="navbarNav">
                <ul class="navbar-nav">
                    <li class="nav-item">
                        <a class="nav-link" href="/">Dashboard</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link" href="/control">Control</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link active" href="/qr_codes">QR Codes</a>
                    </li>
                </ul>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <div class="row">
            <div class="col-12">
                <h2>QR Code Management</h2>
                <p>Use the <code>qr_code_generator.py</code> script to generate new QR codes.</p>
                
                <div class="card mb-4">
                    <div class="card-header">Generated QR Codes</div>
                    <div class="card-body">
                        <div class="row">
                            {% for qr in qr_codes %}
                            <div class="col-md-4 col-sm-6 mb-4">
                                <div class="card">
                                    <div class="card-header">{{ qr.qr_type | capitalize }}</div>
                                    <div class="card-body text-center">
                                        {% set filename = qr.qr_type + '_' + qr.id.lower().replace(' ', '_') + '.png' %}
                                        <img src="/qr/{{ filename }}" class="img-fluid mb-2" alt="{{ qr.id }}" style="max-width: 150px;">
                                        <h5 class="card-title">{{ qr.id }}</h5>
                                        <p class="card-text small">{{ qr.data }}</p>
                                        {% if qr.description %}
                                        <p class="card-text small text-muted">{{ qr.description }}</p>
                                        {% endif %}
                                        <div class="text-center">
                                            <a href="/qr_viewer/{{ filename }}" class="btn btn-sm btn-primary">View</a>
                                            <a href="/qr/{{ filename }}" download class="btn btn-sm btn-secondary">Download</a>
                                        </div>
                                    </div>
                                    <div class="card-footer small text-muted">
                                        Created: {{ qr.created_time }}
                                    </div>
                                </div>
                            </div>
                            {% else %}
                            <div class="col-12">
                                <div class="alert alert-info">
                                    No QR codes have been generated yet. Use the qr_code_generator.py script to create some.
                                </div>
                            </div>
                            {% endfor %}
                        </div>
                    </div>
                </div>
                
                <div class="card mb-4">
                    <div class="card-header">Generate QR Codes</div>
                    <div class="card-body">
                        <h5>Command Examples:</h5>
                        <pre>
# Generate a box QR code
python qr_code_generator.py --type box --id BOX123 --shelf A --weight 2.5

# Generate a floor marker
python qr_code_generator.py --type floor --x 5 --y 10

# Generate a shelf marker
python qr_code_generator.py --type shelf --id A

# Generate a batch of test QR codes
python qr_code_generator.py --type batch
                        </pre>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <script src="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/js/bootstrap.bundle.min.js"></script>
</body>
</html>
        ''')
    
    # Create shelves.html for shelf management
    with open('templates/shelves.html', 'w', encoding='utf-8') as f:
        f.write('''
<!DOCTYPE html>
<html>
<head>
    <title>Shelf Management</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/css/bootstrap.min.css">
    <style>
        .shelf-container {
            display: flex;
            flex-direction: column;
            border: 2px solid #333;
            border-radius: 5px;
            padding: 10px;
            margin-bottom: 20px;
            background-color: #f8f9fa;
        }
        .shelf-name {
            font-weight: bold;
            font-size: 1.2em;
            margin-bottom: 10px;
        }
        .shelf-sections {
            display: flex;
            flex-wrap: wrap;
        }
        .shelf-section {
            width: 100px;
            height: 100px;
            border: 1px solid #ccc;
            margin: 5px;
            padding: 5px;
            text-align: center;
            position: relative;
            display: flex;
            flex-direction: column;
            justify-content: center;
        }
        .section-utilization {
            height: 10px;
            background-color: #28a745;
            position: absolute;
            bottom: 0;
            left: 0;
            transition: width 0.5s ease;
        }
        .section-details {
            font-size: 0.8em;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-dark">
        <div class="container">
            <a class="navbar-brand" href="/">Box-Moving Robot</a>
            <button class="navbar-toggler" type="button" data-bs-toggle="collapse" data-bs-target="#navbarNav">
                <span class="navbar-toggler-icon"></span>
            </button>
            <div class="collapse navbar-collapse" id="navbarNav">
                <ul class="navbar-nav">
                    <li class="nav-item">
                        <a class="nav-link" href="/">Dashboard</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link" href="/control">Control</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link" href="/qr_codes">QR Codes</a>
                    </li>
                    <li class="nav-item">
                        <a class="nav-link active" href="/shelves">Shelves</a>
                    </li>
                </ul>
            </div>
        </div>
    </nav>

    <div class="container mt-4">
        <h2>Shelf Management</h2>
        <p>Overview of shelves and their sections with capacity information</p>
        
        <div id="shelvesContainer">
            <div class="d-flex justify-content-center">
                <div class="spinner-border text-primary" role="status">
                    <span class="visually-hidden">Loading...</span>
                </div>
            </div>
        </div>
        
        <div class="card mt-4">
            <div class="card-header">Boxes by Section</div>
            <div class="card-body">
                <div id="boxesContainer">
                    <div class="text-center">Loading...</div>
                </div>
            </div>
        </div>
    </div>
    
    <script src="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/js/bootstrap.bundle.min.js"></script>
    <script>
        // Fetch shelf data
        function loadShelfData() {
            fetch('/api/shelf_sections')
            .then(response => response.json())
            .then(data => {
                const container = document.getElementById('shelvesContainer');
                container.innerHTML = '';
                
                // Sort shelves by ID
                const shelfIds = Object.keys(data.shelves).sort();
                
                // Create shelf displays
                for (const shelfId of shelfIds) {
                    const sections = data.shelves[shelfId];
                    
                    const shelfDiv = document.createElement('div');
                    shelfDiv.className = 'shelf-container';
                    
                    const shelfName = document.createElement('div');
                    shelfName.className = 'shelf-name';
                    shelfName.textContent = shelfId;
                    shelfDiv.appendChild(shelfName);
                    
                    const sectionsDiv = document.createElement('div');
                    sectionsDiv.className = 'shelf-sections';
                    
                    for (const section of sections) {
                        const sectionDiv = document.createElement('div');
                        sectionDiv.className = 'shelf-section';
                        
                        const sectionId = document.createElement('div');
                        sectionId.textContent = section.section_id;
                        sectionDiv.appendChild(sectionId);
                        
                        const details = document.createElement('div');
                        details.className = 'section-details';
                        details.textContent = `${section.occupied}/${section.capacity}`;
                        sectionDiv.appendChild(details);
                        
                        // Add utilization bar
                        const utilization = document.createElement('div');
                        utilization.className = 'section-utilization';
                        utilization.style.width = `${section.utilization}%`;
                        sectionDiv.appendChild(utilization);
                        
                        sectionsDiv.appendChild(sectionDiv);
                    }
                    
                    shelfDiv.appendChild(sectionsDiv);
                    container.appendChild(shelfDiv);
                }
            })
            .catch(error => {
                console.error('Error fetching shelf data:', error);
                document.getElementById('shelvesContainer').innerHTML = 
                    '<div class="alert alert-danger">Error loading shelf data</div>';
            });
            
            // Load boxes per section
            fetch('/api/tasks')
            .then(response => response.json())
            .then(data => {
                const container = document.getElementById('boxesContainer');
                
                // Group boxes by section
                const boxesBySection = {};
                
                for (const box of data.tasks) {
                    if (box.destination_section && box.destination_shelf) {
                        const key = `${box.destination_shelf}-${box.destination_section}`;
                        if (!boxesBySection[key]) {
                            boxesBySection[key] = [];
                        }
                        boxesBySection[key].push(box);
                    }
                }
                
                // Create boxes table
                let html = '<table class="table table-sm">';
                html += '<thead><tr><th>Shelf</th><th>Section</th><th>Boxes</th></tr></thead><tbody>';
                
                const sectionKeys = Object.keys(boxesBySection).sort();
                
                if (sectionKeys.length === 0) {
                    html += '<tr><td colspan="3" class="text-center">No boxes allocated to sections</td></tr>';
                } else {
                    for (const key of sectionKeys) {
                        const boxes = boxesBySection[key];
                        const [shelf, section] = key.split('-');
                        
                        html += `<tr>
                            <td>${shelf}</td>
                            <td>${section}</td>
                            <td>
                                ${boxes.map(box => `
                                    <span class="badge bg-${box.status === 'delivered' ? 'success' : 'primary'}" 
                                          title="${box.status}">
                                        ${box.id}
                                    </span>
                                `).join(' ')}
                            </td>
                        </tr>`;
                    }
                }
                
                html += '</tbody></table>';
                container.innerHTML = html;
            })
            .catch(error => {
                console.error('Error fetching box data:', error);
                document.getElementById('boxesContainer').innerHTML = 
                    '<div class="alert alert-danger">Error loading box data</div>';
            });
        }
        
        // Load initial data
        document.addEventListener('DOMContentLoaded', loadShelfData);
        
        // Refresh every 30 seconds
        setInterval(loadShelfData, 30000);
    </script>
</body>
</html>
        ''')
    
    # Create error.html
    with open('templates/error.html', 'w', encoding='utf-8') as f:
        f.write('''
<!DOCTYPE html>
<html>
<head>
    <title>Error</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/bootstrap/5.1.3/css/bootstrap.min.css">
</head>
<body>
    <div class="container mt-5">
        <div class="card">
            <div class="card-header bg-danger text-white">
                <h4>Error</h4>
            </div>
            <div class="card-body">
                <p>{{ error }}</p>
                <a href="/" class="btn btn-primary">Return to Dashboard</a>
            </div>
        </div>
    </div>
</body>
</html>
        ''')

# Create robot_log table if it doesn't exist
def init_database():
    """Initialize database with required tables"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # Create robot_log table if it doesn't exist
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS robot_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            event_type TEXT,
            description TEXT,
            position TEXT,
            success INTEGER DEFAULT 1
        )
    ''')
    
    # Check if created_time column exists in boxes table, add it if not
    cursor.execute("PRAGMA table_info(boxes)")
    columns = [column[1] for column in cursor.fetchall()]
    if 'created_time' not in columns:
        try:
            cursor.execute('ALTER TABLE boxes ADD COLUMN created_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP')
            logger.info("Added missing created_time column to boxes table")
            
            # Update existing rows with current timestamp
            cursor.execute('UPDATE boxes SET created_time = CURRENT_TIMESTAMP WHERE created_time IS NULL')
            logger.info("Updated existing rows with timestamp")
        except Exception as e:
            logger.error(f"Error adding created_time column: {e}")
    
    conn.commit()
    conn.close()
    
    logger.info("Database initialized with required tables")

if __name__ == '__main__':
    # Create template files
    create_templates()
    
    # Initialize database
    init_database()
    
    # Start connection monitor thread
    monitor_thread = threading.Thread(target=connection_monitor, daemon=True)
    monitor_thread.start()
    
    try:
        # Find local IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        
        port = 5000
        print(f"Starting Robot Web Server...")
        print(f"Access the web interface at: http://{local_ip}:{port}")
        app.run(host='0.0.0.0', port=port, debug=False)
        
    except Exception as e:
        logger.error(f"Failed to start web server: {e}") 