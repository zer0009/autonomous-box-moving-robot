# Autonomous Box-Moving Robot System

This system controls an autonomous robot that can detect, pick up, and deliver boxes to designated shelves using QR code recognition.

## System Components

1. **Robot Master Controller** (`robot_master_controller.py`) - Central control system running on Raspberry Pi
2. **ESP32 Navigation Controller** (`esp32_navigation_controller.ino`) - Controls robot movement and sensors
3. **ESP32 Arm Controller** (`esp32_arm_controller.ino`) - Controls robotic arm for box manipulation
4. **QR Code Generator** (`qr_code_generator.py`) - Creates QR codes for boxes, shelves, and floor markers
5. **Web Interface** (`robot_web_server.py`) - Flask server providing mobile monitoring and control
6. **Integration Module** (`robot_integration.py`) - Connects the robot controller to the web interface

## File Structure

- `robot_master_controller.py` - Main robot control logic
- `esp32_navigation_controller.ino` - ESP32 navigation firmware
- `esp32_arm_controller.ino` - ESP32 arm control firmware
- `qr_code_generator.py` - QR code generation tool
- `robot_web_server.py` - Web interface for monitoring and control
- `robot_integration.py` - Integration between controller and web interface
- `start_robot_system.py` - Script to start all components together
- `requirements.txt` - Python dependencies
- `robot_tasks.db` - SQLite database for task tracking (created automatically)
- `generated_qr_codes/` - Directory for generated QR codes
- `templates/` - HTML templates for web interface

## Setup Instructions

### 1. Install Python Dependencies

```bash
pip install -r requirements.txt
```

### 2. Flash ESP32 Controllers (if using real hardware)

Flash the navigation and arm controller firmware to two ESP32 boards:

```bash
# Using Arduino IDE or PlatformIO
# Flash esp32_navigation_controller.ino to the navigation ESP32
# Flash esp32_arm_controller.ino to the arm control ESP32
```

### 3. Generate QR Codes

Generate QR codes for boxes, shelves, and floor markers:

```bash
# Generate a box QR code
python qr_code_generator.py --type box --id BOX123 --shelf A --weight 2.5

# Generate a floor marker
python qr_code_generator.py --type floor --x 5 --y 10

# Generate a shelf marker
python qr_code_generator.py --type shelf --id A

# Generate a batch of test QR codes
python qr_code_generator.py --type batch
```

QR codes will be saved in the `generated_qr_codes` directory.

### 4. Start the Complete System

#### Option 1: Start all components with one command (recommended)

```bash
# Start in simulation mode (no hardware required)
python start_robot_system.py

# Start with real hardware connections
python start_robot_system.py --no-simulation
```

#### Option 2: Start components individually

Start the web server:
```bash
python robot_web_server.py
```

Start the robot controller:
```bash
# In simulation mode (no hardware required)
python robot_master_controller.py --simulation

# With real hardware
python robot_master_controller.py
```

### 5. Access the Web Interface

Access the web interface by opening the provided URL in a browser:
```
http://localhost:5000
```

Or from another device on the same network:
```
http://<raspberry-pi-ip>:5000
```

## Running on Raspberry Pi

### Setting up the Raspberry Pi

1. Install Raspberry Pi OS (Lite or Desktop version)

2. Install required packages:
```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv libzbar0 libopencv-dev
```

3. Clone or copy the project to the Pi:
```bash
git clone <repository-url> /home/pi/robot-system
cd /home/pi/robot-system
```

4. Install Python dependencies:
```bash
pip3 install -r requirements.txt
```

### Running in Simulation Mode (without ESP32 controllers)

This mode allows you to test the web interface and software functionality without connecting the ESP32 controllers:

```bash
python3 start_robot_system.py
```

The system will:
- Start the robot controller in simulation mode
- Start the web server
- Simulate robot movements and actions
- Allow full testing of the web interface

### Running with Real Hardware

Connect the ESP32 controllers and camera, then:

```bash
python3 start_robot_system.py --no-simulation
```

### Running with Limited Hardware

The system can operate with only one of the controllers connected:

```bash
# Run with only navigation controller (no arm controller needed)
python3 start_robot_system.py --no-simulation --nav-only

# Run with only arm controller (no navigation controller needed)
python3 start_robot_system.py --no-simulation --arm-only

# Run only the web interface without any controllers
python3 start_robot_system.py --web-only
```

The web interface will clearly indicate which controllers are connected or required.

### Autostart on Boot (optional)

To start the robot system automatically when the Raspberry Pi boots:

1. Create a systemd service file:
```bash
sudo nano /etc/systemd/system/robot-system.service
```

2. Add the following content:
```
[Unit]
Description=Autonomous Box-Moving Robot System
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/robot-system
ExecStart=/usr/bin/python3 /home/pi/robot-system/start_robot_system.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

3. Enable and start the service:
```bash
sudo systemctl enable robot-system.service
sudo systemctl start robot-system.service
```

4. Check status:
```bash
sudo systemctl status robot-system.service
```

## Hardware Connections

### Raspberry Pi

- Camera: USB or Raspberry Pi Camera module
- ESP32 Navigation Controller: UART (/dev/ttyUSB0 or similar)
- ESP32 Arm Controller: UART (/dev/ttyUSB1 or similar)

### ESP32 Navigation Controller

- L298N Motor Driver:
  - Motor Left PWM: GPIO12
  - Motor Left DIR1: GPIO14
  - Motor Left DIR2: GPIO27
  - Motor Right PWM: GPIO13
  - Motor Right DIR1: GPIO26
  - Motor Right DIR2: GPIO25

- Ultrasonic Sensors:
  - Front Trigger: GPIO5
  - Front Echo: GPIO18
  - Left Trigger: GPIO19
  - Left Echo: GPIO21
  - Right Trigger: GPIO22
  - Right Echo: GPIO23

- Encoders (optional):
  - Left A: GPIO34
  - Left B: GPIO35
  - Right A: GPIO32
  - Right B: GPIO33

### ESP32 Arm Controller

- Servos:
  - Base: GPIO12
  - Shoulder: GPIO13
  - Elbow: GPIO14
  - Wrist: GPIO27
  - Gripper: GPIO26

- Limit Switches:
  - Grip Switch 1: GPIO34
  - Grip Switch 2: GPIO35

## Database Structure

The system uses SQLite for data storage with the following tables:

1. **boxes** - Information about boxes being moved
   - id, status, source_position, destination_shelf, weight, etc.

2. **robot_log** - Robot operation logs
   - timestamp, event_type, description, position, success, etc.

3. **qr_codes** - Record of generated QR codes
   - id, qr_type, data, description, created_time

## QR Code Formats

- Box Format: `BOX_ID123_SHELF_A_WEIGHT_2.5`
- Floor Marker: `FLOOR_X10_Y15`
- Shelf: `SHELF_A`

## System Operation

1. The robot scans for QR codes using its camera
2. When a box QR is detected, it parses destination information
3. The robot picks up the box using the robotic arm
4. Navigation system plans a path to the destination shelf
5. Box is delivered and placed on the correct shelf
6. Robot returns to home position for the next task

## Mobile Interface

Access the web interface on a mobile device by navigating to the server's IP address.

1. **Dashboard**: View robot status, pending tasks, and logs
2. **Control**: Manually control robot movement
3. **QR Codes**: Manage QR codes

## Simulation Mode

The system can run in simulation mode without hardware:
- Robot controller simulates hardware responses
- Integration module connects to the web server without a physical robot
- Web interface functions fully for monitoring and control

## Troubleshooting

1. **USB Connection Issues**
   - The system automatically tries multiple USB ports (ttyUSB0, ttyUSB1, COM3, COM4)
   - Confirm connections and permissions with `ls -l /dev/ttyUSB*`

2. **Camera Detection**
   - The system tries multiple camera indices (0, 1, 2)
   - Check camera connections with `ls -l /dev/video*`