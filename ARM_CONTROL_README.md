# ESP32 Arm Control System

This system provides control for the robot arm via ESP32. There are two ways to control the arm:

1. Through the **Robot Master Controller** - The main control system that integrates navigation and arm control
2. Through the **ESP32 Arm Bridge** - A direct interface to the arm controller for testing and development

## Command Mapping

The ESP32 arm controller uses the following command set:

| Command | Key | Description |
|---------|-----|-------------|
| Enable Arm | `z` | Power on the arm |
| Disable Arm | `x` | Power off the arm (safe mode) |
| Base + | `w` | Move base in positive direction |
| Base - | `s` | Move base in negative direction |
| Shoulder + | `a` | Move shoulder in positive direction |
| Shoulder - | `d` | Move shoulder in negative direction |
| Elbow + | `q` | Move elbow in positive direction |
| Elbow - | `e` | Move elbow in negative direction |
| Gripper Open | `i` | Open the gripper |
| Gripper Close | `o` | Close the gripper |

## Running the System

### Option 1: Robot Master Controller

The Robot Master Controller has been updated to work with the new arm control commands. Run it as usual:

```bash
python robot_master_controller.py
```

You can then use the web interface at http://localhost:5000 to control both navigation and arm.

### Option 2: ESP32 Arm Bridge (Direct Control)

For direct control of just the arm (useful for testing), use the ESP32 Arm Bridge:

```bash
python esp32_arm_bridge.py --port /dev/ttyACM1
```

Replace `/dev/ttyACM1` with the appropriate serial port for your ESP32. The web interface will be available at http://localhost:8080.

## Web Interface

The web interface for the Robot Master Controller has been updated to use the new arm commands. The control page now shows:

- **Arm Power**: Enable/Disable the arm
- **Base Control**: Move the base joint
- **Shoulder Control**: Move the shoulder joint
- **Elbow Control**: Move the elbow joint
- **Gripper**: Open/Close the gripper

## Keyboard Controls

When using the Robot Master Controller web interface, you can use these keyboard shortcuts:

- **Navigation**:
  - `W` - Move forward
  - `A` - Turn left
  - `S` - Move backward
  - `D` - Turn right
  - `X` - Stop

- **Arm Control**:
  - `Z` - Enable arm
  - `X` - Disable arm (with Ctrl or Alt key)
  - `W` - Base + (with Ctrl or Alt key)
  - `S` - Base - (with Ctrl or Alt key)
  - `A` - Shoulder + (with Ctrl or Alt key)
  - `D` - Shoulder - (with Ctrl or Alt key)
  - `Q` - Elbow +
  - `E` - Elbow -
  - `I` - Gripper open
  - `O` - Gripper close

## Troubleshooting

If you encounter issues with the arm controller:

1. Check that the ESP32 is properly connected and the correct port is specified
2. Ensure the ESP32 has the correct firmware loaded
3. Try power cycling the ESP32
4. Check the log files for errors:
   - `robot_controller_debug.log` for the master controller
   - `arm_bridge.log` for the direct bridge 