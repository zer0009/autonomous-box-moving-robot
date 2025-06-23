# Robot System with Web Interface

This system consists of two main components:
1. **Robot Master Controller** - Controls the robot hardware
2. **Web Server** - Provides a web interface for monitoring and controlling the robot

## Quick Start

### Windows
Simply double-click the `run_robot_system.bat` file to start both components in separate terminal windows.

### Linux/Mac
1. Make the script executable (one-time setup):
   ```
   chmod +x run_robot_system.sh
   chmod +x run_robot_system.py
   ```

2. Run the launcher:
   ```
   ./run_robot_system.sh
   ```

## Command Line Options

You can also run the launcher with various options:

```
python run_robot_system.py [options]
```

Available options:
- `--web-only` - Only launch the web server
- `--robot-only` - Only launch the robot controller
- `--debug` - Enable debug mode for robot controller
- `--nav-only` - Run robot controller in navigation-only mode
- `--arm-only` - Run robot controller in arm-only mode
- `--port PORT` - Port for web server (default: 5000)
- `--no-browser` - Don't open web browser automatically

Examples:
- Run only the web server: `python run_robot_system.py --web-only`
- Run only the robot controller: `python run_robot_system.py --robot-only`
- Run web server on port 8080: `python run_robot_system.py --port 8080`
- Run in navigation-only mode: `python run_robot_system.py --nav-only`

## Running Components Manually

If you prefer to run the components manually in separate terminals:

### Web Server
```
python robot_web_server.py --port 5000
```

### Robot Controller
```
python robot_master_controller.py [--debug]
```

With environment variables:
```
# Navigation only mode
NAV_ONLY=1 python robot_master_controller.py

# Arm only mode
ARM_ONLY=1 python robot_master_controller.py
```

## Troubleshooting

1. **Port already in use**
   - Use a different port: `python run_robot_system.py --port 8080`
   
2. **Controller not connecting to hardware**
   - Try running in navigation-only mode: `python run_robot_system.py --nav-only`
   - Try running in arm-only mode: `python run_robot_system.py --arm-only`
   
3. **Debug information**
   - Run with debug flag: `python run_robot_system.py --debug` 