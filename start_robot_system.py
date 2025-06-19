#!/usr/bin/env python3
"""
Start the complete robot system - both controller and web server
Can run in simulation mode without hardware connections
"""

import os
import sys
import time
import threading
import subprocess
import argparse
import signal
import atexit

def start_robot_controller(simulation=True, skip_camera=False, debug=False, env_vars=None):
    """Start the robot master controller in a separate process"""
    cmd = [sys.executable, "robot_master_controller.py"]
    if simulation:
        cmd.append("--simulation")
    if debug:
        cmd.append("--debug")
    
    env = os.environ.copy()
    if skip_camera:
        env["SKIP_CAMERA"] = "1"
    
    # Add any additional environment variables
    if env_vars:
        env.update(env_vars)
    
    print(f"Starting robot controller: {' '.join(cmd)}")
    if env_vars:
        print(f"With environment variables: {env_vars}")
    return subprocess.Popen(cmd, env=env)

def start_web_server():
    """Start the web server in a separate process"""
    cmd = [sys.executable, "robot_web_server.py"]
    print(f"Starting web server: {' '.join(cmd)}")
    return subprocess.Popen(cmd)

def cleanup_processes(processes):
    """Clean up all processes on exit"""
    print("Shutting down all processes...")
    for process in processes:
        if process and process.poll() is None:  # If process is still running
            try:
                process.terminate()
                process.wait(timeout=5)  # Wait up to 5 seconds for graceful termination
            except subprocess.TimeoutExpired:
                process.kill()  # Force kill if it doesn't terminate
            except Exception as e:
                print(f"Error terminating process: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Start the robot system')
    parser.add_argument('--no-simulation', action='store_true', 
                        help='Run with real hardware (default: simulation mode)')
    parser.add_argument('--skip-camera', action='store_true',
                        help='Skip camera detection (useful for headless operation)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging for ESP32 communication')
    parser.add_argument('--nav-port', 
                        help='Specify serial port for navigation controller (e.g., /dev/ttyUSB0)')
    parser.add_argument('--arm-port', 
                        help='Specify serial port for arm controller (e.g., /dev/ttyACM0)')
    args = parser.parse_args()
    
    simulation_mode = not args.no_simulation
    skip_camera = args.skip_camera
    debug_mode = args.debug
    nav_port = args.nav_port
    arm_port = args.arm_port
    
    # Set environment variables for ports if specified
    env_vars = {}
    if nav_port:
        env_vars["NAV_PORT"] = nav_port
    if arm_port:
        env_vars["ARM_PORT"] = arm_port
    
    processes = []
    
    try:
        # Register cleanup function
        atexit.register(cleanup_processes, processes)
        
        # Start robot controller
        controller_process = start_robot_controller(simulation=simulation_mode, skip_camera=skip_camera, debug=debug_mode, env_vars=env_vars)
        processes.append(controller_process)
        
        # Give the controller a moment to initialize
        time.sleep(2)
        
        # Start web server
        web_process = start_web_server()
        processes.append(web_process)
        
        print("Robot system started successfully!")
        print("Press Ctrl+C to stop all components")
        
        # Wait for processes to complete (which they won't unless there's an error)
        while all(p.poll() is None for p in processes):
            time.sleep(1)
            
        # Check if any process exited
        for p in processes:
            if p.poll() is not None:
                print(f"Process exited with code {p.returncode}")
                break
                
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cleanup_processes(processes) 