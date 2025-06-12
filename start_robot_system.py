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

def start_robot_controller(simulation=True):
    """Start the robot master controller in a separate process"""
    cmd = [sys.executable, "robot_master_controller.py"]
    if simulation:
        cmd.append("--simulation")
    
    print(f"Starting robot controller: {' '.join(cmd)}")
    return subprocess.Popen(cmd)

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
    args = parser.parse_args()
    
    simulation_mode = not args.no_simulation
    
    processes = []
    
    try:
        # Register cleanup function
        atexit.register(cleanup_processes, processes)
        
        # Start robot controller
        controller_process = start_robot_controller(simulation=simulation_mode)
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