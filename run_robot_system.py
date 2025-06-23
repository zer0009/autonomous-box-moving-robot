#!/usr/bin/env python3
"""
Robot System Launcher
Helps launch the robot controller and web server in separate terminals
"""

import os
import sys
import platform
import subprocess
import argparse
import time
import socket
import webbrowser

def check_port_in_use(port):
    """Check if a port is already in use"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) == 0

def get_local_ip():
    """Get local IP address"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "127.0.0.1"

def launch_terminal(command, title=None):
    """Launch a command in a new terminal window"""
    system = platform.system()
    
    if system == "Windows":
        if title:
            cmd = f'start "Robot {title}" cmd /k "{command}"'
        else:
            cmd = f'start cmd /k "{command}"'
        return subprocess.Popen(cmd, shell=True)
        
    elif system == "Linux":
        # Try different terminal emulators
        terminals = [
            ["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"],
            ["xterm", "-e", f"bash -c '{command}; exec bash'"],
            ["konsole", "-e", f"bash -c '{command}; exec bash'"],
            ["xfce4-terminal", "--command", f"bash -c '{command}; exec bash'"]
        ]
        
        for term_cmd in terminals:
            try:
                return subprocess.Popen(term_cmd)
            except FileNotFoundError:
                continue
        
        print("Could not find a suitable terminal emulator. Please open a terminal and run:")
        print(f"  {command}")
        return None
        
    elif system == "Darwin":  # macOS
        cmd = f"osascript -e 'tell app \"Terminal\" to do script \"{command}\"'"
        return subprocess.Popen(cmd, shell=True)
    
    else:
        print(f"Unsupported operating system: {system}")
        print(f"Please open a terminal and run: {command}")
        return None

def main():
    parser = argparse.ArgumentParser(description="Launch Robot System Components")
    parser.add_argument("--web-only", action="store_true", help="Only launch the web server")
    parser.add_argument("--robot-only", action="store_true", help="Only launch the robot controller")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode for robot controller")
    parser.add_argument("--nav-only", action="store_true", help="Run robot controller in navigation-only mode")
    parser.add_argument("--arm-only", action="store_true", help="Run robot controller in arm-only mode")
    parser.add_argument("--port", type=int, default=5000, help="Port for web server (default: 5000)")
    parser.add_argument("--no-browser", action="store_true", help="Don't open web browser automatically")
    args = parser.parse_args()
    
    # Check if web server port is already in use
    if not args.robot_only and check_port_in_use(args.port):
        print(f"ERROR: Port {args.port} is already in use. The web server may already be running.")
        print(f"Use --port to specify a different port or --robot-only to skip the web server.")
        sys.exit(1)
    
    # Build environment variables for robot controller
    env_vars = ""
    if args.nav_only:
        env_vars += "NAV_ONLY=1 "
    if args.arm_only:
        env_vars += "ARM_ONLY=1 "
    
    # Launch web server
    if not args.robot_only:
        web_server_cmd = f"python robot_web_server.py --port {args.port}"
        print(f"Launching web server on port {args.port}...")
        web_process = launch_terminal(web_server_cmd, "Web Server")
        
        if not args.no_browser:
            # Wait a bit for the server to start
            time.sleep(2)
            local_ip = get_local_ip()
            web_url = f"http://{local_ip}:{args.port}"
            print(f"Opening web interface at: {web_url}")
            webbrowser.open(web_url)
    
    # Launch robot controller
    if not args.web_only:
        robot_cmd = f"{env_vars}python robot_master_controller.py"
        if args.debug:
            robot_cmd += " --debug"
        
        print("Launching robot controller...")
        robot_process = launch_terminal(robot_cmd, "Controller")
    
    print("\nBoth components have been launched in separate terminals.")
    print("Press Ctrl+C in each terminal to stop the respective component.")
    
    # Keep the script running to maintain the launched processes
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting launcher. The component terminals will remain open.")
        print("You'll need to close each terminal manually.")

if __name__ == "__main__":
    main() 