#!/bin/bash

echo "Robot System Launcher"
echo "==================="
echo

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 is not installed or not in PATH."
    echo "Please install Python 3.6 or higher."
    exit 1
fi

# Make the script executable if it's not already
if [ ! -x "run_robot_system.py" ]; then
    chmod +x run_robot_system.py
fi

# Run the launcher script
python3 run_robot_system.py "$@"

if [ $? -ne 0 ]; then
    echo
    echo "An error occurred while running the launcher."
fi 