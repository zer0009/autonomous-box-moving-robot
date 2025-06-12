#!/usr/bin/env python3
"""
Camera Connection Checker
Checks if cameras are connected and provides detailed information
"""

import cv2
import os
import sys
import subprocess
import platform

def check_system_cameras():
    """Check for cameras at the system level"""
    system = platform.system()
    print(f"Operating System: {platform.system()} {platform.release()}")
    
    if system == "Linux":
        # Check for video devices on Linux
        try:
            print("\n=== Linux Video Devices ===")
            result = subprocess.run(["ls", "-l", "/dev/video*"], 
                                   capture_output=True, text=True)
            if result.stdout:
                print(result.stdout)
            else:
                print("No video devices found in /dev/video*")
                
            # Check if v4l-utils is installed and use it
            try:
                print("\n=== V4L2 Device Details ===")
                result = subprocess.run(["v4l2-ctl", "--list-devices"], 
                                       capture_output=True, text=True)
                if result.stdout:
                    print(result.stdout)
            except FileNotFoundError:
                print("v4l2-ctl not found. Install with: sudo apt install v4l-utils")
                
        except Exception as e:
            print(f"Error checking Linux devices: {e}")
            
    elif system == "Windows":
        print("\nWindows doesn't provide a direct way to list camera devices from command line.")
        print("We'll check using OpenCV in the next step.")
        
    elif system == "Darwin":  # macOS
        try:
            print("\n=== macOS Video Devices ===")
            result = subprocess.run(["system_profiler", "SPCameraDataType"], 
                                   capture_output=True, text=True)
            if result.stdout:
                print(result.stdout)
            else:
                print("No camera information found")
        except Exception as e:
            print(f"Error checking macOS devices: {e}")

def check_opencv_cameras():
    """Check for cameras using OpenCV"""
    print("\n=== OpenCV Camera Check ===")
    
    # Try camera indices 0-9
    found_cameras = []
    for index in range(10):
        try:
            print(f"Trying camera index {index}...")
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    # Get camera properties
                    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    fps = cap.get(cv2.CAP_PROP_FPS)
                    
                    print(f"✓ Camera {index} connected successfully!")
                    print(f"  Resolution: {width}x{height}")
                    print(f"  FPS: {fps}")
                    
                    # Save a test image
                    test_filename = f"camera_{index}_test.jpg"
                    cv2.imwrite(test_filename, frame)
                    print(f"  Test image saved to {test_filename}")
                    
                    found_cameras.append(index)
                else:
                    print(f"✗ Camera {index} opened but couldn't read frame")
            else:
                print(f"✗ Camera {index} failed to open")
                
            # Always release the camera
            cap.release()
            
        except Exception as e:
            print(f"✗ Error with camera {index}: {e}")
    
    # Summary
    if found_cameras:
        print(f"\nFound {len(found_cameras)} working camera(s) at indices: {found_cameras}")
    else:
        print("\nNo working cameras found")
        
    return found_cameras

def check_raspberry_pi_camera():
    """Check for Raspberry Pi specific camera"""
    if not platform.system() == "Linux":
        return
        
    print("\n=== Raspberry Pi Camera Check ===")
    
    # Check if this is a Raspberry Pi
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read()
            if 'Raspberry Pi' not in model:
                print("Not running on a Raspberry Pi")
                return
            print(f"Model: {model.strip()}")
    except:
        print("Not running on a Raspberry Pi or couldn't determine model")
        return
    
    # Check if camera is enabled in raspi-config
    try:
        result = subprocess.run(["vcgencmd", "get_camera"], 
                               capture_output=True, text=True)
        if result.stdout:
            print(f"Camera status: {result.stdout.strip()}")
            # Parse the output which is typically "supported=1 detected=1"
            if "detected=1" in result.stdout:
                print("Raspberry Pi camera module detected!")
            else:
                print("No Raspberry Pi camera module detected")
    except:
        print("Couldn't check Raspberry Pi camera status")
    
    # Try to open the Raspberry Pi camera with OpenCV
    try:
        print("\nTrying to access Raspberry Pi camera with OpenCV...")
        # Try different pipelines for Raspberry Pi camera
        pipelines = [
            "libcamerasrc ! video/x-raw, width=640, height=480 ! videoconvert ! appsink",
            "v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480 ! videoconvert ! appsink"
        ]
        
        for i, pipeline in enumerate(pipelines):
            print(f"Trying pipeline {i+1}: {pipeline}")
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print("✓ Successfully accessed Raspberry Pi camera!")
                    test_filename = "picamera_test.jpg"
                    cv2.imwrite(test_filename, frame)
                    print(f"  Test image saved to {test_filename}")
                    cap.release()
                    break
                else:
                    print("✗ Camera opened but couldn't read frame")
            else:
                print("✗ Failed to open camera with this pipeline")
            
            cap.release()
    except Exception as e:
        print(f"Error accessing Raspberry Pi camera: {e}")

def print_troubleshooting_tips():
    """Print troubleshooting tips for camera issues"""
    print("\n=== Troubleshooting Tips ===")
    system = platform.system()
    
    # General tips
    print("General tips:")
    print("1. Make sure the camera is properly connected")
    print("2. Try a different USB port")
    print("3. Restart your computer/device")
    
    if system == "Linux":
        print("\nLinux-specific tips:")
        print("1. Check permissions: ls -l /dev/video*")
        print("2. Add your user to the video group: sudo usermod -a -G video $USER")
        print("3. Install v4l-utils for debugging: sudo apt install v4l-utils")
        print("4. List camera details: v4l2-ctl --list-devices")
        print("5. For Raspberry Pi Camera:")
        print("   - Enable camera in raspi-config: sudo raspi-config")
        print("   - Check camera status: vcgencmd get_camera")
        print("   - Make sure camera ribbon cable is properly connected")
        
    elif system == "Windows":
        print("\nWindows-specific tips:")
        print("1. Check Device Manager for camera devices")
        print("2. Make sure camera is not being used by another application")
        print("3. Update camera drivers")
        print("4. Check Windows privacy settings to allow apps to access camera")
        
    elif system == "Darwin":  # macOS
        print("\nMacOS-specific tips:")
        print("1. Check System Preferences > Security & Privacy > Camera")
        print("2. Make sure applications have permission to use the camera")
        print("3. Reset the SMC if camera is built-in")

if __name__ == "__main__":
    print("=== Camera Connection Checker ===")
    
    # Check system-level camera devices
    check_system_cameras()
    
    # Check cameras using OpenCV
    found_cameras = check_opencv_cameras()
    
    # Check for Raspberry Pi camera specifically
    check_raspberry_pi_camera()
    
    # Print troubleshooting tips
    print_troubleshooting_tips()
    
    # Final summary
    if found_cameras:
        print(f"\nSUMMARY: Found {len(found_cameras)} working camera(s) at indices: {found_cameras}")
        print("You can use these camera indices in your robot code.")
    else:
        print("\nSUMMARY: No working cameras found. See troubleshooting tips above.") 