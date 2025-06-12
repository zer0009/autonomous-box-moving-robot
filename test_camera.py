#!/usr/bin/env python3
"""
Camera Test Utility
Tests different camera indices and device paths to find working cameras
"""

import cv2
import os
import time
import argparse

def test_camera_index(index, show_preview=False):
    """Test if a camera works at the given index"""
    print(f"Testing camera at index {index}...")
    try:
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            print(f"Failed to open camera at index {index}")
            return False
            
        # Try to read a frame
        ret, frame = cap.read()
        if not ret or frame is None:
            print(f"Could not read frame from camera at index {index}")
            cap.release()
            return False
            
        # Success!
        print(f"SUCCESS! Camera works at index {index}")
        print(f"Resolution: {frame.shape[1]}x{frame.shape[0]}")
        
        # Show preview if requested
        if show_preview:
            cv2.imshow(f"Camera {index}", frame)
            print("Press any key to continue...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        cap.release()
        return True
    except Exception as e:
        print(f"Error testing camera at index {index}: {e}")
        return False

def test_device_path(path, show_preview=False):
    """Test if a camera works at the given device path"""
    if not os.path.exists(path):
        print(f"Device path {path} does not exist")
        return False
        
    print(f"Testing camera at device path {path}...")
    try:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print(f"Failed to open camera at device path {path}")
            return False
            
        # Try to read a frame
        ret, frame = cap.read()
        if not ret or frame is None:
            print(f"Could not read frame from camera at device path {path}")
            cap.release()
            return False
            
        # Success!
        print(f"SUCCESS! Camera works at device path {path}")
        print(f"Resolution: {frame.shape[1]}x{frame.shape[0]}")
        
        # Show preview if requested
        if show_preview:
            cv2.imshow(f"Camera {path}", frame)
            print("Press any key to continue...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        cap.release()
        return True
    except Exception as e:
        print(f"Error testing camera at device path {path}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Test camera indices and device paths')
    parser.add_argument('--indices', type=str, default='0,1,2,19-35', help='Camera indices to test (e.g. 0,1,2,19-35)')
    parser.add_argument('--devices', action='store_true', help='Test device paths (/dev/videoX) instead of indices')
    parser.add_argument('--show', action='store_true', help='Show preview of each working camera')
    parser.add_argument('--first-only', action='store_true', help='Stop after finding the first working camera')
    args = parser.parse_args()
    
    # Parse indices
    indices = []
    for part in args.indices.split(','):
        if '-' in part:
            start, end = map(int, part.split('-'))
            indices.extend(range(start, end + 1))
        else:
            indices.append(int(part))
    
    found_camera = False
    
    # Test device paths if requested
    if args.devices:
        print("Testing camera device paths...")
        for index in indices:
            device_path = f"/dev/video{index}"
            if test_device_path(device_path, args.show):
                found_camera = True
                if args.first_only:
                    print(f"Found working camera at device path {device_path}")
                    break
    # Otherwise test indices
    else:
        print("Testing camera indices...")
        for index in indices:
            if test_camera_index(index, args.show):
                found_camera = True
                if args.first_only:
                    print(f"Found working camera at index {index}")
                    break
    
    if not found_camera:
        print("No working cameras found!")
        print("Check that your camera is properly connected and has the correct permissions.")
        print("On Raspberry Pi, ensure the camera is enabled with 'sudo raspi-config'")
    
if __name__ == "__main__":
    main() 