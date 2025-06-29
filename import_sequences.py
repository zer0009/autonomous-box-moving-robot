#!/usr/bin/env python3
"""
Import Line Following Sequences

This script imports the line following sequences from line_follow_sequences.json
into the main sequences.json file used by the robot controller.
"""

import json
import os
import sys

def main():
    # Check if files exist
    if not os.path.exists('line_follow_sequences.json'):
        print("Error: line_follow_sequences.json not found")
        return 1
    
    # Check if sequences.json exists, if not create empty dict
    if os.path.exists('sequences.json'):
        try:
            with open('sequences.json', 'r') as f:
                existing_sequences = json.load(f)
        except json.JSONDecodeError:
            print("Error: sequences.json exists but is not valid JSON")
            return 1
    else:
        existing_sequences = {}
        print("No existing sequences.json found, will create a new one")
    
    # Load new sequences
    try:
        with open('line_follow_sequences.json', 'r') as f:
            new_sequences = json.load(f)
    except json.JSONDecodeError:
        print("Error: line_follow_sequences.json is not valid JSON")
        return 1
    
    # Check for conflicts
    conflicts = []
    for key in new_sequences:
        if key in existing_sequences:
            conflicts.append(key)
    
    if conflicts:
        print(f"Warning: The following sequences already exist and will be overwritten:")
        for conflict in conflicts:
            print(f"  - {conflict}")
        
        response = input("Do you want to continue? (y/n): ")
        if response.lower() != 'y':
            print("Import cancelled")
            return 0
    
    # Merge sequences
    existing_sequences.update(new_sequences)
    
    # Save merged sequences
    try:
        with open('sequences.json', 'w') as f:
            json.dump(existing_sequences, f, indent=4)
        print(f"Successfully imported {len(new_sequences)} sequences into sequences.json")
    except Exception as e:
        print(f"Error saving sequences.json: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 