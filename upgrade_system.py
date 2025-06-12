#!/usr/bin/env python3
"""
Box-Moving Robot System Upgrade Script
Upgrades the robot database and QR codes for multi-box handling and shelf sections
"""

import os
import sqlite3
import argparse
import shutil
from datetime import datetime

def backup_database(db_path='robot_tasks.db'):
    """Create a backup of the database before upgrading"""
    if os.path.exists(db_path):
        backup_name = f"robot_tasks_backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}.db"
        shutil.copy2(db_path, backup_name)
        print(f"Database backup created: {backup_name}")
        return backup_name
    return None

def upgrade_database(db_path='robot_tasks.db'):
    """Upgrade database schema to support shelf sections and multi-box handling"""
    print("Upgrading database schema...")
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Check if destination_section column exists in boxes table
    cursor.execute("PRAGMA table_info(boxes)")
    columns = [column[1] for column in cursor.fetchall()]
    
    # Add destination_section column if it doesn't exist
    schema_changes = []
    if 'destination_section' not in columns:
        cursor.execute('''
            ALTER TABLE boxes ADD COLUMN destination_section TEXT
        ''')
        schema_changes.append("Added destination_section column to boxes table")
        
    # Add stack_position column if it doesn't exist
    if 'stack_position' not in columns:
        cursor.execute('''
            ALTER TABLE boxes ADD COLUMN stack_position INTEGER DEFAULT 0
        ''')
        schema_changes.append("Added stack_position column to boxes table")
    
    # Create shelf_sections table if it doesn't exist
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS shelf_sections (
            shelf_id TEXT,
            section_id TEXT,
            capacity INTEGER,
            occupied INTEGER DEFAULT 0,
            last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            PRIMARY KEY (shelf_id, section_id)
        )
    ''')
    schema_changes.append("Created shelf_sections table if needed")
    
    # Initialize shelf sections
    shelf_data = {
        'A': ['A1', 'A2', 'A3'],
        'B': ['B1', 'B2', 'B3'],
        'C': ['C1', 'C2', 'C3']
    }
    
    for shelf_id, sections in shelf_data.items():
        for section_id in sections:
            cursor.execute('''
                INSERT OR IGNORE INTO shelf_sections (shelf_id, section_id, capacity, occupied)
                VALUES (?, ?, ?, ?)
            ''', (f"SHELF_{shelf_id}", section_id, 3, 0))
    schema_changes.append("Initialized shelf sections data")
            
    # Update existing boxes with default section values
    if 'destination_section' in columns:
        cursor.execute('''
            UPDATE boxes
            SET destination_section = (
                CASE
                    WHEN destination_shelf = 'SHELF_A' THEN 'A1'
                    WHEN destination_shelf = 'SHELF_B' THEN 'B1'
                    WHEN destination_shelf = 'SHELF_C' THEN 'C1'
                    ELSE NULL
                END
            )
            WHERE destination_section IS NULL
        ''')
        schema_changes.append("Updated existing boxes with default section values")
    
    conn.commit()
    conn.close()
    
    if schema_changes:
        print("Database schema upgraded successfully:")
        for change in schema_changes:
            print(f"- {change}")
    else:
        print("Database schema is already up to date.")

def update_qr_data(db_path='robot_tasks.db'):
    """Update QR code data in the database to new format"""
    print("Updating QR code data format...")
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get all box QR codes
    cursor.execute('''
        SELECT id, data
        FROM qr_codes
        WHERE qr_type = 'box'
    ''')
    
    qr_codes = cursor.fetchall()
    updated_count = 0
    
    for qr_id, qr_data in qr_codes:
        if '_SECTION_' not in qr_data:
            # Legacy format: BOX_ID123_SHELF_A_WEIGHT_2.5
            parts = qr_data.split('_')
            if len(parts) >= 4 and parts[0] == 'BOX':
                box_id = parts[1]
                shelf = parts[3]
                # Extract weight (if present)
                weight = "1.0"
                for i, part in enumerate(parts):
                    if part == "WEIGHT" and i+1 < len(parts):
                        weight = parts[i+1]
                        break
                
                # New format: BOX_ID123_SHELF_A_SECTION_A1_WEIGHT_2.5
                section = f"{shelf}1"  # Default to section 1
                new_data = f"BOX_{box_id}_SHELF_{shelf}_SECTION_{section}_WEIGHT_{weight}"
                
                # Update QR code data
                cursor.execute('''
                    UPDATE qr_codes
                    SET data = ?
                    WHERE id = ?
                ''', (new_data, qr_id))
                
                # Also update the box record
                cursor.execute('''
                    UPDATE boxes
                    SET destination_section = ?
                    WHERE id = ?
                ''', (section, qr_id))
                
                updated_count += 1
    
    conn.commit()
    conn.close()
    
    print(f"Updated {updated_count} QR codes to new format.")

def check_install():
    """Check if all required files are present"""
    required_files = [
        'esp32_arm_controller.ino',
        'esp32_navigation_controller.ino',
        'robot_master_controller.py',
        'robot_integration.py',
        'robot_web_server.py',
        'qr_code_generator.py'
    ]
    
    missing_files = []
    for file in required_files:
        if not os.path.exists(file):
            missing_files.append(file)
    
    if missing_files:
        print("WARNING: The following required files are missing:")
        for file in missing_files:
            print(f"- {file}")
        return False
    
    return True

def generate_sample_qr_codes():
    """Generate sample QR codes with the new format"""
    try:
        import subprocess
        print("Generating sample QR codes with new format...")
        
        # Generate a multi-box batch for SHELF_A
        subprocess.call(['python', 'qr_code_generator.py', '--type', 'multi', '--shelf', 'A', '--multi-count', '3'])
        
        # Generate section markers
        for shelf in ['A', 'B', 'C']:
            for section in range(1, 4):
                section_id = f"{shelf}{section}"
                subprocess.call(['python', 'qr_code_generator.py', '--type', 'section', '--shelf', shelf, '--section', section_id])
        
        print("Sample QR codes generated successfully.")
    except Exception as e:
        print(f"Error generating sample QR codes: {e}")

def main():
    parser = argparse.ArgumentParser(description='Upgrade Box-Moving Robot System')
    
    parser.add_argument('--no-backup', action='store_true', help='Skip database backup')
    parser.add_argument('--skip-qr-update', action='store_true', help='Skip QR code data update')
    parser.add_argument('--no-samples', action='store_true', help='Skip sample QR code generation')
    parser.add_argument('--db-path', default='robot_tasks.db', help='Path to robot database')
    
    args = parser.parse_args()
    
    print("Box-Moving Robot System Upgrade Tool")
    print("===================================")
    
    # Check installation
    if not check_install():
        proceed = input("Some required files are missing. Continue anyway? (y/n): ")
        if proceed.lower() != 'y':
            print("Upgrade aborted.")
            return
    
    # Backup database
    if not args.no_backup:
        backup_name = backup_database(args.db_path)
        if backup_name:
            print(f"Backup created: {backup_name}")
    
    # Upgrade database schema
    upgrade_database(args.db_path)
    
    # Update QR data
    if not args.skip_qr_update:
        update_qr_data(args.db_path)
    
    # Generate sample QR codes
    if not args.no_samples:
        generate_sample_qr_codes()
    
    print("\nUpgrade complete!")
    print("To take advantage of the new features:")
    print("1. Upload the updated esp32_arm_controller.ino to your arm controller ESP32")
    print("2. Restart the robot_master_controller.py script")
    print("3. Use the new multi-box and section features in the web interface")

if __name__ == "__main__":
    main() 