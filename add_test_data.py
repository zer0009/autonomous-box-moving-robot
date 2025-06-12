#!/usr/bin/env python3
"""
Add test data to the robot database for testing the web interface
"""

import sqlite3
import datetime
import random

def get_db_connection():
    """Create a database connection"""
    conn = sqlite3.connect('robot_tasks.db')
    conn.row_factory = sqlite3.Row
    return conn

def add_test_logs():
    """Add test log entries to the database"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # Create robot_log table if it doesn't exist
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS robot_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            event_type TEXT,
            description TEXT,
            position TEXT,
            success INTEGER DEFAULT 1
        )
    ''')
    
    # Sample log events
    log_events = [
        ("startup", "Robot system initialized", "(0,0)", 1),
        ("movement", "Moving to position (2,3)", "(2,3)", 1),
        ("scan", "Scanned QR code: BOX_ID123", "(2,3)", 1),
        ("pickup", "Picked up box ID123", "(2,3)", 1),
        ("movement", "Moving to shelf A", "(5,7)", 1),
        ("error", "Failed to recognize shelf QR code", "(5,7)", 0),
        ("retry", "Retrying shelf scan", "(5,7)", 1),
        ("delivery", "Delivered box ID123 to shelf A", "(5,7)", 1),
        ("movement", "Returning to home position", "(0,0)", 1),
        ("status", "Robot idle and waiting for next task", "(0,0)", 1)
    ]
    
    # Insert logs with timestamps spread over the last hour
    now = datetime.datetime.now()
    for i, (event_type, description, position, success) in enumerate(log_events):
        # Create timestamps spread over the last hour
        timestamp = now - datetime.timedelta(minutes=(len(log_events) - i) * 6)
        
        cursor.execute('''
            INSERT INTO robot_log (timestamp, event_type, description, position, success)
            VALUES (?, ?, ?, ?, ?)
        ''', (timestamp, event_type, description, position, success))
    
    conn.commit()
    print("Added test log entries to the database")
    
    # Verify logs were added
    logs = cursor.execute('SELECT * FROM robot_log').fetchall()
    print(f"Total log entries: {len(logs)}")
    
    conn.close()

def add_test_boxes():
    """Add test box entries to the database"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # Sample box data
    boxes = [
        ("BOX123", "delivered", "(2,3)", "SHELF_A", "A1", 
         datetime.datetime.now() - datetime.timedelta(hours=2), 
         datetime.datetime.now() - datetime.timedelta(hours=1), 1.5),
        
        ("BOX456", "in_transit", "(5,7)", "SHELF_B", "B2", 
         datetime.datetime.now() - datetime.timedelta(minutes=30), 
         None, 2.0),
        
        ("BOX789", "pending", None, "SHELF_C", "C3", 
         None, None, 0.8),
    ]
    
    for box_id, status, source, dest_shelf, dest_section, pickup, delivery, weight in boxes:
        cursor.execute('''
            INSERT OR REPLACE INTO boxes 
            (id, status, source_position, destination_shelf, destination_section, 
             pickup_time, delivery_time, weight)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ''', (box_id, status, source, dest_shelf, dest_section, pickup, delivery, weight))
    
    conn.commit()
    print("Added test box entries to the database")
    
    # Verify boxes were added
    boxes = cursor.execute('SELECT * FROM boxes').fetchall()
    print(f"Total box entries: {len(boxes)}")
    
    conn.close()

def update_shelf_occupancy():
    """Update shelf occupancy based on delivered boxes"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # Reset all shelf occupancy
    cursor.execute('UPDATE shelf_sections SET occupied = 0')
    
    # Count delivered boxes by section
    cursor.execute('''
        SELECT destination_shelf, destination_section, COUNT(*) as count
        FROM boxes 
        WHERE status = 'delivered'
        GROUP BY destination_shelf, destination_section
    ''')
    
    sections = cursor.execute('''
        SELECT shelf_id, section_id FROM shelf_sections
    ''').fetchall()
    
    # Update occupancy randomly for testing
    for shelf_id, section_id in sections:
        occupied = random.randint(0, 3)
        cursor.execute('''
            UPDATE shelf_sections 
            SET occupied = ?
            WHERE shelf_id = ? AND section_id = ?
        ''', (occupied, shelf_id, section_id))
    
    conn.commit()
    print("Updated shelf occupancy")
    
    conn.close()

if __name__ == "__main__":
    add_test_logs()
    add_test_boxes()
    update_shelf_occupancy()
    print("Test data added successfully!") 