#!/usr/bin/env python3
"""
QR Code Generator for Box-Moving Robot System
Generates QR codes for boxes, shelves, and floor markers
"""

import qrcode
import argparse
import os
from PIL import Image, ImageDraw, ImageFont
import sqlite3
import uuid
from datetime import datetime

class RobotQRGenerator:
    def __init__(self, db_path='robot_tasks.db'):
        """Initialize QR code generator with database connection"""
        self.db_path = db_path
        self.output_dir = "generated_qr_codes"
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # Connect to database
        self.init_database()
    
    def init_database(self):
        """Initialize database connection and ensure tables exist"""
        self.conn = sqlite3.connect(self.db_path)
        cursor = self.conn.cursor()
        
        # Create tables if they don't exist
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS boxes (
                id TEXT PRIMARY KEY,
                status TEXT DEFAULT 'pending',
                source_position TEXT,
                destination_shelf TEXT,
                destination_section TEXT,
                pickup_time TIMESTAMP,
                delivery_time TIMESTAMP,
                weight REAL,
                attempts INTEGER DEFAULT 0,
                created_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        # Drop and recreate qr_codes table to ensure it has the correct schema
        cursor.execute('DROP TABLE IF EXISTS qr_codes')
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS qr_codes (
                id TEXT PRIMARY KEY,
                qr_type TEXT,
                data TEXT,
                description TEXT,
                created_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
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
        
        # Initialize shelf sections if needed
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
        
        self.conn.commit()
    
    def generate_box_qr(self, box_id=None, destination="SHELF_A", section=None, weight=1.0, description=""):
        """Generate a QR code for a box with tracking in database"""
        if not box_id:
            box_id = f"ID{uuid.uuid4().hex[:6].upper()}"
        
        # If section not specified, choose a section from the destination shelf
        if not section:
            if destination[-1] in ['A', 'B', 'C']:
                section = f"{destination[-1]}1"  # Default to first section
            else:
                section = "A1"  # Default if shelf not recognized
        
        # Format: BOX_ID123_SHELF_A_SECTION_A2_WEIGHT_2.5
        qr_data = f"BOX_{box_id}_SHELF_{destination[-1]}_SECTION_{section}_WEIGHT_{weight}"
        
        # Generate QR code
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=10,
            border=4,
        )
        qr.add_data(qr_data)
        qr.make(fit=True)
        
        # Create QR code image with label
        qr_img = qr.make_image(fill_color="black", back_color="white")
        
        # Add text label below QR code
        label = f"Box: {box_id}\nDest: {destination}\nSection: {section}\nWeight: {weight}kg"
        img = self.add_label_to_qr(qr_img, label)
        
        # Save to file
        filename = f"{self.output_dir}/box_{box_id}.png"
        img.save(filename)
        
        # Add to database
        cursor = self.conn.cursor()
        
        # Add to boxes table
        cursor.execute('''
            INSERT INTO boxes (id, destination_shelf, destination_section, weight, status)
            VALUES (?, ?, ?, ?, ?)
        ''', (box_id, destination, section, weight, "created"))
        
        # Add to qr_codes table
        cursor.execute('''
            INSERT INTO qr_codes (id, qr_type, data, description)
            VALUES (?, ?, ?, ?)
        ''', (box_id, "box", qr_data, description))
        
        self.conn.commit()
        
        print(f"Box QR code generated: {filename}")
        return filename
    
    def generate_shelf_section_marker(self, shelf_id, section_id, description=""):
        """Generate a shelf section identification QR code"""
        # Format: SHELF_A_SECTION_A1
        qr_data = f"SHELF_{shelf_id}_SECTION_{section_id}"
        marker_id = f"SHELF_{shelf_id}_SECTION_{section_id}"
        
        # Generate QR code
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=10,
            border=4,
        )
        qr.add_data(qr_data)
        qr.make(fit=True)
        
        # Create QR code image with label
        qr_img = qr.make_image(fill_color="black", back_color="white")
        
        # Add text label below QR code
        label = f"Shelf: {shelf_id}\nSection: {section_id}"
        img = self.add_label_to_qr(qr_img, label)
        
        # Save to file
        filename = f"{self.output_dir}/shelf_{shelf_id}_section_{section_id}.png"
        img.save(filename)
        
        # Add to database
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT OR REPLACE INTO qr_codes (id, qr_type, data, description)
            VALUES (?, ?, ?, ?)
        ''', (marker_id, "shelf_section", qr_data, description))
        
        # Ensure shelf section exists in database
        cursor.execute('''
            INSERT OR IGNORE INTO shelf_sections (shelf_id, section_id, capacity, occupied)
            VALUES (?, ?, ?, ?)
        ''', (f"SHELF_{shelf_id}", section_id, 3, 0))
        
        self.conn.commit()
        
        print(f"Shelf section QR code generated: {filename}")
        return filename
    
    def generate_multi_box_batch(self, count=3, destination="SHELF_A", description="Multi-box batch"):
        """Generate a batch of QR codes for the same destination but different sections"""
        shelf_id = destination[-1]  # Extract A from SHELF_A
        sections = [f"{shelf_id}{i}" for i in range(1, 4)]  # Create A1, A2, A3
        
        filenames = []
        for i in range(count):
            # Distribute boxes across available sections
            section = sections[i % len(sections)]
            weight = round(0.5 + (i * 0.3), 1)  # Vary weights
            box_id = f"BATCH{uuid.uuid4().hex[:4].upper()}"
            
            filename = self.generate_box_qr(
                box_id=box_id,
                destination=destination,
                section=section,
                weight=weight,
                description=f"{description} - Box {i+1} of {count}"
            )
            filenames.append(filename)
        
        return filenames
    
    def generate_floor_marker(self, x, y, description=""):
        """Generate a floor position marker QR code"""
        # Format: FLOOR_X10_Y15
        qr_data = f"FLOOR_X{x}_Y{y}"
        marker_id = f"FLOOR_{x}_{y}"
        
        # Generate QR code
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=10,
            border=4,
        )
        qr.add_data(qr_data)
        qr.make(fit=True)
        
        # Create QR code image with label
        qr_img = qr.make_image(fill_color="black", back_color="white")
        
        # Add text label below QR code
        label = f"Position: ({x},{y})"
        img = self.add_label_to_qr(qr_img, label)
        
        # Save to file
        filename = f"{self.output_dir}/floor_x{x}_y{y}.png"
        img.save(filename)
        
        # Add to database
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO qr_codes (id, qr_type, data, description)
            VALUES (?, ?, ?, ?)
        ''', (marker_id, "floor_marker", qr_data, description))
        self.conn.commit()
        
        print(f"Floor marker QR code generated: {filename}")
        return filename
    
    def generate_shelf_marker(self, shelf_id, description=""):
        """Generate a shelf identification QR code"""
        # Format: SHELF_A
        qr_data = f"SHELF_{shelf_id}"
        
        # Generate QR code
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=10,
            border=4,
        )
        qr.add_data(qr_data)
        qr.make(fit=True)
        
        # Create QR code image with label
        qr_img = qr.make_image(fill_color="black", back_color="white")
        
        # Add text label below QR code
        label = f"Shelf: {shelf_id}"
        img = self.add_label_to_qr(qr_img, label)
        
        # Save to file
        filename = f"{self.output_dir}/shelf_{shelf_id}.png"
        img.save(filename)
        
        # Add to database
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO qr_codes (id, qr_type, data, description)
            VALUES (?, ?, ?, ?)
        ''', (shelf_id, "shelf", qr_data, description))
        self.conn.commit()
        
        print(f"Shelf QR code generated: {filename}")
        return filename
    
    def add_label_to_qr(self, qr_img, label_text):
        """Add a text label below the QR code image"""
        # Calculate image dimensions
        qr_width, qr_height = qr_img.size
        label_height = 60  # Height for label text
        
        # Create a new white image with extra space for label
        new_img = Image.new('RGB', (qr_width, qr_height + label_height), color='white')
        # Convert QR image to RGB if it's not already
        if qr_img.mode != 'RGB':
            qr_img = qr_img.convert('RGB')
        new_img.paste(qr_img, (0, 0))
        
        # Add text
        try:
            draw = ImageDraw.Draw(new_img)
            font = ImageFont.truetype("arial.ttf", 16)
            draw.text((10, qr_height + 5), label_text, fill="black", font=font)
        except IOError:
            # Fallback if font not available
            draw = ImageDraw.Draw(new_img)
            draw.text((10, qr_height + 5), label_text, fill="black")
        
        return new_img
    
    def generate_batch(self, num_boxes=5, num_floors=10):
        """Generate a batch of QR codes for testing"""
        print(f"Generating batch of QR codes: {num_boxes} boxes, {num_floors} floor markers")
        
        # Generate box QR codes (distributed across shelves and sections)
        for i in range(num_boxes):
            weight = round(0.5 + (i * 0.5), 1)  # 0.5kg to 2.5kg
            shelf_id = chr(65 + (i % 3))  # A, B, C
            section_id = f"{shelf_id}{((i // 3) % 3) + 1}"  # A1, A2, A3, B1, etc.
            
            self.generate_box_qr(
                destination=f"SHELF_{shelf_id}",
                section=section_id,
                weight=weight,
                description=f"Test box {i+1} for section {section_id}"
            )
        
        # Generate floor markers
        for i in range(num_floors):
            x = i % 5
            y = i // 5
            self.generate_floor_marker(x, y, f"Floor position ({x},{y})")
        
        # Generate shelf section markers
        for shelf_id in ['A', 'B', 'C']:
            for section_num in range(1, 4):
                section_id = f"{shelf_id}{section_num}"
                self.generate_shelf_section_marker(
                    shelf_id, 
                    section_id, 
                    f"Shelf {shelf_id} Section {section_num}"
                )
            
        print("Batch generation complete!")
    
    def close(self):
        """Close database connection"""
        if hasattr(self, 'conn') and self.conn:
            self.conn.close()
    
    def print_qr_code(self, filename):
        """Print a QR code to the default printer"""
        try:
            # Try to use the default printer via the default system print command
            if os.name == 'posix':  # Linux/Mac
                os.system(f"lp {filename}")
                print(f"Sent {filename} to printer")
            elif os.name == 'nt':  # Windows
                os.system(f'print /d:"%printer%" {filename}')
                print(f"Sent {filename} to printer")
            else:
                print("Printing not supported on this OS")
        except Exception as e:
            print(f"Failed to print: {e}")
            print("Please print the file manually")

def main():
    parser = argparse.ArgumentParser(description='Generate QR codes for robot system')
    
    parser.add_argument('--type', choices=['box', 'floor', 'shelf', 'section', 'batch', 'multi'],
                      help='Type of QR code to generate')
    parser.add_argument('--id', help='ID for box or shelf')
    parser.add_argument('--x', type=int, help='X coordinate for floor marker')
    parser.add_argument('--y', type=int, help='Y coordinate for floor marker')
    parser.add_argument('--shelf', help='Destination shelf for box (A, B, C)')
    parser.add_argument('--section', help='Destination section for box (A1, A2, A3, B1, etc.)')
    parser.add_argument('--weight', type=float, default=1.0, help='Weight of box in kg')
    parser.add_argument('--print', action='store_true', help='Print the QR code after generation')
    parser.add_argument('--batch-boxes', type=int, default=5, help='Number of boxes in batch generation')
    parser.add_argument('--batch-floors', type=int, default=10, help='Number of floor markers in batch generation')
    parser.add_argument('--multi-count', type=int, default=3, help='Number of boxes in multi-box batch')
    
    args = parser.parse_args()
    
    generator = RobotQRGenerator()
    
    try:
        if args.type == 'box':
            filename = generator.generate_box_qr(
                box_id=args.id,
                destination=f"SHELF_{args.shelf}" if args.shelf else "SHELF_A",
                section=args.section,
                weight=args.weight
            )
            
        elif args.type == 'floor':
            if args.x is None or args.y is None:
                print("Error: x and y coordinates required for floor markers")
                return
            filename = generator.generate_floor_marker(args.x, args.y)
            
        elif args.type == 'shelf':
            if not args.shelf:
                print("Error: shelf ID required for shelf marker")
                return
            filename = generator.generate_shelf_marker(args.shelf)
        
        elif args.type == 'section':
            if not args.shelf or not args.section:
                print("Error: both shelf ID and section ID required for shelf section marker")
                return
            filename = generator.generate_shelf_section_marker(args.shelf, args.section)
            
        elif args.type == 'multi':
            filenames = generator.generate_multi_box_batch(
                count=args.multi_count,
                destination=f"SHELF_{args.shelf}" if args.shelf else "SHELF_A"
            )
            filename = filenames[0] if filenames else None
            
        elif args.type == 'batch':
            generator.generate_batch(args.batch_boxes, args.batch_floors)
            filename = None
            
        else:
            print("Please specify a type: --type box|floor|shelf|section|multi|batch")
            return
        
        if args.print and filename:
            generator.print_qr_code(filename)
            
    finally:
        generator.close()

if __name__ == "__main__":
    main() 