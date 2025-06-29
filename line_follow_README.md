# Line Following Sequences for Robot Controller

This README explains how to use the custom sequences for line following, shelf detection, and box placement.

## Available Sequences

The `line_follow_sequences.json` file contains the following sequences:

1. **line_follow_to_shelf** - Simple line following until reaching a shelf (all IR sensors read 0)
2. **line_follow_to_shelf_and_place** - Line following, then pick up a box from back position 1 and place it on shelf A
3. **custom_line_follow_shelf_b** - Line following, then pick up a box from back position 1 and place it on shelf B

## How to Import These Sequences

1. Copy the `line_follow_sequences.json` file to the same directory as your `robot_controller.py` file
2. Merge the sequences into your existing `sequences.json` file by running:

```
python -c "import json; s1 = json.load(open('sequences.json')); s2 = json.load(open('line_follow_sequences.json')); s1.update(s2); json.dump(s1, open('sequences.json', 'w'), indent=4)"
```

Alternatively, you can manually add these sequences using the Sequence Recorder interface:

1. Go to the Sequence Recorder page
2. Create a new sequence with the appropriate name (e.g., "line_follow_to_shelf")
3. Add the commands as shown in the JSON file
4. Save the sequence

## How These Sequences Work

### Line Following Logic

The line following sequences rely on the robot's IR sensors to detect the black line. The `continuous_line_monitor` function in `robot_controller.py` handles the automatic corrections to keep the robot on the line.

When all IR sensors read 0 (indicating a shelf or the end of the line), the robot will stop. This is detected by the `continuous_line_monitor` function.

### Sequence Components

Each sequence consists of:

1. **Navigation Phase** - Enable motion and move forward to follow the line until reaching a shelf
2. **Pickup Phase** - Turn to the back, lower the arm, grip the box, and return to the front
3. **Placement Phase** - Turn to the appropriate shelf, position the arm, release the box, and return to home position

## Tips for Using These Sequences

1. **Starting Position**: Place the robot with its IR sensors directly over the black line before starting the sequence
2. **IR Calibration**: Make sure your IR sensors are properly calibrated to detect the black line
3. **Box Position**: Ensure a box is properly placed in the back position 1 before running the full sequence
4. **Testing**: Test the sequence in parts first (navigation, pickup, placement) before running the full sequence

## Customizing the Sequences

You can modify these sequences using the Sequence Recorder:

1. Go to the Sequence Recorder page
2. Click "Edit" next to the sequence you want to modify
3. Make your changes (add/remove commands, adjust delays)
4. Save the sequence with the same name to override it

## Troubleshooting

If the robot doesn't follow the line correctly:

1. Check that the IR sensors are properly positioned over the line
2. Verify that the black line has good contrast against the background
3. Adjust the thresholds in the `continuous_line_monitor` function if needed
4. Increase the delays between forward commands to give more time for corrections

If the robot doesn't detect the shelf:

1. Make sure the shelf is positioned at the end of the line
2. Check that all IR sensors are reading 0 when at the shelf position
3. Adjust the `max_consecutive_zeros` threshold in the `continuous_line_monitor` function 