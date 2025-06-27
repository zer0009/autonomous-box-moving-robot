# Robot Sequence Recorder

This tool allows you to create, edit, test, and manage custom sequences for the robot's arm and navigation controls.

## Features

- **Record Commands**: Create sequences by recording arm and navigation commands with custom delays
- **Edit Sequences**: Modify existing sequences, update command delays, or remove commands
- **Test Sequences**: Test sequences directly from the interface before saving
- **Manage Sequences**: Save, delete, and organize your custom sequences
- **Persistent Storage**: All sequences are saved to a JSON file for persistent storage

## How to Use

### Recording a New Sequence

1. Go to the "Sequence Recorder" page from the main menu
2. Click "Start Recording" to begin recording commands
3. Use the Arm Controls and Navigation Controls panels to send commands to the robot
   - Each command will be recorded with the specified delay
   - The delay slider controls how long to wait after each command
4. Click "Stop Recording" when finished
5. Enter a name for your sequence and click "Save Sequence"

### Editing a Sequence

1. Find the sequence you want to edit in the "Manage Sequences" panel
2. Click the "Edit" button next to the sequence name
3. The sequence will be loaded into the recorder
4. Click "Start Recording" to modify the sequence
   - You can add new commands
   - You can update delays for existing commands
   - You can remove individual commands
5. Click "Stop Recording" when finished
6. Click "Save Sequence" to save your changes

### Testing Sequences

1. To test a saved sequence, click the "Test" button next to the sequence name
2. To test the currently recorded sequence, click "Test Current Sequence"
3. The robot will execute each command in the sequence with the specified delays
4. Test results will be displayed in the "Test Results" panel

### Managing Sequences

- **Save**: Enter a name and click "Save Sequence" to save the current commands as a sequence
- **Delete**: Click the "Delete" button next to a sequence to remove it
- **Clear**: Click "Clear Commands" to remove all currently recorded commands
- **Edit**: Click the "Edit" button to load a sequence for editing

## Command Delays

Each command in a sequence has an associated delay (in seconds) that determines how long to wait after executing the command before moving to the next one. This allows for precise timing control.

- Use the delay slider to set the default delay for new commands
- Use the delay input field next to each recorded command to adjust individual delays
- Click "Update" to save changes to a command's delay

## Tips

- Test sequences thoroughly before using them in production
- Use shorter delays for simple movements and longer delays for complex operations
- Create modular sequences that can be combined for more complex tasks
- Back up your sequences.json file regularly to avoid data loss 