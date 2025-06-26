from flask import Flask, render_template_string, request
import serial
import time
import threading

SERIAL_PORT = '/dev/ttyUSB1'  # Change as needed
BAUDRATE = 9600

COMMANDS = {
    'Forward': 'w',
    'Backward': 's',
    'Left': 'a',
    'Right': 'd',
    'Rotate CW': 'q',
    'Rotate CCW': 'e',
    'Stop': 'x',
    'Enable Motion': 'z',
    'Disable Motion': 'x'
}

app = Flask(__name__)
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)  # Wait for ESP32 to reset

# Shared variable for latest IR correction value
latest_correction = "N/A"

def serial_reader():
    global latest_correction
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line.startswith("CORRECTION:"):
                latest_correction = line.split(":", 1)[1].strip()
        except Exception:
            pass

# Start background thread to read serial for IR correction
threading.Thread(target=serial_reader, daemon=True).start()

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>AMR Control GUI</title>
    <style>
        body { font-family: Arial; text-align: center; }
        button { width: 120px; height: 40px; margin: 8px; font-size: 18px; }
        .row { margin-bottom: 16px; }
        input[type=number] { width: 80px; font-size: 18px; }
        .irbox { margin: 30px auto; background: #222; color: #0f0; padding: 12px 0; border-radius: 8px; width: 320px; font-size: 22px; }
    </style>
</head>
<body>
    <h1>AMR Motion Control</h1>
    <form method="post">
        <div class="row">
            <button name="cmd" value="Enable Motion">Enable Motion</button>
            <button name="cmd" value="Disable Motion">Disable Motion</button>
        </div>
        <div class="row">
            <button name="cmd" value="Forward">Forward</button>
        </div>
        <div class="row">
            <button name="cmd" value="Left">Left</button>
            <button name="cmd" value="Stop">Stop</button>
            <button name="cmd" value="Right">Right</button>
        </div>
        <div class="row">
            <button name="cmd" value="Backward">Backward</button>
        </div>
        <div class="row">
            <button name="cmd" value="Rotate CW">Rotate CW</button>
            <button name="cmd" value="Rotate CCW">Rotate CCW</button>
        </div>
    </form>
    <form method="post" style="margin-top:30px;">
        <label>Correction value (e.g. -2, 3): </label>
        <input type="number" name="correction" required>
        <button type="submit" name="send_correction" value="1">Send Correction</button>
    </form>
    <div class="irbox">
        <b>Latest IR Correction:</b> {{ latest_correction }}
    </div>
    {% if response %}
    <div>
        <h3>ESP32 Response:</h3>
        <pre>{{ response }}</pre>
    </div>
    {% endif %}
</body>
</html>
"""

@app.route('/', methods=['GET', 'POST'])
def index():
    response = ""
    global latest_correction
    if request.method == 'POST':
        if 'cmd' in request.form:
            cmd_label = request.form['cmd']
            cmd = COMMANDS.get(cmd_label)
            if cmd:
                ser.write((cmd + '\n').encode())
                time.sleep(0.1)
                while ser.in_waiting:
                    response += ser.readline().decode(errors='ignore')
        elif 'send_correction' in request.form:
            correction = request.form.get('correction')
            if correction:
                ser.write((str(correction) + '\n').encode())
                time.sleep(0.1)
                while ser.in_waiting:
                    response += ser.readline().decode(errors='ignore')
    return render_template_string(HTML, response=response, latest_correction=latest_correction)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)