# ServoArm — Real-time CV → 3D-printed Servo Arm

ServoArm mirrors your arm and hand to a simple 4-servo 3D-printed robotic arm using MediaPipe + OpenCV on the PC and an Arduino on the robot. The Python controller reads pose + hand landmarks from the camera, converts them to compact servo commands, and sends them over serial to the Arduino sketch (ServoArm.ino).

Quick overview
- Tracks one hand and the left arm in real time.
- Sends compact commands: `C{claw}H{height}E{extension}B{base}` over serial.
- Minimal smoothing + deadzones to reduce jitter.
- Camera UI can display landmarks and a small status overlay.

Requirements
- Linux or macOS
- Python 3.8+
- Arduino (any UNO/compatible) with `ServoArm.ino` uploaded
- A working webcam

Python dependencies (install in a virtualenv)
```
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Important: run with the following command to show the camera UI
```
python ServoArm.py --show-frame --port /dev/ttyACM0
```
- Replace `/dev/ttyACM0` with your platform port (e.g. `/dev/cu.usbmodem101` on macOS).
- If you omit `--show-frame` the camera UI and overlays will not appear.

Files
- `ServoArm.py` — Python controller (MediaPipe + OpenCV + pyserial).
- `ServoArm.ino` — Arduino sketch that parses serial commands and drives servos.
- `requirements.txt` — Python packages.
- `.gitignore`, README, other project files.

Arduino / Serial notes
- Default baud: 9600 (must match in code + sketch).
- Serial format example: `C45H30E60B90\n`.
- On Linux, ports are typically `/dev/ttyACM0` or `/dev/ttyUSB0`. On macOS, ports are like `/dev/cu.usbmodem101`.
- Only one process can open the serial port — close Arduino IDE Serial Monitor before running the Python script.
- If you see permission errors on Linux, add your user to the dialout group:
  ```
  sudo usermod -a -G dialout $USER
  ```
  then log out/in.

Power & wiring
- Small servos may draw more current than the Arduino 5V regulator can supply. Use an external 5V supply if servos stutter or don’t move. Always connect grounds between Arduino and the servo power supply.

Troubleshooting
- No serial response: verify the port and baud rate, ensure Arduino sketch is uploaded, close Serial Monitor.
- Servo jitter: use external power and confirm wiring.
- UI shows reversed/open/closed discrepancy: verify actual servo positions and test by sending direct serial commands from a terminal or the Arduino Serial Monitor (e.g. `C45`).
- Quick test script (Python) to exercise the claw:
  ```py
  import serial, time
  ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
  time.sleep(2)
  ser.write(b'C45H45E45B90\n')
  ser.close()
  ```

Usage
- Run with camera UI:
  ```
  python ServoArm.py --show-frame --port /dev/ttyACM0
  ```
- Run in dry-run mode (no Arduino):
  ```
  python ServoArm.py --dry-run --show-frame
  ```
- Reset servos from the UI or send the reset command:
  ```
  C45H45E45B90
  ```

Contributing
- Tweak deadzones, smoothing and camera resolution in `ServoArm.py`.
- Submit fixes or feature requests via PRs.

License
- MIT-style — see project LICENSE or add one.

Notes
- This README focuses on running the local controller with a tethered Arduino. If you plan to use Arduino Cloud or other remote tooling, follow those services' setup separately.
