# ServoArm

This repo contains a Python controller that uses MediaPipe (pose + hands) and OpenCV to mirror your arm/hand to a 4-servo robotic arm via an Arduino.

Files
- `ServoArm.ino` - Arduino sketch that listens on Serial for commands in the format: C{claw}H{height}E{extension}B{base}\n (values constrained in the sketch).
- `ServoArm.py` - Python controller (MediaPipe + OpenCV) that sends compact commands like `C45H30E60B90` over serial.
- `requirements.txt` - Python dependencies.

Quick start
1. Create a virtual environment and install deps:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

2. Update the Arduino device path in `ServoArm.py` if necessary (default is `/dev/cu.usbmodem101` on macOS).
3. Run the controller:

```bash
python ServoArm.py
```

Controls
- Press `q` to quit.
- Press `r` to reset servos to a neutral pose.

Notes
- If the Arduino doesn't respond, check the serial port and baud rate in `ServoArm.py` and the `ServoArm.ino` sketch.
- Camera index (0) and frame size are configured in the Python script; adjust if needed.
