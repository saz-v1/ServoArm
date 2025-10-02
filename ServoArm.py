import cv2
import mediapipe as mp
import serial
import time
import numpy as np
import math

class HandMirrorController:
    def __init__(self, arduino_port='/dev/cu.usbmodem101', baud_rate=9600):
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Initialize Arduino
        try:
            self.arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
            time.sleep(2)  # wait for Arduino reset
            print(f"✅ Connected to Arduino on {arduino_port}")
        except Exception as e:
            print(f"❌ Could not connect to Arduino: {e}")
            self.arduino = None

        # Camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Control rate
        self.last_command_time = 0
        self.command_interval = 0.5  # seconds between Arduino updates

        # Previous servo values
        self.prev_claw = 90
        self.prev_height = 45
        self.prev_extension = 45
        self.prev_base = 90

        print("🤚 Hand Mirror Controller Ready!")
        print("Controls:")
        print("🤜 FIST → Claw closes")
        print("🖐 OPEN → Claw opens")
        print("⬆️ Forearm up → Arm up")
        print("⬇️ Forearm down → Arm down")
        print("↔️ Tilt forward/back → Extension")
        print("🔄 Rotate wrist → Base rotate")
        print("Press 'q' to quit, 'r' to reset servos")

    def send_command(self, command):
        """Send string command to Arduino"""
        if self.arduino:
            try:
                self.arduino.write((command + '\n').encode())
                return True
            except Exception as e:
                print(f"Arduino error: {e}")
        return False

    # --- Hand landmark calculations ---
    def calculate_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def calculate_fist_closure(self, landmarks):
        fingertips = [4, 8, 12, 16, 20]
        joints = [3, 6, 10, 14, 18]
        curl_scores = []
        for tip, joint in zip(fingertips, joints):
            tip_pos, joint_pos, wrist_pos = landmarks[tip], landmarks[joint], landmarks[0]
            wrist_to_tip = self.calculate_distance(wrist_pos, tip_pos)
            wrist_to_joint = self.calculate_distance(wrist_pos, joint_pos)
            curl_scores.append(wrist_to_tip / (wrist_to_joint + 1e-6))
        avg_curl = sum(curl_scores) / len(curl_scores)
        claw_angle = int(np.clip((avg_curl - 0.8) * 300, 0, 90))
        return claw_angle

    def calculate_wrist_rotation(self, landmarks):
        wrist, index_mcp, pinky_mcp = landmarks[0], landmarks[5], landmarks[17]
        vec = np.array([index_mcp.x - pinky_mcp.x, index_mcp.y - pinky_mcp.y])
        angle = math.atan2(vec[1], vec[0])
        deg = np.degrees(angle)
        if abs(deg) < 15:
            return 90
        elif deg > 15:
            return int(np.clip(90 + (deg - 15) / 75 * 20, 90, 110))
        else:
            return int(np.clip(90 + (deg + 15) / 75 * 20, 70, 90))

    def calculate_forearm_pitch(self, landmarks):
        wrist, middle_mcp = landmarks[0], landmarks[9]
        vec = np.array([middle_mcp.x - wrist.x, middle_mcp.y - wrist.y])
        up = np.array([0, -1])
        angle = math.degrees(math.atan2(np.cross(vec, up), np.dot(vec, up)))
        return int(np.clip(-angle + 90, 0, 90))

    def calculate_hand_extension(self, landmarks):
        wrist, middle_tip = landmarks[0], landmarks[12]
        forward_component = -(middle_tip.y - wrist.y)
        extension_ratio = forward_component / (abs(middle_tip.y - wrist.y) + 1e-6)
        return int(np.clip(extension_ratio * 45 + 45, 0, 90))

    # --- Mirror hand to servo commands ---
    def mirror_hand_to_servos(self, landmarks):
        claw = self.calculate_fist_closure(landmarks)
        height = self.calculate_forearm_pitch(landmarks)
        extension = self.calculate_hand_extension(landmarks)
        base = self.calculate_wrist_rotation(landmarks)

        current_time = time.time()
        if current_time - self.last_command_time >= self.command_interval:
            if (abs(claw - self.prev_claw) > 5 or
                abs(height - self.prev_height) > 5 or
                abs(extension - self.prev_extension) > 5 or
                abs(base - self.prev_base) > 5):

                self.send_command(f"C{claw}")
                time.sleep(0.05)
                self.send_command(f"H{height}")
                time.sleep(0.05)
                self.send_command(f"E{extension}")
                time.sleep(0.05)
                self.send_command(f"B{base}")

                print(f"Sent -> Claw:{claw} Height:{height} Ext:{extension} Base:{base}")

                self.prev_claw, self.prev_height = claw, height
                self.prev_extension, self.prev_base = extension, base
            self.last_command_time = current_time

    # --- Main loop ---
    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)

            if results.multi_hand_landmarks:
                landmarks = results.multi_hand_landmarks[0].landmark
                self.mirror_hand_to_servos(landmarks)
                self.mp_draw.draw_landmarks(frame, results.multi_hand_landmarks[0], self.mp_hands.HAND_CONNECTIONS)

            cv2.imshow("Hand Control", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                self.send_command("C90")
                self.send_command("H45")
                self.send_command("E45")
                self.send_command("B90")
                print("🔄 Resetting to default")

        self.cleanup()

    def cleanup(self):
        if self.arduino:
            self.arduino.close()
        self.cap.release()
        cv2.destroyAllWindows()
        print("✅ Exit cleanly")

if __name__ == "__main__":
    # on mac use this controller = HandMirrorController('/dev/cu.usbmodem101')
    controller = HandMirrorController('/dev/ttyACM0') # on linux use this
    controller.run()
