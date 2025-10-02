import cv2
import mediapipe as mp
import serial
import time
import numpy as np
import math

class HandMirrorController:
    def __init__(self, arduino_port='/dev/cu.usbmodem101', baud_rate=9600, show_frame=False):
        self.show_frame = show_frame

        # --- MediaPipe Hands ---
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils

        # --- MediaPipe Pose ---
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        # --- Arduino setup ---
        try:
            self.arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to Arduino on {arduino_port}")
        except Exception as e:
            print(f"❌ Could not connect to Arduino: {e}")
            self.arduino = None

        # --- Camera setup ---
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # reduced resolution
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # --- Control rate ---
        self.last_command_time = 0
        self.command_interval = 0.5

        # --- Previous servo values ---
        self.prev_claw = 90
        self.prev_height = 0
        self.prev_extension = 45
        self.prev_base = 90

        print("🤚 Hand Mirror Controller Ready! Press 'q' to quit, 'r' to reset servos.")

    def send_command(self, command):
        if self.arduino:
            try:
                self.arduino.write((command + '\n').encode())
                return True
            except Exception as e:
                print(f"Arduino error: {e}")
        return False

    # --- Utility functions ---
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
        deg = math.degrees(angle)
        base_angle = int(np.clip(90 + deg / 90 * 20, 70, 110))
        return base_angle

    def calculate_forearm_pitch(self, pose_landmarks):
        shoulder = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        elbow = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW]
        wrist = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]

        forearm_vec = np.array([wrist.x - elbow.x, wrist.y - elbow.y])
        upper_arm_vec = np.array([elbow.x - shoulder.x, elbow.y - shoulder.y])

        dot = np.dot(forearm_vec, upper_arm_vec)
        mag_forearm = np.linalg.norm(forearm_vec)
        mag_upper = np.linalg.norm(upper_arm_vec)
        angle_rad = math.acos(np.clip(dot / (mag_forearm * mag_upper + 1e-6), -1.0, 1.0))
        angle_deg = math.degrees(angle_rad)
        servo_angle = int(np.clip(-angle_deg + 90, 0, 90))
        return servo_angle

    def calculate_hand_extension(self, pose_landmarks, hand_landmarks):
        shoulder = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        wrist = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]

        dx = wrist.x - shoulder.x
        dy = wrist.y - shoulder.y
        distance = math.sqrt(dx**2 + dy**2)

        min_distance = 0.1
        max_distance = 0.4

        extension_ratio = (distance - min_distance) / (max_distance - min_distance)
        extension_angle = int(np.clip(extension_ratio * 90, 0, 90))
        return extension_angle

    # --- Mirror to Arduino ---
    def mirror_hand_to_servos(self, pose_landmarks, hand_landmarks):
        claw = self.calculate_fist_closure(hand_landmarks)
        height = self.calculate_forearm_pitch(pose_landmarks)
        extension = self.calculate_hand_extension(pose_landmarks, hand_landmarks)
        base = self.calculate_wrist_rotation(hand_landmarks)

        now = time.time()
        if now - self.last_command_time >= self.command_interval:
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
            self.last_command_time = now

    # --- Main loop ---
    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            pose_results = self.pose.process(rgb)
            hands_results = self.hands.process(rgb)

            if pose_results.pose_landmarks and hands_results.multi_hand_landmarks:
                pose_landmarks = pose_results.pose_landmarks.landmark
                hand_landmarks = hands_results.multi_hand_landmarks[0].landmark
                self.mirror_hand_to_servos(pose_landmarks, hand_landmarks)

                # Only draw frame if show_frame=True
                if self.show_frame:
                    self.mp_draw.draw_landmarks(frame, hands_results.multi_hand_landmarks[0], self.mp_hands.HAND_CONNECTIONS)
                    mp.solutions.drawing_utils.draw_landmarks(frame, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
                    cv2.imshow("Hand Control", frame)

            if self.show_frame:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    self.send_command("C90")
                    self.send_command("H0")
                    self.send_command("E45")
                    self.send_command("B90")
                    print("🔄 Resetting to default")
            else:
                # Check for 'q' without showing frame
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.cleanup()

    def cleanup(self):
        if self.arduino:
            self.arduino.close()
        self.cap.release()
        cv2.destroyAllWindows()
        print("✅ Exit cleanly")

if __name__ == "__main__":
    controller = HandMirrorController('/dev/ttyACM0', show_frame=True)  # Set show_frame=False for max speed
    controller.run()
