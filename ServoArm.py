import cv2
import mediapipe as mp
import serial
import time
import numpy as np
import math

# FOR lINUX USE '/dev/ttyACM0'
# FOR MAC USE '/dev/cu.usbmodem101'

class HandMirrorController:
    def __init__(self, arduino_port='/dev/ttyACM0', baud_rate=9600, show_frame=False):
        self.show_frame = show_frame

        # --- MediaPipe Hands ---
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=0
        )
        self.mp_draw = mp.solutions.drawing_utils

        # --- MediaPipe Pose ---
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # --- Arduino setup ---
        self.arduino = None
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        if arduino_port:
            try:
                self.arduino = serial.Serial(arduino_port, baud_rate, timeout=0.05)
                time.sleep(2)
                print(f"✅ Connected to Arduino on {arduino_port}")
            except Exception as e:
                print(f"❌ Could not connect to Arduino: {e}")
                self.arduino = None
        else:
            print("⚠️ Skipping Arduino connection (dry-run)")

        # --- Camera setup ---
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # --- Control rate ---
        self.last_command_time = 0
        self.command_interval = 0.05

        # --- Previous servo values ---
        self.prev_claw = 45
        self.prev_height = 45
        self.prev_extension = 45
        self.prev_base = 90

        # --- Smoothing (keep original responsive value) ---
        self.smooth_alpha = 0.8

        # --- DEADZONES (only ignore tiny jitter) ---
        self.deadzone_claw = 3      # Small deadzone to prevent jitter
        self.deadzone_height = 2    
        self.deadzone_extension = 2
        self.deadzone_base = 5

        # --- Frame processing ---
        self.frame_skip = 1
        self.frame_count = 0

        print("🤚 Hand Mirror Controller Ready! Press 'q' to quit, 'r' to reset servos.")

    def send_command(self, command):
        if self.arduino:
            try:
                self.arduino.write((command + '\n').encode())
                self.arduino.flush()
                return True
            except Exception as e:
                print(f"Arduino error: {e}")
        return False

    def apply_deadzone(self, new_value, old_value, deadzone):
        """Only update if change is larger than deadzone"""
        if abs(new_value - old_value) < deadzone:
            return old_value
        return new_value

    def calculate_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def calculate_fist_closure(self, landmarks):
        """ORIGINAL WORKING METHOD - but snaps to 0 or 90"""
        fingertips = [4, 8, 12, 16, 20]
        joints = [3, 6, 10, 14, 18]
        curl_scores = []
        for tip, joint in zip(fingertips, joints):
            tip_pos, joint_pos, wrist_pos = landmarks[tip], landmarks[joint], landmarks[0]
            wrist_to_tip = self.calculate_distance(wrist_pos, tip_pos)
            wrist_to_joint = self.calculate_distance(wrist_pos, joint_pos)
            curl_scores.append(wrist_to_tip / (wrist_to_joint + 1e-6))
        avg_curl = sum(curl_scores) / len(curl_scores)
        raw_angle = int(np.clip((avg_curl - 0.8) * 300, 0, 90))
        
        # Snap to closest: 0 (open) or 90 (closed)
        if raw_angle < 45:
            return 0
        else:
            return 90

    def calculate_forearm_pitch(self, pose_landmarks):
        """ORIGINAL WORKING METHOD - clamped to 0-90°"""
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
        servo_angle = int(np.clip(-angle_deg + 90, 0, 90))  # Cap at 90
        return servo_angle

    def calculate_hand_extension(self, pose_landmarks, hand_landmarks):
        """ORIGINAL WORKING METHOD - clamped to 0-90°"""
        shoulder = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        wrist = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]

        dx = wrist.x - shoulder.x
        dy = wrist.y - shoulder.y
        distance = math.sqrt(dx**2 + dy**2)

        min_distance = 0.1
        max_distance = 0.4

        extension_ratio = (distance - min_distance) / (max_distance - min_distance)
        extension_angle = int(np.clip(extension_ratio * 90, 0, 90))  # Cap at 90
        return extension_angle

    def mirror_hand_to_servos(self, pose_landmarks, hand_landmarks):
        # Calculate raw target angles (ORIGINAL formulas)
        claw_raw = self.calculate_fist_closure(hand_landmarks)
        height_raw = self.calculate_forearm_pitch(pose_landmarks)
        extension_raw = self.calculate_hand_extension(pose_landmarks, hand_landmarks)
        base_target = 90

        # Apply deadzones to RAW values (before smoothing)
        claw_target = self.apply_deadzone(claw_raw, self.prev_claw, self.deadzone_claw)
        height_target = self.apply_deadzone(height_raw, self.prev_height, self.deadzone_height)
        extension_target = self.apply_deadzone(extension_raw, self.prev_extension, self.deadzone_extension)

        # Apply smoothing
        claw = int(self.prev_claw + self.smooth_alpha * (claw_target - self.prev_claw))
        height = int(self.prev_height + self.smooth_alpha * (height_target - self.prev_height))
        extension = int(self.prev_extension + self.smooth_alpha * (extension_target - self.prev_extension))
        base = base_target

        now = time.time()
        if now - self.last_command_time >= self.command_interval:
            command = f"C{claw}H{height}E{extension}B{base}"
            sent = self.send_command(command)
            if sent:
                print(f"Sent -> {command}", end='\r')

            self.prev_claw, self.prev_height = claw, height
            self.prev_extension, self.prev_base = extension, base
            self.last_command_time = now

    def run(self):
        print("🎥 Starting camera loop...")
        
        # Force window creation if showing frame
        if self.show_frame:
            cv2.namedWindow("Hand Control", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Hand Control", 800, 600)
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("❌ Failed to read frame")
                time.sleep(0.1)
                continue  # Try again instead of breaking
            
            self.frame_count += 1
            frame = cv2.flip(frame, 1)
            
            if self.frame_count % self.frame_skip == 0:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                pose_results = self.pose.process(rgb)
                hands_results = self.hands.process(rgb)

                if pose_results.pose_landmarks and hands_results.multi_hand_landmarks:
                    pose_landmarks = pose_results.pose_landmarks.landmark
                    hand_landmarks = hands_results.multi_hand_landmarks[0].landmark
                    self.mirror_hand_to_servos(pose_landmarks, hand_landmarks)

                    if self.show_frame:
                        # Draw landmarks with better styling
                        self.mp_draw.draw_landmarks(
                            frame, 
                            hands_results.multi_hand_landmarks[0], 
                            self.mp_hands.HAND_CONNECTIONS,
                            landmark_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(
                                color=(0, 255, 0), thickness=2, circle_radius=3
                            ),
                            connection_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(
                                color=(0, 200, 0), thickness=2
                            )
                        )
                        mp.solutions.drawing_utils.draw_landmarks(
                            frame, 
                            pose_results.pose_landmarks, 
                            self.mp_pose.POSE_CONNECTIONS,
                            landmark_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(
                                color=(255, 100, 0), thickness=2, circle_radius=2
                            ),
                            connection_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(
                                color=(255, 150, 0), thickness=2
                            )
                        )
                
                if self.show_frame:
                    # Create a dark overlay for better text readability
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (0, 0), (320, 140), (0, 0, 0), -1)
                    frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
                    
                    # Status indicator
                    status_color = (0, 255, 0) if pose_results.pose_landmarks and hands_results.multi_hand_landmarks else (0, 0, 255)
                    cv2.circle(frame, (15, 20), 8, status_color, -1)
                    cv2.putText(frame, "TRACKING" if status_color == (0, 255, 0) else "NO DETECTION", 
                              (30, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Servo values with icons
                    claw_status = "CLOSED" if self.prev_claw > 45 else "OPEN"
                    cv2.putText(frame, f"CLAW:      {claw_status} ({self.prev_claw})", 
                              (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 2)
                    cv2.putText(frame, f"HEIGHT:    {self.prev_height}", 
                              (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 2)
                    cv2.putText(frame, f"EXTENSION: {self.prev_extension}", 
                              (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 2)
                    
                    # Instructions at bottom
                    cv2.putText(frame, "Press 'Q' to quit | 'R' to reset", 
                              (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            if self.show_frame:
                cv2.imshow("Hand Control", frame)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('r'):
                self.send_command("C45H45E45B90")
                print("\n🔄 Resetting to default")

        self.cleanup()

    def cleanup(self):
        if self.arduino:
            self.arduino.close()
        self.cap.release()
        cv2.destroyAllWindows()
        print("\n✅ Exit cleanly")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Hand mirror controller for ServoArm')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0', 
                       help='Arduino serial port')
    parser.add_argument('--baud', type=int, default=9600, help='Serial baud rate')
    parser.add_argument('--show-frame', action='store_true', help='Show camera frames with landmarks')
    parser.add_argument('--dry-run', action='store_true', help='Do not try to connect to an Arduino')
    parser.add_argument('--deadzone-claw', type=int, default=3, help='Claw deadzone in degrees')
    parser.add_argument('--deadzone-height', type=int, default=2, help='Height deadzone in degrees')
    parser.add_argument('--deadzone-ext', type=int, default=2, help='Extension deadzone in degrees')

    args = parser.parse_args()
    port = None if args.dry_run else args.port
    controller = HandMirrorController(port, baud_rate=args.baud, show_frame=args.show_frame)
    
    # Allow command-line tuning of deadzones
    controller.deadzone_claw = args.deadzone_claw
    controller.deadzone_height = args.deadzone_height
    controller.deadzone_extension = args.deadzone_ext
    
    controller.run()