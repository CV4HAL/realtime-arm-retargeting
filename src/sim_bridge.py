import argparse
import threading
import time

import cv2
import mediapipe as mp
import numpy as np

from holistic_tracking import grip_value, hand_openness, scaled_points

mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles
PoseLandmark = mp_holistic.PoseLandmark

BOX_CENTER_MM = np.array([250.0, 0.0, 200.0])
BOX_HALF_SIZE_MM = np.array([100.0, 150.0, 100.0])
HOME_SPEED_DEG_S = 30
MOVE_SPEED_MM_S = 100

FORWARD_NEUTRAL = 0.45
FORWARD_RANGE = 0.35

EMA_ALPHA = 0.3
MAX_STEP_MM = 8.0
MAX_ANGLE_STEP_DEG = 2.0
SERVO_RATE_HZ = 30
MIN_VISIBILITY = 0.5
RECOVERY_COOLDOWN_S = 2.0

HAND_ANGLE_RANGE_DEG = np.array([60.0, 45.0, 60.0])
ORIENTATION_RANGE_DEG = np.array([30.0, 30.0, 45.0])
ORIENTATION_DEADZONE_DEG = 8.0
FIST_ON_THRESHOLD = 1.15
FIST_OFF_THRESHOLD = 1.4

WINDOW_NAME = "CV4HAL - Sim Bridge (press q to quit)"


def arm_to_offset(world_landmarks):
    lm = world_landmarks.landmark
    shoulder = lm[PoseLandmark.RIGHT_SHOULDER.value]
    elbow = lm[PoseLandmark.RIGHT_ELBOW.value]
    wrist = lm[PoseLandmark.RIGHT_WRIST.value]

    p_shoulder = np.array([shoulder.x, shoulder.y, shoulder.z])
    p_elbow = np.array([elbow.x, elbow.y, elbow.z])
    p_wrist = np.array([wrist.x, wrist.y, wrist.z])

    arm_length = np.linalg.norm(p_elbow - p_shoulder) + np.linalg.norm(p_wrist - p_elbow)
    if arm_length < 1e-6:
        return None

    dx, dy, dz = (p_wrist - p_shoulder) / arm_length
    forward = (-dz - FORWARD_NEUTRAL) / FORWARD_RANGE
    offset = np.array([forward, dx, -dy])
    return np.clip(offset, -1.0, 1.0)


def arm_visible(pose_landmarks):
    lm = pose_landmarks.landmark
    joints = (PoseLandmark.RIGHT_SHOULDER, PoseLandmark.RIGHT_ELBOW, PoseLandmark.RIGHT_WRIST)
    return all(lm[j.value].visibility >= MIN_VISIBILITY for j in joints)


def offset_to_target(offset):
    return BOX_CENTER_MM + offset * BOX_HALF_SIZE_MM


def hand_to_angles(hand_landmarks, width, height):
    wrist, index_mcp, middle_mcp, pinky_mcp = scaled_points(
        hand_landmarks, (0, 5, 9, 17), width, height)

    forward = middle_mcp - wrist
    normal = np.cross(index_mcp - wrist, pinky_mcp - wrist)
    forward_norm = np.linalg.norm(forward)
    normal_norm = np.linalg.norm(normal)
    if forward_norm < 1e-6 or normal_norm < 1e-6:
        return None
    forward /= forward_norm
    normal /= normal_norm

    roll = np.degrees(np.arctan2(normal[0], normal[2]))
    pitch = np.degrees(np.arcsin(np.clip(forward[2], -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(forward[0], -forward[1]))
    return np.array([roll, pitch, yaw])


def angles_to_offset(angles):
    shrunk = np.sign(angles) * np.maximum(np.abs(angles) - ORIENTATION_DEADZONE_DEG, 0.0)
    span = HAND_ANGLE_RANGE_DEG - ORIENTATION_DEADZONE_DEG
    return np.clip(shrunk / span, -1.0, 1.0) * ORIENTATION_RANGE_DEG


class Clutch:
    def __init__(self):
        self.engaged = False

    def update(self, openness):
        if self.engaged and openness > FIST_OFF_THRESHOLD:
            self.engaged = False
        elif not self.engaged and openness < FIST_ON_THRESHOLD:
            self.engaged = True
        return self.engaged


class TargetFilter:
    def __init__(self, initial, alpha=EMA_ALPHA):
        self.value = np.array(initial, dtype=float)
        self.alpha = alpha

    def update(self, raw):
        self.value = self.alpha * np.asarray(raw) + (1.0 - self.alpha) * self.value
        return self.value.copy()


def limit_step(current, desired, max_step=MAX_STEP_MM):
    delta = desired - current
    distance = np.linalg.norm(delta)
    if distance <= max_step:
        return desired.copy()
    return current + delta * (max_step / distance)


def wrap_angle(angles):
    return np.where(np.abs(angles) > 180.0, angles - 360.0 * np.sign(angles), angles)


class RobotLink:
    def __init__(self, ip, rate_hz):
        self.ip = ip
        self.period = 1.0 / rate_hz
        self.arm = None
        self.orientation = None
        self.command = BOX_CENTER_MM.copy()
        self.goal = BOX_CENTER_MM.copy()
        self.orientation_command = np.zeros(3)
        self.orientation_goal = np.zeros(3)
        self.status = "OFFLINE"
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None

    def connect(self):
        from xarm.wrapper import XArmAPI

        self.status = "CONNECTING"
        self.arm = XArmAPI(self.ip)
        if not self.arm.connected:
            raise ConnectionError(f"Could not connect to the robot at {self.ip}")
        self.prepare()
        self.go_home()
        self.start_servo()

    def prepare(self):
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

    def go_home(self):
        self.status = "HOMING"
        self.arm.move_gohome(speed=HOME_SPEED_DEG_S, wait=True)
        _, pose = self.arm.get_position()
        self.orientation = list(pose[3:6])
        self.arm.set_position(*BOX_CENTER_MM, *self.orientation,
                              speed=MOVE_SPEED_MM_S, wait=True)

    def start_servo(self):
        self.arm.set_mode(1)
        self.arm.set_state(0)
        self.status = "SERVO"
        self.thread = threading.Thread(target=self.servo_loop, daemon=True)
        self.thread.start()

    def set_goal(self, target, orientation_offset=None):
        with self.lock:
            self.goal = np.asarray(target, dtype=float)
            if orientation_offset is not None:
                self.orientation_goal = np.asarray(orientation_offset, dtype=float)

    def servo_loop(self):
        next_tick = time.perf_counter()
        last_recovery = 0.0
        while not self.stop_event.is_set():
            with self.lock:
                goal = self.goal.copy()
                orientation_goal = self.orientation_goal.copy()
            self.command = limit_step(self.command, goal)
            self.orientation_command = limit_step(self.orientation_command, orientation_goal,
                                                  MAX_ANGLE_STEP_DEG)
            angles = wrap_angle(np.asarray(self.orientation) + self.orientation_command)
            code = self.arm.set_servo_cartesian([*self.command, *angles], is_radian=False)
            if code != 0 or self.arm.error_code != 0:
                self.status = f"ERROR {code}/{self.arm.error_code}"
                now = time.perf_counter()
                if now - last_recovery > RECOVERY_COOLDOWN_S:
                    last_recovery = now
                    self.recover()
            else:
                self.status = "SERVO"
            next_tick += self.period
            time.sleep(max(0.0, next_tick - time.perf_counter()))

    def recover(self):
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1)
        self.arm.set_state(0)

    def shutdown(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.arm is None:
            return
        try:
            self.arm.set_state(4)
            self.arm.set_mode(0)
            self.arm.set_state(0)
        finally:
            self.arm.disconnect()
            self.status = "DISCONNECTED"


def operator_hands(results, swap):
    left, right = results.left_hand_landmarks, results.right_hand_landmarks
    return (right, left) if swap else (left, right)


def draw_hand_label(image, hand_landmarks, text):
    height, width = image.shape[:2]
    wrist = hand_landmarks.landmark[0]
    x = width - int(wrist.x * width)
    y = int(wrist.y * height)
    cv2.putText(image, text, (x - 10, y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)


def draw_overlay(image, fps, target, grip, tracking, robot_status, orientation=None, clutch=None):
    cv2.rectangle(image, (0, 0), (360, 158 if orientation is None else 210), (30, 30, 30), -1)

    def put(text, row, color=(255, 255, 255)):
        cv2.putText(image, text, (12, 28 + row * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA)

    put(f"FPS:      {fps:5.1f}", 0, (180, 220, 180))
    put(f"Target:   {target[0]:6.1f} {target[1]:6.1f} {target[2]:6.1f} mm", 1)
    put(f"Grip:     {grip:5.2f}" if grip is not None else "Grip:       --", 2)
    put("Tracking: OK" if tracking else "Tracking: LOST (holding pose)", 3,
        (120, 230, 120) if tracking else (80, 180, 255))
    put(f"Robot:    {robot_status}", 4)
    if orientation is not None:
        put(f"RPY:      {orientation[0]:5.1f} {orientation[1]:5.1f} {orientation[2]:5.1f} deg", 5)
        put("Clutch:   FROZEN (fist)" if clutch else "Clutch:   LIVE (open)", 6,
            (80, 180, 255) if clutch else (120, 230, 120))


def run(args):
    robot = None if args.dry_run else RobotLink(args.ip, args.rate)
    cap = cv2.VideoCapture(args.camera)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera {args.camera}")
        if robot is not None:
            robot.connect()

        target_filter = TargetFilter(BOX_CENTER_MM)
        target = BOX_CENTER_MM.copy()
        orientation_filter = TargetFilter(np.zeros(3))
        orientation = np.zeros(3)
        clutch = Clutch()
        grip = None
        fps = 0.0
        prev_time = time.perf_counter()

        with mp_holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=1,
        ) as holistic:
            while True:
                ok, frame = cap.read()
                if not ok:
                    continue

                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = holistic.process(image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                tracking = (results.pose_landmarks is not None
                            and results.pose_world_landmarks is not None
                            and arm_visible(results.pose_landmarks))
                if tracking:
                    offset = arm_to_offset(results.pose_world_landmarks)
                    if offset is not None:
                        target = target_filter.update(offset_to_target(offset))
                        if robot is not None:
                            robot.set_goal(target)
                    else:
                        tracking = False

                left_hand, right_hand = operator_hands(results, args.swap_hands)

                if right_hand:
                    grip = grip_value(right_hand)

                if args.dual_hand and left_hand:
                    height, width = frame.shape[:2]
                    clutch.update(hand_openness(left_hand, width, height))
                    if not clutch.engaged:
                        angles = hand_to_angles(left_hand, width, height)
                        if angles is not None:
                            orientation = orientation_filter.update(angles_to_offset(angles))

                if args.dual_hand and robot is not None:
                    robot.set_goal(target, orientation)

                if results.pose_landmarks:
                    mp_drawing.draw_landmarks(
                        image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS,
                        landmark_drawing_spec=mp_styles.get_default_pose_landmarks_style())

                now = time.perf_counter()
                dt = now - prev_time
                prev_time = now
                if dt > 0:
                    fps = 1.0 / dt if fps == 0 else 0.9 * fps + 0.1 * (1.0 / dt)

                image = cv2.flip(image, 1)
                for hand, text in ((left_hand, "L"), (right_hand, "R")):
                    if hand:
                        draw_hand_label(image, hand, text)
                status = "DRY RUN" if robot is None else robot.status
                if args.dual_hand:
                    draw_overlay(image, fps, target, grip, tracking, status,
                                 orientation, clutch.engaged)
                else:
                    draw_overlay(image, fps, target, grip, tracking, status)
                cv2.imshow(WINDOW_NAME, image)

                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if robot is not None:
            robot.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(description="CV4HAL MediaPipe to UFACTORY Lite 6 bridge")
    parser.add_argument("--ip", default="127.0.0.1", help="robot or simulator IP")
    parser.add_argument("--camera", type=int, default=0, help="camera index")
    parser.add_argument("--rate", type=float, default=SERVO_RATE_HZ, help="servo rate in Hz")
    parser.add_argument("--dual-hand", action="store_true",
                        help="left hand sets the TCP orientation, closed fist freezes it")
    parser.add_argument("--swap-hands", action="store_true",
                        help="swap the left and right hand assignment")
    parser.add_argument("--dry-run", action="store_true",
                        help="run tracking and compute targets without connecting to the robot")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
