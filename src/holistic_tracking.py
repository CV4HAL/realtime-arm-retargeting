"""
CV4HAL - Real-Time Arm Retargeting
Holistic tracking: body pose + full finger landmarks.

Adds to pose_tracking.py:
  - 21 landmarks per hand, so every finger joint is tracked
  - a pinch/grip value, which is what will eventually drive the gripper
  - an operator-presence check (face detected = a real human is in frame)
  - an FPS readout, which is our first latency measurement

Face landmarks are NOT drawn. They are only used to confirm a human
operator is present, so the system is not driven by a poster, a reflection,
or a mannequin.

Run:   python src/holistic_tracking.py
       python src/holistic_tracking.py --camera 1     (if camera 0 is wrong)
Quit:  press q (or Esc) in the video window
"""

import argparse
import time

import cv2
import mediapipe as mp
import numpy as np

mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles


# ----------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------

def calculate_angle(a, b, c):
    """
    Angle at point b formed by segments b->a and b->c, in degrees (0-180).
    Example: elbow angle = calculate_angle(shoulder, elbow, wrist)
    """
    a, b, c = np.array(a), np.array(b), np.array(c)
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - \
              np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(np.degrees(radians))
    if angle > 180.0:
        angle = 360.0 - angle
    return angle


def xy(landmarks, index):
    """Normalized [x, y] of one landmark by index."""
    lm = landmarks[index]
    return [lm.x, lm.y]


def grip_value(hand_landmarks):
    """
    A scale-invariant pinch measure from the hand landmarks.

    Distance between thumb tip (4) and index tip (8), divided by hand size
    (wrist 0 to middle-finger MCP 9). Dividing by hand size means the value
    does not change when the operator moves closer to or further from the
    camera.

    Returns roughly 0.0 (pinched shut) to 1.0+ (wide open).
    This is the signal that will map to the Lite 6 gripper.
    """
    lm = hand_landmarks.landmark

    thumb_tip = np.array([lm[4].x, lm[4].y])
    index_tip = np.array([lm[8].x, lm[8].y])
    wrist = np.array([lm[0].x, lm[0].y])
    middle_mcp = np.array([lm[9].x, lm[9].y])

    pinch = np.linalg.norm(thumb_tip - index_tip)
    hand_size = np.linalg.norm(wrist - middle_mcp)

    if hand_size < 1e-6:          # guard against a degenerate frame
        return 0.0
    return pinch / hand_size


def scaled_points(hand_landmarks, indices, width, height):
    lm = hand_landmarks.landmark
    scale = np.array([width, height, width])
    return np.array([[lm[i].x, lm[i].y, lm[i].z] for i in indices]) * scale


def hand_openness(hand_landmarks, width, height):
    wrist, middle_mcp, *tips = scaled_points(hand_landmarks, (0, 9, 8, 12, 16, 20), width, height)

    hand_size = np.linalg.norm(wrist - middle_mcp)
    if hand_size < 1e-6:
        return 0.0
    return np.mean(np.linalg.norm(np.array(tips) - wrist, axis=1)) / hand_size


# ----------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------

def main(camera_index=0):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Could not open camera {camera_index}.")
        print("Try a different index:  python src/holistic_tracking.py --camera 1")
        return

    prev_time = time.time()
    fps_smooth = 0.0

    with mp_holistic.Holistic(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=1,
        refine_face_landmarks=False,   # we don't need detailed face mesh
    ) as holistic:

        while cap.isOpened():
            ok, frame = cap.read()
            if not ok:
                print("Dropped a frame, retrying...")
                continue

            # Process the UNFLIPPED frame so MediaPipe's left/right hand
            # labels match the operator's actual anatomy. We mirror only
            # for display, further down.
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = holistic.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # ---------- draw skeleton and hands (not the face) ----------
            if results.pose_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    results.pose_landmarks,
                    mp_holistic.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_styles.get_default_pose_landmarks_style(),
                )

            for hand in (results.left_hand_landmarks,
                         results.right_hand_landmarks):
                if hand:
                    mp_drawing.draw_landmarks(
                        image,
                        hand,
                        mp_holistic.HAND_CONNECTIONS,
                        landmark_drawing_spec=mp_styles.get_default_hand_landmarks_style(),
                        connection_drawing_spec=mp_styles.get_default_hand_connections_style(),
                    )

            # ---------- compute the values we care about ----------
            elbow_angle = None
            shoulder_angle = None
            grip = None

            if results.pose_landmarks:
                p = results.pose_landmarks.landmark
                L = mp_holistic.PoseLandmark

                shoulder = xy(p, L.RIGHT_SHOULDER.value)
                elbow    = xy(p, L.RIGHT_ELBOW.value)
                wrist    = xy(p, L.RIGHT_WRIST.value)
                hip      = xy(p, L.RIGHT_HIP.value)

                elbow_angle    = calculate_angle(shoulder, elbow, wrist)
                shoulder_angle = calculate_angle(hip, shoulder, elbow)

            if results.right_hand_landmarks:
                grip = grip_value(results.right_hand_landmarks)

            # Face is used ONLY as a presence check, never drawn
            operator_present = results.face_landmarks is not None

            # ---------- FPS (our first latency figure) ----------
            now = time.time()
            dt = now - prev_time
            prev_time = now
            if dt > 0:
                fps_now = 1.0 / dt
                # exponential smoothing so the number is readable
                fps_smooth = fps_now if fps_smooth == 0 else \
                             0.9 * fps_smooth + 0.1 * fps_now

            # ---------- mirror for display, then draw the text ----------
            image = cv2.flip(image, 1)

            cv2.rectangle(image, (0, 0), (330, 132), (30, 30, 30), -1)

            def put(text, row, color=(255, 255, 255)):
                cv2.putText(image, text, (12, 28 + row * 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA)

            put(f"Elbow:    {elbow_angle:5.1f} deg" if elbow_angle is not None
                else "Elbow:      --", 0)
            put(f"Shoulder: {shoulder_angle:5.1f} deg" if shoulder_angle is not None
                else "Shoulder:   --", 1)
            put(f"Grip:     {grip:5.2f}" if grip is not None
                else "Grip:       --  (show right hand)", 2)
            put(f"FPS:      {fps_smooth:5.1f}", 3, (180, 220, 180))

            if operator_present:
                put("Operator: CONFIRMED", 4, (120, 230, 120))
            else:
                put("Operator: not detected", 4, (80, 180, 255))

            cv2.imshow("CV4HAL - Holistic Tracking (press q to quit)", image)

            if cv2.waitKey(5) & 0xFF in (ord("q"), 27):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CV4HAL holistic tracking")
    parser.add_argument("--camera", type=int, default=0,
                        help="camera index (default 0; try 1 if that fails)")
    args = parser.parse_args()
    main(camera_index=args.camera)
    