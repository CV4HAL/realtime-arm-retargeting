"""
CV4HAL - Real-Time Arm Retargeting
Step 1: Webcam pose tracking with joint angle extraction.

Reads your webcam, finds your body landmarks with MediaPipe, and computes
the shoulder and elbow angles of your right arm. Those angles are what will
eventually be mapped onto the UFACTORY Lite 6 joints.

Run:   python src/pose_tracking.py
Quit:  press q  (or Esc) in the video window
"""

import cv2
import mediapipe as mp
import numpy as np

# MediaPipe setup
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles


def calculate_angle(a, b, c):
    """
    Angle at point b, formed by the segments b->a and b->c.

    Each point is [x, y]. Returns degrees (0-180).
    Example: elbow angle = calculate_angle(shoulder, elbow, wrist)
    """
    a, b, c = np.array(a), np.array(b), np.array(c)

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - \
              np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(np.degrees(radians))

    # Keep it in 0-180 so it reads like a physical joint angle
    if angle > 180.0:
        angle = 360.0 - angle

    return angle


def get_xy(landmarks, landmark_enum):
    """Pull the normalized [x, y] of one landmark."""
    lm = landmarks[landmark_enum.value]
    return [lm.x, lm.y]


def main():
    cap = cv2.VideoCapture(0)          # 0 = default webcam
    if not cap.isOpened():
        print("Could not open the webcam.")
        print("On Mac: allow camera access for your terminal / VS Code.")
        print("On Windows: close Zoom/Teams/Camera app, or try cv2.VideoCapture(1).")
        return

    with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=1,
    ) as pose:

        while cap.isOpened():
            ok, frame = cap.read()
            if not ok:
                print("Dropped a frame, retrying...")
                continue

            # Mirror the image so moving your right arm looks right on screen
            frame = cv2.flip(frame, 1)

            # MediaPipe wants RGB; OpenCV gives BGR
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = pose.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Extract and display the right-arm angles
            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark

                shoulder = get_xy(lm, mp_pose.PoseLandmark.RIGHT_SHOULDER)
                elbow    = get_xy(lm, mp_pose.PoseLandmark.RIGHT_ELBOW)
                wrist    = get_xy(lm, mp_pose.PoseLandmark.RIGHT_WRIST)
                hip      = get_xy(lm, mp_pose.PoseLandmark.RIGHT_HIP)

                elbow_angle    = calculate_angle(shoulder, elbow, wrist)
                shoulder_angle = calculate_angle(hip, shoulder, elbow)

                cv2.rectangle(image, (0, 0), (300, 80), (30, 30, 30), -1)
                cv2.putText(image, f"Elbow:    {elbow_angle:5.1f} deg",
                            (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, f"Shoulder: {shoulder_angle:5.1f} deg",
                            (12, 62), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2, cv2.LINE_AA)

                # Draw the skeleton
                mp_drawing.draw_landmarks(
                    image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_styles.get_default_pose_landmarks_style(),
                )
            else:
                cv2.putText(image, "No pose detected - step back from the camera",
                            (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 200, 255), 2, cv2.LINE_AA)

            cv2.imshow("CV4HAL - Pose Tracking (press q to quit)", image)

            key = cv2.waitKey(5) & 0xFF
            if key in (ord("q"), 27):      # q or Esc
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()