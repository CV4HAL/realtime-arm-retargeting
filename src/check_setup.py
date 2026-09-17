"""
CV4HAL - Environment check.
Run this FIRST to confirm your setup works before running pose_tracking.py

Run:  python src/check_setup.py
"""

import sys

print("=" * 50)
print("CV4HAL Environment Check")
print("=" * 50)

# --- Python version ---
v = sys.version_info
print(f"\nPython version: {v.major}.{v.minor}.{v.micro}")
if v.major == 3 and v.minor == 11:
    print("  OK - Python 3.11")
elif v.major == 3 and v.minor == 12:
    print("  Probably fine, but 3.11 is what we standardized on.")
else:
    print(f"  PROBLEM - we need Python 3.11. MediaPipe does not support {v.major}.{v.minor}.")
    print("  Install 3.11 from python.org and rebuild the virtual environment.")

# --- Imports ---
ok = True

try:
    import cv2
    print(f"\nOpenCV:    {cv2.__version__}  OK")
except ImportError as e:
    ok = False
    print(f"\nOpenCV:    MISSING  ({e})")

try:
    import mediapipe as mp
    print(f"MediaPipe: {mp.__version__}  OK")
except ImportError as e:
    ok = False
    print(f"MediaPipe: MISSING  ({e})")

try:
    import numpy as np
    print(f"NumPy:     {np.__version__}  OK")
except ImportError as e:
    ok = False
    print(f"NumPy:     MISSING  ({e})")

# --- Camera ---
if ok:
    print("\nChecking the webcam...")
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        grabbed, frame = cap.read()
        if grabbed:
            h, w = frame.shape[:2]
            print(f"  Camera OK - captured a {w}x{h} frame")
        else:
            print("  Camera opened but no frame. Close other apps using it.")
        cap.release()
    else:
        print("  Could not open the camera.")
        print("  Mac:     System Settings > Privacy & Security > Camera")
        print("  Windows: close Zoom/Teams/Camera, or try VideoCapture(1)")

print("\n" + "=" * 50)
if ok:
    print("Setup looks good. Now run:  python src/pose_tracking.py")
else:
    print("Fix the missing packages:  pip install -r requirements.txt")
print("=" * 50)