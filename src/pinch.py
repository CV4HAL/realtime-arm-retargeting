"""Pinch gesture detection for the right hand (thumb tip touching a fingertip)."""
import numpy as np

from holistic_tracking import scaled_points

THUMB_TIP = 4
WRIST = 0
MIDDLE_MCP = 9

# Pinch distance (thumb tip to fingertip) divided by hand size (wrist to
# middle-finger MCP). The gap between the two thresholds is the hysteresis band.
PINCH_ON_RATIO = 0.25
PINCH_OFF_RATIO = 0.40
PINCH_MIN_FRAMES_ON = 3
PINCH_MIN_FRAMES_OFF = 3

# Gesture name -> fingertip landmark. The order is also the tie-break priority
# when several pinches start at the same time: index > middle > ring.
PINCH_FINGERS = {"index": 8, "middle": 12, "ring": 16}

# Axis locked while each gesture is active (0 = X, 1 = Y, 2 = Z of the TCP target).
MIDDLE_PINCH_AXIS = 2
RING_PINCH_AXIS = 0
AXIS_NAMES = ("X", "Y", "Z")
GESTURE_AXIS = {"middle": MIDDLE_PINCH_AXIS, "ring": RING_PINCH_AXIS}

# Two pinch ratios closer than this are treated as a tie.
PINCH_TIE_MARGIN = 0.03


def pinch_ratios(hand_landmarks, width, height):
    """Thumb-to-fingertip distance per finger, normalised by hand size."""
    indices = (WRIST, MIDDLE_MCP, THUMB_TIP, *PINCH_FINGERS.values())
    wrist, middle_mcp, thumb, *tips = scaled_points(hand_landmarks, indices, width, height)
    hand_size = np.linalg.norm(wrist - middle_mcp)
    if hand_size < 1e-6:
        return None
    return {name: float(np.linalg.norm(tip - thumb) / hand_size)
            for name, tip in zip(PINCH_FINGERS, tips)}


class PinchState:
    """Hysteresis plus a minimum frame count for one finger."""

    def __init__(self):
        self.active = False
        self.streak = 0

    def update(self, ratio):
        wants_change = (ratio > PINCH_OFF_RATIO) if self.active else (ratio < PINCH_ON_RATIO)
        self.streak = self.streak + 1 if wants_change else 0
        needed = PINCH_MIN_FRAMES_OFF if self.active else PINCH_MIN_FRAMES_ON
        if self.streak >= needed:
            self.active = not self.active
            self.streak = 0
        return self.active


class PinchDetector:
    """Reports the single active pinch gesture, or None.

    Priority when more than one pinch is active: a gesture that is already
    active keeps priority until it is released. Among gestures that become
    active together, the tightest pinch (smallest ratio) wins; ratios within
    PINCH_TIE_MARGIN are resolved by finger order (index > middle > ring).
    """

    def __init__(self):
        self.states = {name: PinchState() for name in PINCH_FINGERS}
        self.current = None
        self.ratios = None

    def reset(self):
        self.__init__()

    def update(self, hand_landmarks, width, height):
        ratios = pinch_ratios(hand_landmarks, width, height)
        if ratios is None:
            return self.current
        self.ratios = ratios
        active = [name for name, state in self.states.items() if state.update(ratios[name])]
        if self.current in active:
            return self.current
        if not active:
            self.current = None
            return None
        best = min(ratios[name] for name in active)
        self.current = next(name for name in PINCH_FINGERS
                            if name in active and ratios[name] <= best + PINCH_TIE_MARGIN)
        return self.current


class AxisLock:
    """Freezes every axis except the chosen one while a gesture is active."""

    def __init__(self):
        self.axis = None
        self.reference = None

    def update(self, axis, target, held=None):
        """Return the target to command. `held` is the last commanded target."""
        if axis is None:
            self.axis = None
            return target
        if axis != self.axis:
            base = held if held is not None else target
            self.reference = np.array(base, dtype=float)
            self.axis = axis
        locked = self.reference.copy()
        locked[axis] = target[axis]
        return locked
