from types import SimpleNamespace

import numpy as np

from pinch import AxisLock, PinchDetector

WIDTH, HEIGHT = 640, 480


def make_hand(gaps):
    """Hand of size 100 px; `gaps` maps fingertip index -> distance from thumb tip (px)."""
    points = np.zeros((21, 2))
    points[9] = [0, -100]
    points[4] = [-50, -60]
    defaults = {8: 90, 12: 100, 16: 110}
    for tip, gap in {**defaults, **gaps}.items():
        points[tip] = points[4] + [gap, 0]
    points += [WIDTH / 2, HEIGHT / 2]
    return SimpleNamespace(landmark=[
        SimpleNamespace(x=x / WIDTH, y=y / HEIGHT, z=0.0) for x, y in points])


def feed(detector, hand, frames):
    result = None
    for _ in range(frames):
        result = detector.update(hand, WIDTH, HEIGHT)
    return result


def test_open_hand_never_triggers():
    detector = PinchDetector()
    assert feed(detector, make_hand({}), 30) is None


def test_pinch_needs_min_frames_and_releases():
    detector = PinchDetector()
    assert feed(detector, make_hand({8: 10}), 2) is None
    assert feed(detector, make_hand({8: 10}), 1) == "index"
    assert feed(detector, make_hand({8: 10}), 10) == "index"
    assert feed(detector, make_hand({}), 5) is None


def test_hysteresis_band_does_not_flicker():
    detector = PinchDetector()
    feed(detector, make_hand({8: 10}), 5)
    assert feed(detector, make_hand({8: 32}), 10) == "index"   # between on and off
    assert feed(detector, make_hand({8: 60}), 5) is None


def test_single_frame_glitch_is_ignored():
    detector = PinchDetector()
    feed(detector, make_hand({12: 10}), 5)
    assert detector.update(make_hand({}), WIDTH, HEIGHT) == "middle"


def test_tightest_pinch_wins_when_started_together():
    detector = PinchDetector()
    assert feed(detector, make_hand({8: 20, 12: 5}), 5) == "middle"


def test_tie_resolved_by_finger_order():
    detector = PinchDetector()
    assert feed(detector, make_hand({8: 12, 12: 10}), 5) == "index"


def test_active_gesture_keeps_priority():
    detector = PinchDetector()
    feed(detector, make_hand({8: 20}), 5)
    assert feed(detector, make_hand({8: 20, 12: 2}), 10) == "index"


def test_axis_lock_freezes_other_axes():
    lock = AxisLock()
    held = np.array([1.0, 2.0, 3.0])
    out = lock.update(2, np.array([9.0, 9.0, 9.0]), held)
    assert out.tolist() == [1.0, 2.0, 9.0]
    out = lock.update(2, np.array([5.0, 5.0, 4.0]), out)
    assert out.tolist() == [1.0, 2.0, 4.0]
    assert lock.update(None, np.array([7.0, 7.0, 7.0])).tolist() == [7.0, 7.0, 7.0]
