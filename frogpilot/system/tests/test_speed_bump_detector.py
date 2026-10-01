import math
from pathlib import Path

import numpy as np
import pytest

from openpilot.frogpilot.system.speed_bump_detector import HP_WINDOW, PEAK_WINDOW, REFINE_WINDOW, REFRACTORY, SpeedBumpDetector, find_bump_near

RATE = 104.0
DATA = Path(__file__).parent / "data" / "speed_bump_imu.npz"


def bump_pulse(t, at, amplitude):
  """Nose up then down over ~0.4 s, like the pitch rate over a bump."""
  x = (t - at) / 0.1
  return amplitude * -x * math.exp(-x * x / 2) / 0.6065  # peak |value| = amplitude


def run(signal, duration, speed=8.0, detector=None, accel=None):
  det = detector or SpeedBumpDetector()
  det.set_speed(speed)
  events = []
  for i in range(int(duration * RATE)):
    t = 100.0 + i / RATE
    det.add_accel(t, 9.81 + (accel(t) if accel else 0.0))
    e = det.add_gyro(t, signal(t))
    if e is not None:
      events.append(e)
  return events


def test_single_bump_detected_at_its_peak():
  events = run(lambda t: 0.002 * math.sin(7 * t) + bump_pulse(t, 105.0, 0.3), 10,
               accel=lambda t: bump_pulse(t, 105.0, 3.0))
  assert len(events) == 1
  e = events[0]
  assert abs(e.t - 105.0) < 0.15, "event time is the peak's own sample time, not the filter-delayed one"
  assert 0.2 < e.pitch < 0.35
  assert e.az_pp > 3.0
  assert e.v_ego == 8.0


def test_below_threshold_or_standing_still_is_ignored():
  assert run(lambda t: bump_pulse(t, 105.0, 0.12), 10) == []
  assert run(lambda t: bump_pulse(t, 105.0, 0.3), 10, speed=1.0) == []
  # the threshold is live-adjustable, as the process does from the param
  det = SpeedBumpDetector(threshold=0.1)
  assert len(run(lambda t: bump_pulse(t, 105.0, 0.12), 10, detector=det)) == 1


def test_slow_pitch_changes_are_filtered_out():
  # a hill crest, a braking dive: large but slow; plus a constant gyro bias
  slow = lambda t: 0.05 + 0.4 * math.sin(2 * math.pi * 0.15 * t)  # noqa: E731
  assert run(slow, 30) == []


def test_refractory_merges_a_double_hit_but_not_two_bumps():
  # front and rear axle / body bounce 0.6 s apart: one bump
  events = run(lambda t: bump_pulse(t, 105.0, 0.3) + bump_pulse(t, 105.6, 0.25), 10)
  assert len(events) == 1
  # two bumps further apart than the refractory period: two events
  gap = REFRACTORY + PEAK_WINDOW
  events = run(lambda t: bump_pulse(t, 105.0, 0.3) + bump_pulse(t, 105.0 + gap, 0.3), 15)
  assert len(events) == 2


def test_az_floor_suppresses_pitch_without_a_jolt():
  det = SpeedBumpDetector(az_pp_floor=2.0)
  assert run(lambda t: bump_pulse(t, 105.0, 0.3), 10, detector=det) == []


def test_warm_up_needs_a_full_window():
  det = SpeedBumpDetector()
  det.set_speed(8)
  assert all(det.add_gyro(100 + i / RATE, 1.0) is None for i in range(HP_WINDOW - 1))


def _replay(prefix, threshold):
  d = np.load(DATA)
  acc, gyr, cs = d[f"{prefix}_acc"], d[f"{prefix}_gyr"], d[f"{prefix}_cs"]
  stream = sorted([(t, 0, x) for t, x in acc] + [(t, 1, y) for t, y in gyr])
  det = SpeedBumpDetector(threshold=threshold)
  ci, events = 0, []
  for t, kind, x in stream:
    while ci + 1 < len(cs) and cs[ci + 1, 0] <= t:
      ci += 1
    det.set_speed(float(cs[ci, 1]))
    if kind == 0:
      det.add_accel(float(t), float(x))
    elif (e := det.add_gyro(float(t), float(x))) is not None:
      events.append(e)
  return events, d


@pytest.mark.parametrize("threshold", [0.15])
def test_recorded_mapped_bump_is_detected(threshold):
  # A low hump on Södra Förstadsgatan at 16 km/h, from the 2026-09-29 drives:
  # one of the weakest mapped bumps that still crosses the default threshold.
  events, d = _replay("bump", threshold)
  assert len(events) == 1
  assert abs(events[0].t - float(d["bump_event_t"])) < 0.02
  assert abs(events[0].pitch - float(d["bump_pitch"])) < 1e-3


def test_recorded_mapped_bump_missed_when_less_sensitive():
  events, _ = _replay("bump", 0.17)
  assert events == []


def test_recorded_background_has_no_detection():
  # 30 s of moving city driving, well away from any bump or detection
  events, d = _replay("quiet", 0.15)
  assert events == []
  assert d["quiet_cs"][:, 1].min() > 4


# --- Refining a driver's mark from the IMU ---



def samples(signal, start=100.0, duration=40.0, speed=8.0):
  return [(start + i / RATE, signal(start + i / RATE), speed) for i in range(int(duration * RATE))]


@pytest.mark.parametrize("offset", [-8.0, -3.0, 0.0, 4.0, 9.0])
def test_refine_finds_the_bump_either_side_of_the_tap(offset):
  bump_t = 120.0
  s = samples(lambda t: bump_pulse(t, bump_t, 0.2))
  found = find_bump_near(s, tap_t=bump_t - offset)
  assert found is not None
  assert abs(found[0] - bump_t) < 0.15


def test_refine_prefers_the_strongest_jolt_not_the_nearest():
  s = samples(lambda t: bump_pulse(t, 115.0, 0.12) + bump_pulse(t, 124.0, 0.3))
  found = find_bump_near(s, tap_t=116.0)
  assert abs(found[0] - 124.0) < 0.15


def test_refine_ignores_jolts_outside_the_window_and_while_stopped():
  s = samples(lambda t: bump_pulse(t, 120.0, 0.3))
  assert find_bump_near(s, tap_t=120.0 + REFINE_WINDOW + 2.0) is None
  assert find_bump_near(samples(lambda t: bump_pulse(t, 120.0, 0.3), speed=0.5), tap_t=120.0) is None


def test_refine_finds_nothing_on_a_quiet_road():
  assert find_bump_near(samples(lambda t: 0.01 * math.sin(t * 7)), tap_t=120.0) is None


def test_refine_on_recorded_bump_tapped_late():
  # The recorded Södra Förstadsgatan hump, with the button pressed 6 s after crossing it
  d = np.load(DATA)
  gyr, cs = d["bump_gyr"], d["bump_cs"]
  s = [(float(t), float(y), float(np.interp(t, cs[:, 0], cs[:, 1]))) for t, y in gyr]
  bump_t = float(d["bump_event_t"])
  found = find_bump_near(s, tap_t=bump_t + 6.0)
  assert found is not None and abs(found[0] - bump_t) < 0.5
