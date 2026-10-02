import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib.roundabout_controller import ENTRY_OFFSET, MAX_DIAMETER, MINI_SPEED, roundabout_config, \
                                                                   roundabout_speed
from openpilot.frogpilot.controls.lib.speed_bump_controller import SpeedBumpConfig, SpeedBumpController

CFG = SpeedBumpConfig()
RING = 150.0  # m, where the road meets the ring's centreline


class TestRoundaboutSpeed:
  @pytest.mark.parametrize("diameter, kph", [(18.4, 15.0), (57.6, 30.0)])
  def test_driver_examples(self, diameter, kph):
    # Ärtholmsvägen x Bellevuevägen and Agnesfridsvägen x Arrievägen
    assert roundabout_speed(diameter) * CV.MS_TO_KPH == pytest.approx(kph)

  def test_bigger_is_faster(self):
    speeds = [roundabout_speed(d) for d in range(10, int(MAX_DIAMETER) + 1, 5)]
    assert all(b >= a for a, b in zip(speeds, speeds[1:], strict=False))
    assert speeds[0] * CV.MS_TO_KPH == pytest.approx(12.0)  # held at the smallest

  def test_mini_roundabout(self):
    assert roundabout_speed(0.0) == MINI_SPEED

  def test_large_interchange_is_not_slowed_for(self):
    assert roundabout_speed(MAX_DIAMETER + 1) is None

  def test_config_keeps_the_speed_bump_settings_and_lets_go_at_the_entry(self):
    base = SpeedBumpConfig(brake_time=2.5, max_decel=2.0, margin=3.0, hold_distance=10.0)
    config = roundabout_config(base, 5.0)
    assert (config.v_target, config.hold_distance) == (5.0, 0.0)
    assert (config.brake_time, config.max_decel, config.margin) == (2.5, 2.0, 3.0)


def approach(diameter, kph, seconds=25.0):
  """Drives at "kph" towards a roundabout the way frogpilot_vcruise does, with mapd reporting the ring until 1 m before it.
  Same crude car as the speed bump tests: cruise P-control at up to 1.2 m/s^2 or the decel request, with a 0.3 s lag."""
  rbc = SpeedBumpController()
  v_set = kph * CV.KPH_TO_MS
  x, v, a = 0.0, v_set, 0.0
  log = []
  for _ in range(int(seconds / DT_MDL)):
    has_roundabout = RING - x >= 1.0
    entry_speed = roundabout_speed(diameter) if has_roundabout else None
    distance = RING - x - ENTRY_OFFSET if has_roundabout else 0.0
    cap, decel = rbc.update(True, entry_speed is not None, distance, v, roundabout_config(CFG, entry_speed or CFG.v_target))
    v_cruise = v_set if cap is None else min(v_set, cap)
    a_target = max(-1.2, min(1.2, (v_cruise - v) / 0.5))
    if decel > 0:
      a_target = min(a_target, -decel)
    a += (a_target - a) * DT_MDL / 0.3
    v = max(0.0, v + a * DT_MDL)
    x += v * DT_MDL
    log.append((x, v, cap, decel))
  return log


def speed_at(log, position):
  return next(v for x, v, _, _ in log if x >= position)


class TestRoundaboutApproach:
  @pytest.mark.parametrize("diameter", [18.4, 57.6, 0.0])
  @pytest.mark.parametrize("kph", [40, 50, 60])
  def test_at_entry_speed_by_the_give_way_line(self, diameter, kph):
    log = approach(diameter, kph)
    give_way = RING - ENTRY_OFFSET
    assert speed_at(log, give_way) <= roundabout_speed(diameter) + 0.5
    # and not much slower: it lands on the speed rather than dipping under
    assert min(v for x, v, _, _ in log if x <= give_way) >= roundabout_speed(diameter) - 1.0
    assert max(d for _, _, _, d in log) <= CFG.max_decel + 1e-6

  def test_lets_go_in_the_ring(self):
    log = approach(18.4, 50)
    # the cap is lifted at the give-way line and ramped away within a few seconds, then the car picks up speed again
    capped = [x for x, _, cap, _ in log if cap is not None]
    assert max(capped) < RING + 25.0
    assert log[-1][1] > roundabout_speed(18.4) + 3.0
    assert all(d == 0 for x, _, _, d in log if x > RING)

  def test_slower_already_is_left_alone(self):
    log = approach(57.6, 25)
    assert all(d == 0 for _, _, _, d in log)
    assert min(v for _, v, _, _ in log) == pytest.approx(25 * CV.KPH_TO_MS, abs=0.1)

  def test_large_interchange_does_nothing(self):
    log = approach(MAX_DIAMETER + 10, 50)
    assert all(cap is None and d == 0 for _, _, cap, d in log)
