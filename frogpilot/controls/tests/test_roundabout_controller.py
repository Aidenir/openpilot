import math

import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib.roundabout_controller import ENTRY_OFFSET, MAX_DIAMETER, MINI_SPEED, RING_EXTRA, \
                                                                   ROUNDABOUT_STALE_TIMEOUT, RingFilter, \
                                                                   roundabout_config, roundabout_speed
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


def ring_pass(reports, v=5.0):
  """Feeds "reports" (mapd's hasNextRoundabout, distance, diameter per step) to a RingFilter at "v" m/s; returns what it lets through."""
  f = RingFilter()
  return [f.update(has, d, D, v) for has, d, D in reports]


def entering(diameter, start=40.0, v=5.0):
  # mapd counting down to the ring's centreline, the car moving v * DT_MDL a step
  d, out = start, []
  while d >= 0.1:
    out.append((True, d, diameter))
    d -= v * DT_MDL
  return out


class TestRingFilter:
  def test_ring_reported_from_inside_it_is_ignored(self):
    # Drive 27d, Limhamnsvägen: the 22 m ring reported as 74.7 m ahead the moment the car was in it, until mapd dropped it at 44 m
    inside = [(True, 74.7 - 5.0 * DT_MDL * i, 22.0) for i in range(120)]
    kept = ring_pass(entering(22.0) + inside + [(False, 0.0, 0.0)] * 20)
    n = len(entering(22.0))
    assert all(kept[:n])
    assert not any(kept[n:])

  def test_next_roundabout_along_the_road_counts(self):
    # A different ring (another size) 80 m on is slowed for
    kept = ring_pass(entering(22.0) + [(False, 0.0, 0.0)] * 5 + [(True, 80.0 - 5.0 * DT_MDL * i, 31.0) for i in range(50)])
    assert all(kept[-50:])

  def test_same_ring_counts_again_once_driven_past(self):
    f = RingFilter()
    for has, d, D in entering(22.0):
      f.update(has, d, D, 5.0)
    f.update(False, 0.0, 0.0, 5.0)
    # drive the ring's circumference and RING_EXTRA: a 22 m ring further on is a different one
    for _ in range(int((math.pi * 22.0 + RING_EXTRA) / (5.0 * DT_MDL)) + 2):
      f.update(False, 0.0, 0.0, 5.0)
    assert f.update(True, 60.0, 22.0, 5.0)

  def test_mini_roundabouts_in_a_row_all_count(self):
    kept = ring_pass(entering(0.0) + [(False, 0.0, 0.0)] * 5 + entering(0.0, start=25.0))
    assert all(k for k, (has, _, _) in zip(kept, entering(0.0) + [(False, 0.0, 0.0)] * 5 + entering(0.0, start=25.0), strict=True) if has)


class TestVanishedRoundabout:
  def run(self, controller, vanish_at):
    # Braking at 10 m/s for a roundabout 60 m ahead, which mapd stops reporting once it is "vanish_at" m away
    v, x, decels = 10.0, 0.0, []
    for _ in range(int(12 / DT_MDL)):
      remaining = 60.0 - x
      has = remaining > vanish_at
      _, decel = controller.update(True, has, remaining if has else 0.0, v, roundabout_config(CFG, roundabout_speed(22.0)))
      decels.append((remaining, decel))
      v = max(0.0, v - decel * DT_MDL)
      x += v * DT_MDL
    return decels

  # Braking starts ~33 m out here; the roundabout vanishes 25 m out, while it is being braked for
  def test_roundabout_gone_far_out_is_let_go(self):
    decels = self.run(SpeedBumpController(hold_vanished=False, stale_timeout=ROUNDABOUT_STALE_TIMEOUT), vanish_at=25.0)
    assert any(d > 0 for r, d in decels if r > 25.0)
    assert all(d == 0 for r, d in decels if r < 25.0 - 10.0)  # the stale timeout and the brakes easing off

  def test_bump_gone_while_braking_is_still_held(self):
    decels = self.run(SpeedBumpController(), vanish_at=25.0)
    assert any(d > 0 for r, d in decels if r < 25.0 - 15.0)
