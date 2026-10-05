import numpy as np
import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib.red_light_controller import BRAKE_TIME, END_OFFSET, LOST_KEEP_DISTANCE, LOST_TIME, MARGIN, \
                                                                   MAX_DECEL, RELEASE_TIME, SIGNAL_OFFSET, SLOWING_SCALE, \
                                                                   STOP_SPEED, RedLightController, model_stop_distance
from openpilot.frogpilot.controls.lib.speed_bump_controller import SpeedBumpConfig
from openpilot.selfdrive.modeld.constants import ModelConstants

CFG = SpeedBumpConfig()
T = np.array(ModelConstants.T_IDXS)
LINE = 65.0   # m, the stop line
CREEP = 2.0   # m, a plan that comes to rest ends about this far past where drivers stop (drives 269-275)


def stopping_plan(v, distance):
  """The model's plan from "v": braking evenly to a stop "distance" metres ahead."""
  if distance <= 0.1 or v <= 0.1:
    return np.zeros_like(T), np.zeros_like(T)
  decel = v**2 / (2.0 * distance)
  t = np.minimum(T, v / decel)
  return v * t - decel * t**2 / 2.0, np.maximum(v - decel * T, 0.0)


def cruising_plan(v):
  return v * T, np.full_like(T, v)


def creeping_plan(x, v):
  """Comes to rest CREEP past the line, as the model's did in Malmö."""
  return stopping_plan(v, LINE - x + CREEP)


def slowing_plan(v_end):
  """Slows evenly to "v_end" and holds it: the plan for a bump, a turn or a crossing."""
  def plan(x, v):
    velocity = np.maximum(v - 1.0 * T, v_end)
    return np.concatenate([[0.0], np.cumsum((velocity[1:] + velocity[:-1]) / 2 * np.diff(T))]), velocity
  return plan


def approach(kph=40, seconds=30.0, plan=creeping_plan, wanted=None, active=True, slowdowns=()):
  """Drives at "kph" towards a red light at LINE that the model plans to stop for, as it did in Malmö. Same crude car
  as the speed bump tests: cruise P-control at up to 1.2 m/s^2 or the decel request, with a 0.3 s lag. "plan(x, v)" and
  "wanted(x, t)" change the model's plan and whether the light is being stopped for."""
  wanted = wanted or (lambda x, t: True)
  rlc = RedLightController()
  v_set = kph * CV.KPH_TO_MS
  x, v, a = 0.0, v_set, 0.0
  log = []
  for step in range(int(seconds / DT_MDL)):
    t = step * DT_MDL
    position, velocity = plan(x, v)
    cap, decel = rlc.update(active, wanted(x, t), position, velocity, v, CFG, slowdowns=[d - x for d in slowdowns])
    v_cruise = v_set if cap is None else min(v_set, cap)
    a_target = max(-1.2, min(1.2, (v_cruise - v) / 0.5))
    if decel > 0:
      a_target = min(a_target, -decel)
    a += (a_target - a) * DT_MDL / 0.3
    v = max(0.0, v + a * DT_MDL)
    x += v * DT_MDL
    log.append((x, v, cap, decel, a))
  return rlc, log


class TestModelStopDistance:
  def test_a_plan_that_comes_to_rest(self):
    position, velocity = stopping_plan(8.0, 30.0)
    assert velocity[-1] == 0.0
    assert model_stop_distance(position, velocity) == pytest.approx(30.0 - END_OFFSET)

  def test_a_plan_that_slows_to_a_crawl(self):
    position, velocity = slowing_plan(1.5)(0.0, 10.0)
    k = int(np.argmax(velocity < STOP_SPEED))
    assert model_stop_distance(position, velocity) == pytest.approx(position[k] * SLOWING_SCALE)

  def test_none_for_a_plan_that_only_slows(self):
    position, velocity = slowing_plan(4.0)(0.0, 10.0)
    assert model_stop_distance(position, velocity) is None


class TestRedLightStop:
  @pytest.mark.parametrize("kph", [30, 40, 50])
  def test_stops_at_the_line(self, kph):
    _, log = approach(kph)
    x, v = log[-1][:2]
    assert v == 0.0
    assert LINE - MARGIN - 1.5 < x < LINE

  @pytest.mark.parametrize("kph", [30, 40, 50])
  def test_brakes_from_brake_time_out_and_gently(self, kph):
    _, log = approach(kph)
    v0 = kph * CV.KPH_TO_MS
    onset = next(x for x, _, _, decel, _ in log if decel > 0)
    assert LINE - onset == pytest.approx(v0 * BRAKE_TIME, rel=0.2)
    assert max(decel for *_, decel, _ in log) < 2.5
    jerks = np.diff([a for *_, a in log]) / DT_MDL
    assert np.max(np.abs(jerks)) < 2.5

  def test_never_more_than_max_decel(self):
    # The light first seen 20 m out at 50 km/h
    _, log = approach(50, plan=lambda x, v: creeping_plan(x, v) if x > LINE - 20.0 else cruising_plan(v))
    assert max(decel for *_, decel, _ in log) <= MAX_DECEL + 1e-6

  def test_holds_the_stop_when_the_model_loses_the_light(self):
    # Close in the model plans to drive on, but the light is still red: it stops where the stop point had got to by then
    _, log = approach(40, plan=lambda x, v: creeping_plan(x, v) if x < LINE - 20.0 else cruising_plan(10.0))
    x, v = log[-1][:2]
    assert v == 0.0
    assert x < LINE + 3.0

  def test_a_few_short_plans_do_not_stop_the_car_early(self):
    def plan(x, v):
      return stopping_plan(v, 15.0) if 10.0 <= x < 12.0 else creeping_plan(x, v)
    _, log = approach(40, plan=plan)
    _, undisturbed = approach(40)
    assert log[-1][0] == pytest.approx(undisturbed[-1][0], abs=2.0)

  def test_lets_go_when_the_light_turns_green(self):
    green = 4.0  # s
    rlc, log = approach(40, seconds=12.0, wanted=lambda x, t: t < green)
    released = int((green + RELEASE_TIME) / DT_MDL) + 2
    assert any(decel > 0 for *_, decel, _ in log[:released])
    assert all(decel == 0 for *_, decel, _ in log[released:])
    assert rlc.stop_distance is None and not rlc.stopping
    # and drives on rather than stopping at the light
    assert log[-1][1] > 8.0

  def test_ignores_a_plan_that_only_slows(self):
    # Slowing for a speed bump or a turn with the stop light condition up: the plan never comes to a stop
    _, log = approach(40, plan=lambda x, v: cruising_plan(min(v, 4.0)))
    assert all(cap is None and decel == 0 for _, _, cap, decel, _ in log)

  def test_holds_at_a_standstill(self):
    rlc, log = approach(40, seconds=40.0)
    assert all(v == 0.0 for _, v, *_ in log[-100:])
    assert log[-1][2] == 0.0
    assert rlc.stopping

  def test_nothing_while_not_engaged(self):
    rlc, log = approach(40, seconds=5.0, active=False)
    assert all(cap is None and decel == 0 for _, _, cap, decel, _ in log)
    assert rlc.stop_distance is not None


class TestFalseStops:
  def test_one_stray_plan_does_not_brake_early(self):
    # Drive 275, Ystadvägen x Nobelvägen: one plan stopping 13 m short of the rest set the stop point and braked from 75 m out
    def plan(x, v):
      return stopping_plan(v, LINE - x - 15.0) if 0.0 <= x < 0.2 else creeping_plan(x, v)
    _, log = approach(40, plan=plan)
    _, undisturbed = approach(40)
    assert log[-1][0] == pytest.approx(undisturbed[-1][0], abs=1.5)
    def early(lg):
      return max(decel for x, _, _, decel, _ in lg if x < 25.0)
    assert early(log) < early(undisturbed) + 0.3

  def test_lets_go_when_the_plan_stops_stopping(self):
    # Drive 275, Lorensborgsgatan: the stop light condition stayed up while the plan only slowed, for a bump and a roundabout
    def plan(x, v):
      return creeping_plan(x, v) if x < 5.0 else slowing_plan(5.0)(x, v)
    rlc, log = approach(40, seconds=12.0, plan=plan)
    gone = next(i for i, (x, *_) in enumerate(log) if x >= 5.0) + int((LOST_TIME + RELEASE_TIME) / DT_MDL) + 2
    assert rlc.stop_distance is None
    assert all(decel == 0.0 for *_, decel, _ in log[gone:])
    assert log[-1][1] > 8.0

  def test_kept_close_in(self):
    # Inside LOST_KEEP_DISTANCE the plan losing the stop is the model losing the light
    def plan(x, v):
      return creeping_plan(x, v) if x < LINE - LOST_KEEP_DISTANCE + 5.0 else slowing_plan(5.0)(x, v)
    _, log = approach(40, plan=plan)
    assert log[-1][1] == 0.0

  def test_a_stop_at_a_mapped_bump_is_left_to_the_bump(self):
    # The model slows to a stop at a bump (or roundabout) on the map: that is not a light
    bump = LINE + CREEP - 3.0
    _, log = approach(40, plan=lambda x, v: creeping_plan(x, v) if x < bump else cruising_plan(v), slowdowns=[bump])
    assert all(cap is None and decel == 0.0 for _, _, cap, decel, _ in log)


class TestMappedSignals:
  def run(self, plan, signal_at):
    rlc = RedLightController()
    v, x, a, log = 40 * CV.KPH_TO_MS, 0.0, 0.0, []
    for _ in range(int(30 / DT_MDL)):
      position, velocity = plan(x, v)
      signal = None if signal_at is None or signal_at - x < 0 else signal_at - x
      cap, decel = rlc.update(True, True, position, velocity, v, CFG, signal=signal)
      v_cruise = 40 * CV.KPH_TO_MS if cap is None else min(40 * CV.KPH_TO_MS, cap)
      a_target = max(-1.2, min(1.2, (v_cruise - v) / 0.5))
      if decel > 0:
        a_target = min(a_target, -decel)
      a += (a_target - a) * DT_MDL / 0.3
      v = max(0.0, v + a * DT_MDL)
      x += v * DT_MDL
      log.append((x, v, rlc.stop_distance, decel))
    return log

  @staticmethod
  def noisy_plan(x, v):
    # Far out the model's stop point is 25% long, close in it is right: MARGIN past the line, where the map puts it too (CREEP past
    # that, like creeping_plan), so only the noise far out tells the two apart whatever MARGIN is
    remaining = LINE + MARGIN - x
    return stopping_plan(v, remaining * (1.25 if remaining > 30.0 else 1.0) + CREEP)

  def test_a_mapped_signal_places_the_stop_far_out(self):
    signal = LINE + MARGIN + SIGNAL_OFFSET  # where the map has the light, for a stop at LINE
    with_map, without = self.run(self.noisy_plan, signal), self.run(self.noisy_plan, None)
    def error_at(log, out):
      x, _, stop, _ = next(r for r in log if LINE - r[0] <= out)
      return stop - (LINE + MARGIN - x)
    assert abs(error_at(with_map, 45.0)) < 1.0
    assert abs(error_at(without, 45.0)) > 3.0
    # Close in the model's own stop point is used, map or no map
    assert with_map[-1][0] == pytest.approx(without[-1][0], abs=1.0)

  def test_a_signal_far_from_the_models_stop_is_ignored(self):
    # The model stops for something 40 m before the next mapped light: that light doesn't move the stop point
    assert self.run(creeping_plan, LINE + 40.0)[-1][0] == pytest.approx(self.run(creeping_plan, None)[-1][0], abs=0.1)

  def test_no_stop_from_the_map_alone(self):
    # A mapped signal with a plan that doesn't stop (the light is green) brakes for nothing
    log = self.run(lambda x, v: cruising_plan(v), LINE)
    assert all(stop is None and decel == 0 for _, _, stop, decel in log)


class TestTooLate:
  def test_a_light_found_too_late_is_let_go(self):
    # First stopped for ~10 m out at 50 km/h: braking can't make it, and capping the cruise speed at 0 slowed the car to a stop
    # in or past the junction, with no forced stop for the gas to override
    rlc = RedLightController()
    v = 50 * CV.KPH_TO_MS
    for step in range(int(3.0 / DT_MDL)):
      position, velocity = stopping_plan(v, max(0.0, 10.0 + END_OFFSET - v * step * DT_MDL))
      cap, decel = rlc.update(True, True, position, velocity, v, CFG)
      assert cap is None and decel == 0.0
      assert not rlc.stopping

  def test_the_next_light_is_stopped_for_again(self):
    rlc = RedLightController()
    v = 50 * CV.KPH_TO_MS
    position, velocity = stopping_plan(v, 10.0 + END_OFFSET)
    rlc.update(True, True, position, velocity, v, CFG)
    for _ in range(int(2 * RELEASE_TIME / DT_MDL)):
      rlc.update(True, False, *cruising_plan(v), v, CFG)
    v = 40 * CV.KPH_TO_MS
    caps = []
    for _ in range(int(2.0 / DT_MDL)):
      cap, _ = rlc.update(True, True, *stopping_plan(v, 60.0), v, CFG)
      caps.append(cap)
    assert rlc.stopping and caps[-1] is not None
