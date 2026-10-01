import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib.speed_bump_controller import CONFIRM_SAMPLES, SpeedBumpConfig, SpeedBumpController, \
                                                                    brake_start_distance, braking_distance, feasible_start_distance, \
                                                                    max_jerk, required_decel

CFG = SpeedBumpConfig()
V_TARGET = CFG.v_target
BRAKE_TIME = CFG.brake_time
MAX_DECEL = CFG.max_decel
HOLD_DISTANCE = CFG.hold_distance
TARGET_MARGIN = CFG.margin
RESPONSE_LAG = CFG.response_lag


class TestSpeedBumpMath:
  def test_braking_distance(self):
    assert braking_distance(50 * CV.KPH_TO_MS, V_TARGET, 3.0) == pytest.approx(((50 / 3.6)**2 - (20 / 3.6)**2) / 6.0)
    assert braking_distance(V_TARGET, V_TARGET, 3.0) == 0.0
    assert braking_distance(3.0, V_TARGET, 3.0) == 0.0

  def test_brake_start_scales_with_speed(self):
    # Where the time point is reachable, braking starts exactly "brake time" seconds out at the current speed
    v = 30 * CV.KPH_TO_MS
    assert brake_start_distance(v, CFG) == pytest.approx(v * BRAKE_TIME)
    assert brake_start_distance(v, SpeedBumpConfig(brake_time=3.0)) == pytest.approx(v * 3.0)
    starts = [brake_start_distance(kph * CV.KPH_TO_MS, CFG) for kph in range(20, 90, 5)]
    assert all(b > a for a, b in zip(starts, starts[1:], strict=False))

  def test_brake_start_moves_out_when_max_decel_needs_it(self):
    v = 50 * CV.KPH_TO_MS
    start = brake_start_distance(v, CFG)
    assert start > v * BRAKE_TIME
    # From that point, the required deceleration fits within the max even after the response lag and the jerk-limited
    # ramps in and out
    assert start == pytest.approx(feasible_start_distance(v, CFG))
    jerk = max_jerk(v, CFG)
    peak = min(MAX_DECEL, (jerk * (v - V_TARGET)) ** 0.5)
    ramps = v * (RESPONSE_LAG + peak / jerk / 2) + V_TARGET * peak / jerk / 2
    assert start == pytest.approx(TARGET_MARGIN + ramps + braking_distance(v, V_TARGET, peak))
    assert required_decel(start, v, CFG) < MAX_DECEL
    # A gentler max moves it further out still
    assert brake_start_distance(v, SpeedBumpConfig(max_decel=2.0)) > start

  def test_required_decel(self):
    v = 40 * CV.KPH_TO_MS
    d = 30.0
    assert required_decel(d, v, CFG) == pytest.approx((v**2 - V_TARGET**2) / (2 * (d - TARGET_MARGIN - v * RESPONSE_LAG)))
    assert required_decel(d, V_TARGET, CFG) == 0.0
    assert required_decel(0.0, v, CFG) > 0  # finite at the bump


def drive(sbc, bump_positions, v_start, seconds, v_set=None, mapd=None, config=CFG):
  """Crude closed loop: the car follows min(cruise P-control at up to 1.2 m/s^2, -decel request) with a 0.3 s lag.
  "mapd(x)" returns (has_bump, distance)."""
  v_set = v_start if v_set is None else v_set
  x, v, a = 0.0, v_start, 0.0
  log = []
  for _ in range(int(seconds / DT_MDL)):
    if mapd is None:
      ahead = [b - x for b in bump_positions if b - x > 0]
      has_bump, d = (True, min(ahead)) if ahead else (False, 0.0)
    else:
      has_bump, d = mapd(x)
    cap, decel = sbc.update(True, has_bump, d, v, config)
    v_cruise = v_set if cap is None else min(v_set, cap)
    a_target = max(-1.2, min(1.2, (v_cruise - v) / 0.5))
    if decel > 0:
      a_target = min(a_target, -decel)
    a += (a_target - a) * DT_MDL / 0.3
    v = max(0.0, v + a * DT_MDL)
    x += v * DT_MDL
    log.append((x, v, cap, decel))
  return log


class TestSpeedBumpController:
  @pytest.mark.parametrize("kph", [30, 40, 50, 60])
  def test_reaches_target_at_bump_and_holds_past_it(self, kph):
    bump = 200.0
    # Long enough after the bump for the raised cap to climb back past the set speed and be dropped
    log = drive(SpeedBumpController(), [bump], kph * CV.KPH_TO_MS, 40)

    for x, v, cap, _ in log:
      if bump - TARGET_MARGIN <= x <= bump + HOLD_DISTANCE - 1:
        assert v <= V_TARGET + 2 * CV.KPH_TO_MS, (x, v)
        assert cap == pytest.approx(V_TARGET, abs=1 * CV.KPH_TO_MS)
    assert log[-1][2] is None  # released afterwards, once the raised cap no longer holds the car back

  def test_braking_starts_at_time_point(self):
    v0 = 30 * CV.KPH_TO_MS
    bump = 200.0
    log = drive(SpeedBumpController(), [bump], v0, 30)
    first = next(x for x, _, cap, _ in log if cap is not None)
    assert bump - first == pytest.approx(v0 * BRAKE_TIME, abs=v0 * DT_MDL * 2)

  def test_decel_is_bounded_and_jerk_limited(self):
    log = drive(SpeedBumpController(), [200.0], 60 * CV.KPH_TO_MS, 30, config=SpeedBumpConfig(max_decel=2.5))
    decels = [decel for *_, decel in log]
    assert max(decels) <= 2.5 + 1e-9
    for (_, v, _, a), (_, _, _, b) in zip(log, log[1:], strict=False):
      assert b - a <= max_jerk(v, CFG) * DT_MDL + 1e-6
      assert a - b <= 2 * max_jerk(v, CFG) * DT_MDL + 1e-6

  def test_never_below_target(self):
    log = drive(SpeedBumpController(), [150.0, 170.0, 300.0], 50 * CV.KPH_TO_MS, 40)
    assert all(cap >= V_TARGET - 1e-6 for _, _, cap, _ in log if cap is not None)
    assert min(v for _, v, _, _ in log) >= V_TARGET - 1.5 * CV.KPH_TO_MS

  def test_set_speed_below_target_is_kept(self):
    log = drive(SpeedBumpController(), [100.0], 3.0, 40, v_set=3.0)
    assert all(abs(v - 3.0) < 1e-6 and decel == 0.0 for _, v, _, decel in log)

  def test_single_glitch_sample_is_ignored(self):
    sbc = SpeedBumpController()
    v = 50 * CV.KPH_TO_MS
    assert sbc.update(True, True, 5.0, v, CFG) == (None, 0.0)
    for _ in range(20):
      assert sbc.update(True, False, 0.0, v, CFG) == (None, 0.0)

  def test_too_late_bump_does_not_slam(self):
    # Confirmed 8 m out at 50 km/h: can't be made, so no hard braking, only the gentle cruise-speed cap
    sbc = SpeedBumpController()
    v = 50 * CV.KPH_TO_MS
    results = [sbc.update(True, True, 8.0, v, CFG) for _ in range(CONFIRM_SAMPLES + 5)]
    assert all(decel == 0.0 for _, decel in results)
    assert results[-1][0] == pytest.approx(V_TARGET)

  def test_holds_when_mapd_moves_on_to_next_bump(self):
    # Once the car crosses the middle, mapd immediately reports a further bump; the first must still be held
    log = drive(SpeedBumpController(), [120.0, 135.0], 40 * CV.KPH_TO_MS, 20)
    for x, _, cap, _ in log:
      if 120.0 <= x <= 135.0:
        assert cap == pytest.approx(V_TARGET, abs=1 * CV.KPH_TO_MS), (x, cap)

  def test_holds_when_bump_vanishes(self):
    def mapd(x):
      return (True, 100.0 - x) if x < 100.0 else (False, 0.0)
    log = drive(SpeedBumpController(), [], 40 * CV.KPH_TO_MS, 20, mapd=mapd)
    held = [x for x, _, cap, _ in log if x >= 100.0 and cap is not None]
    assert held and max(held) >= 100.0 + HOLD_DISTANCE - 1.0
    assert log[-1][2] is None

  def test_clamped_zero_after_passing_is_not_readopted(self):
    # mapd clamps a passed bump's distance to 0 until its next re-evaluation; that must not start a second hold
    def mapd(x):
      if x < 100.0:
        return True, 100.0 - x
      if x < 130.0:
        return True, 0.0
      return False, 0.0
    log = drive(SpeedBumpController(), [], 40 * CV.KPH_TO_MS, 20, mapd=mapd)
    after = [(cap, decel) for x, _, cap, decel in log if x > 100.0 + HOLD_DISTANCE + 0.5]
    # Past the hold the cap is only ever raised (the car pulling away), never held at the bump speed again
    assert all(decel == 0.0 for _, decel in after)
    caps = [cap for cap, _ in after if cap is not None]
    assert all(b > a for a, b in zip(caps, caps[1:], strict=False))
    assert not caps or caps[0] > V_TARGET

  def test_new_bump_after_passed_one_is_adopted(self):
    log = drive(SpeedBumpController(), [100.0, 200.0], 40 * CV.KPH_TO_MS, 30)
    assert any(cap is not None for x, _, cap, _ in log if 150.0 < x < 200.0)

  def test_inactive_returns_nothing_but_keeps_tracking(self):
    sbc = SpeedBumpController()
    for _ in range(CONFIRM_SAMPLES):
      assert sbc.update(False, True, 15.0, 10.0, CFG) == (None, 0.0)
    cap, decel = sbc.update(True, True, 14.5, 10.0, CFG)
    # Braking starts at once, with the cap at the car's speed rather than a step down to the bump speed
    assert cap == pytest.approx(10.0)
    assert decel > 0


def first_braking_distance(log, bump):
  return bump - next(x for x, _, cap, _ in log if cap is not None)


def speed_at(log, x_at):
  return next(v for x, v, _, _ in log if x >= x_at)


class TestSpeedBumpModes:
  def test_strict_vs_default_at_50(self):
    bump = 200.0
    v0 = 50 * CV.KPH_TO_MS
    default = drive(SpeedBumpController(), [bump], v0, 30)
    strict = drive(SpeedBumpController(), [bump], v0, 30, config=SpeedBumpConfig(strict=True))

    # Default moves the braking point out so the bump speed is still reached
    assert first_braking_distance(default, bump) == pytest.approx(feasible_start_distance(v0, CFG), abs=v0 * DT_MDL * 2)
    assert first_braking_distance(default, bump) > v0 * BRAKE_TIME + 5
    assert speed_at(default, bump - TARGET_MARGIN) <= V_TARGET + 2 * CV.KPH_TO_MS

    # Strict starts exactly at the braking point, brakes at the max, and arrives faster
    assert first_braking_distance(strict, bump) == pytest.approx(v0 * BRAKE_TIME, abs=v0 * DT_MDL * 2)
    assert max(decel for *_, decel in strict) == pytest.approx(MAX_DECEL)
    assert speed_at(strict, bump - TARGET_MARGIN) > V_TARGET + 5 * CV.KPH_TO_MS
    assert speed_at(strict, bump - TARGET_MARGIN) < v0 - 10 * CV.KPH_TO_MS

  def test_strict_same_as_default_when_reachable(self):
    v0 = 30 * CV.KPH_TO_MS
    assert brake_start_distance(v0, SpeedBumpConfig(strict=True)) == pytest.approx(brake_start_distance(v0, CFG))

  @pytest.mark.parametrize("margin", [0.0, 2.0, 6.0])
  def test_margin(self, margin):
    bump = 200.0
    log = drive(SpeedBumpController(), [bump], 40 * CV.KPH_TO_MS, 30, config=SpeedBumpConfig(margin=margin))
    assert speed_at(log, bump - margin) <= V_TARGET + 2 * CV.KPH_TO_MS

  @pytest.mark.parametrize("hold", [0.0, 6.0, 15.0])
  def test_hold_distance(self, hold):
    bump = 200.0
    log = drive(SpeedBumpController(), [bump], 40 * CV.KPH_TO_MS, 30, config=SpeedBumpConfig(hold_distance=hold))
    held = [x for x, _, cap, _ in log if x >= bump and cap is not None and cap <= V_TARGET + 0.5 * CV.KPH_TO_MS]
    last_held = max(held) if held else bump
    assert last_held == pytest.approx(bump + hold, abs=1.0)

  @pytest.mark.parametrize("scale", [0.5, 1.0, 2.0])
  def test_jerk_scale(self, scale):
    config = SpeedBumpConfig(jerk_scale=scale)
    log = drive(SpeedBumpController(), [200.0], 50 * CV.KPH_TO_MS, 30, config=config)
    for (_, v, _, a), (_, _, _, b) in zip(log, log[1:], strict=False):
      assert b - a <= max_jerk(v, config) * DT_MDL + 1e-6
    # A softer ramp needs an earlier braking point to still make it
    assert feasible_start_distance(13.9, SpeedBumpConfig(jerk_scale=0.5)) > feasible_start_distance(13.9, config) or scale == 0.5

  def test_response_time_moves_braking_out(self):
    v0 = 50 * CV.KPH_TO_MS
    assert feasible_start_distance(v0, SpeedBumpConfig(response_lag=0.8)) == pytest.approx(
      feasible_start_distance(v0, SpeedBumpConfig(response_lag=0.3)) + v0 * 0.5)
    assert required_decel(30.0, v0, SpeedBumpConfig(response_lag=0.8)) > required_decel(30.0, v0, SpeedBumpConfig(response_lag=0.3))


SMOOTH_CFG = SpeedBumpConfig(v_target=14 * CV.KPH_TO_MS, brake_time=3.1, strict=True)  # the settings it felt jerky on


class TestSmoothness:
  def drive(self, v0=32 * CV.KPH_TO_MS, config=SMOOTH_CFG, seconds=20):
    # Ideal car that does whatever deceleration is requested, at least down to the cap
    sbc = SpeedBumpController()
    x, v, bump = 0.0, v0, 120.0
    out = []
    for _ in range(int(seconds / DT_MDL)):
      target, decel = sbc.update(True, True, bump - x, v, config)
      v = max(0.0, v - decel * DT_MDL)
      x += v * DT_MDL
      out.append((x, v, target, decel))
    return out

  def test_cap_follows_speed_instead_of_stepping(self):
    # Dropping the cap straight to the bump speed made the MPC brake hard on its own at the start of braking
    rows = self.drive()
    first = next(i for i, r in enumerate(rows) if r[2] is not None)
    assert rows[first][2] == pytest.approx(rows[first - 1][1], abs=0.05)
    for a, b in zip(rows[first:], rows[first + 1:], strict=False):
      if a[2] is not None and b[2] is not None and b[0] < 120.0:
        # Never above the speed the car was doing, never below the bump speed, only ever coming down
        assert 14 * CV.KPH_TO_MS - 1e-6 <= b[2] <= a[1] + 1e-6
        assert b[2] <= a[2] + 1e-9

  def test_decel_ramps_both_ways_at_the_jerk_limit(self):
    rows = self.drive()
    for a, b in zip(rows, rows[1:], strict=False):
      assert abs(b[3] - a[3]) <= max_jerk(a[1], SpeedBumpConfig(v_target=14 * CV.KPH_TO_MS)) * DT_MDL + 1e-6

  def test_cap_is_raised_gradually_after_the_bump(self):
    from openpilot.frogpilot.controls.lib.speed_bump_controller import RELEASE_RAMP
    rows = self.drive()
    past = [r for r in rows if r[0] > 120.0 + HOLD_DISTANCE + 1.0]
    raised = [r[2] for r in past if r[2] is not None]
    assert raised, "the cap should still be there, being raised, just after the hold"
    for a, b in zip(raised, raised[1:], strict=False):
      assert b - a == pytest.approx(RELEASE_RAMP * DT_MDL)
