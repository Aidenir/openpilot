from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib.refuge_island_controller import A_V, B_V, HOLD_AFTER, HOLD_BEFORE, MATCH_DISTANCE, \
                                                                      RefugeIslandController, SPEED_BP

ISLAND = 150.0  # m ahead at the start
LAG = 0.3       # s from the curvature asked for to the car following it


def model(right_edge=3.3, edge_std=0.3):
  """The parts of modelV2 the controller reads: Geijersgatan's right kerb was ~3.3 m right of the car."""
  return SimpleNamespace(roadEdges=[SimpleNamespace(y=[-5.4]), SimpleNamespace(y=[right_edge])], roadEdgeStds=[edge_std, edge_std])


def drive(v=10.5, islands=(ISLAND,), distance=320.0, model_gain=1.0, right_edge=3.3, setting=0.4, first_seen=None, noise=0.0,
          push_left_at=None, seed=0):
  """Drives past "islands" (m from the start) at "v", with mapd reporting the next one ahead (from "first_seen" m out, if set,
  e.g. just after turning onto the road) and a driving model that steers back to its own line the way the logged one does,
  "model_gain" times as hard. Returns per step: s, e (m right of the model's line), the added curvature and the offset."""
  rng = np.random.default_rng(seed)
  ric = RefugeIslandController()
  a = float(np.interp(v, SPEED_BP, A_V)) * model_gain
  b = float(np.interp(v, SPEED_BP, B_V)) * model_gain
  lag = deque([0.0] * round(LAG / DT_MDL))
  s = e = de = 0.0
  out = []
  while s < distance:
    ahead = [i - s for i in islands if i - s >= 0 and (first_seen is None or i - s <= first_seen)]
    found = bool(ahead)
    reported = min(ahead) + rng.normal(0, noise) if found else 0.0
    pushing = push_left_at is not None and push_left_at <= s < push_left_at + 10
    model_curvature = -a * e - b * de
    ric.update(True, True, v, found, max(reported, 0.0), model(right_edge), pushing, 100.0 if pushing else 0.0, False, False,
               model_curvature, setting)
    lag.append(ric.curvature)
    ds = v * DT_MDL
    de += (model_curvature + lag.popleft()) * ds
    e += de * ds
    s += ds
    out.append((s, e, ric.curvature, ric.offset))
  return np.array(out)


def at(run, s):
  return run[np.searchsorted(run[:, 0], s)]


class TestRefugeIslandController:
  def test_moves_over_for_the_island_and_back(self):
    run = drive()
    # over by the setting along the island, give or take 5 cm
    for s in np.arange(ISLAND - HOLD_BEFORE, ISLAND + HOLD_AFTER, 1.0):
      assert at(run, s)[1] == pytest.approx(0.4, abs=0.05), s
    # nothing well before it, and back on the model's line well after
    assert abs(at(run, ISLAND - 60)[1]) < 0.01
    assert abs(at(run, ISLAND + 60)[1]) < 0.03
    assert np.all(np.abs(run[:, 1]) < 0.45)

  def test_gentle(self):
    run = drive()
    curvature = run[:, 2]
    assert np.max(np.abs(curvature)) * 10.5 ** 2 < 0.5  # m/s^2 of lateral acceleration
    assert np.max(np.abs(np.diff(curvature))) / DT_MDL * 10.5 ** 2 < 1.0  # m/s^3 of lateral jerk

  @pytest.mark.parametrize("gain", [0.7, 1.3])
  def test_a_model_that_steers_back_harder_or_softer(self, gain):
    # The fit is only a fit: a model 30% off still takes the car over by about the setting, and back
    run = drive(model_gain=gain)
    assert 0.25 < at(run, ISLAND)[1] < 0.6
    assert abs(at(run, ISLAND + 70)[1]) < 0.06

  @pytest.mark.parametrize("v", [5.0, 8.0, 14.0, 16.5])
  def test_other_speeds(self, v):
    run = drive(v=v)
    assert at(run, ISLAND)[1] == pytest.approx(0.4, abs=0.1)
    assert np.max(np.abs(run[:, 2])) * v ** 2 <= 1.0 + 1e-6

  def test_too_fast_or_slow(self):
    for v in (2.0, 20.0):
      assert np.all(drive(v=v)[:, 2] == 0.0)

  def test_no_room_on_the_right(self):
    # the kerb 1.55 m right of the car: there is nowhere to go
    assert np.all(drive(right_edge=1.55)[:, 2] == 0.0)
    # 1.75 m: a quarter of a metre
    assert at(drive(right_edge=1.75), ISLAND)[3] == pytest.approx(0.25)

  def test_islands_close_together_are_passed_in_one_go(self):
    # Geijersgatan has two 30 m apart. mapd only reports the second once the first is passed
    run = drive(islands=(ISLAND, ISLAND + 30), distance=400)
    for s in np.arange(ISLAND, ISLAND + 30, 1.0):
      assert at(run, s)[1] > 0.35, s
    assert abs(at(run, ISLAND + 100)[1]) < 0.03

  def test_islands_far_apart_are_passed_one_by_one(self):
    run = drive(islands=(ISLAND, ISLAND + 120), distance=420)
    assert at(run, ISLAND)[1] == pytest.approx(0.4, abs=0.05)
    assert at(run, ISLAND + 120)[1] == pytest.approx(0.4, abs=0.05)
    assert abs(at(run, ISLAND + 60)[1]) < 0.05

  def test_gps_noise(self):
    clean, noisy = drive(), drive(noise=3.0, seed=1)
    assert at(noisy, ISLAND)[1] == pytest.approx(0.4, abs=0.08)
    assert np.max(np.abs(noisy[:, 2])) < 2 * np.max(np.abs(clean[:, 2]))

  def test_seen_late(self):
    # Just turned onto the road 30 m before it: a shorter easing over, no jump
    run = drive(first_seen=30.0)
    assert at(run, ISLAND)[1] > 0.3
    assert np.max(np.abs(np.diff(run[:, 3]))) < 0.05
    # 12 m before it: too late, stay put
    assert np.all(drive(first_seen=12.0)[:, 2] == 0.0)

  def test_driver_pushing_left_lets_go_of_the_island(self):
    run = drive(push_left_at=ISLAND - 20)
    held = at(run, ISLAND - 20)[3]
    assert held > 0.2
    # eased out from there, not dropped, and not picked up again while mapd still reports it
    assert np.max(np.abs(np.diff(run[:, 3]))) < 0.05
    assert at(run, ISLAND)[3] < held
    assert np.all(run[run[:, 0] > ISLAND + 20, 3] == 0.0)

  def test_driver_pulling_right_is_fine(self):
    ric = RefugeIslandController()
    for _ in range(int(5 / DT_MDL)):
      ric.update(True, True, 10.0, True, 40.0, model(), True, -100.0, False, False, 0.0, 0.4)
    assert len(ric.islands) == 1

  def test_signalling_or_turning_lets_go(self):
    for blinker, lane_changing, road_curvature in [(True, False, 0.0), (False, True, 0.0), (False, False, 0.03)]:
      ric = RefugeIslandController()
      ric.update(True, True, 10.0, True, 60.0, model(), False, 0.0, False, False, 0.0, 0.4)
      assert len(ric.islands) == 1
      ric.update(True, True, 10.0, True, 60.0, model(), False, 0.0, blinker, lane_changing, road_curvature, 0.4)
      assert ric.islands == []

  def test_off(self):
    ric = RefugeIslandController()
    for enabled, lat_active in [(False, True), (True, False)]:
      ric.update(enabled, lat_active, 10.0, True, 20.0, model(), False, 0.0, False, False, 0.0, 0.4)
      assert (ric.curvature, ric.offset, ric.islands) == (0.0, 0.0, [])

  def test_readings_of_one_island_stay_one_island(self):
    ric = RefugeIslandController()
    for k in range(40):
      jitter = MATCH_DISTANCE * 0.4 * (-1) ** k
      ric.update(True, True, 10.0, True, 120.0 - 10.0 * DT_MDL * k + jitter, model(), False, 0.0, False, False, 0.0, 0.4)
    assert len(ric.islands) == 1
