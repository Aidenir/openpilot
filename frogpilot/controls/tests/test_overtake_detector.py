from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib.overtake_detector import HOLD_TIME, RELEASE_TIME, OvertakeDetector, map_lanes_ahead

LIMIT = 80 * CV.KPH_TO_MS
SLOW = 60 * CV.KPH_TO_MS
LEAD = SimpleNamespace(status=True, dRel=40.0, vLead=SLOW)
NO_LEAD = SimpleNamespace(status=False, dRel=0.0, vLead=0.0)

# A one-way road with two lanes, from the map
ROAD = dict(mapd_alive=True, one_way=True, lanes=2, left_lane_prob=0.0, lane_width_left=0.0)


def drive(detector, seconds, enabled=True, v_ego=SLOW, speed_limit=LIMIT, lead=LEAD, signalling=False, **road):
  road = {**ROAD, **road}
  for _ in range(round(seconds / DT_MDL)):
    detector.update(enabled, v_ego, speed_limit, lead, road["mapd_alive"], road["one_way"], road["lanes"], road["left_lane_prob"],
                    road["lane_width_left"], signalling)
  return detector.suggested


class TestMapLanesAhead:
  @pytest.mark.parametrize("one_way, lanes, expected", [
    (True, 1, 1), (True, 2, 2), (True, 3, 3),
    (False, 2, 1), (False, 4, 2), (False, 6, 3),
    (False, 3, None),  # 2+1 road, the middle lane changes direction
    (True, 0, None), (False, 0, None),  # not tagged
  ])
  def test_lanes(self, one_way, lanes, expected):
    assert map_lanes_ahead(one_way, lanes) == expected


class TestOvertakeDetector:
  def test_suggests_after_hold_time(self):
    od = OvertakeDetector()
    assert not drive(od, HOLD_TIME - 0.5)
    assert drive(od, 1.0)
    assert od.lane_source == "2 lanes (map)"
    assert od.speed_limit == LIMIT

  def test_released_after_conditions_are_gone_for_a_while(self):
    od = OvertakeDetector()
    drive(od, HOLD_TIME + 0.5)
    assert drive(od, RELEASE_TIME - 0.5, lead=NO_LEAD)  # a lead dropping out for a moment
    assert not drive(od, 1.0, lead=NO_LEAD)
    assert (od.lane_source, od.speed_limit) == ("", 0.0)

  def test_a_short_gap_restarts_the_hold(self):
    od = OvertakeDetector()
    drive(od, HOLD_TIME - 0.5)
    drive(od, 0.2, lead=NO_LEAD)
    assert not drive(od, HOLD_TIME - 0.5)

  @pytest.mark.parametrize("change", [
    dict(enabled=False),
    dict(lead=NO_LEAD),
    dict(lead=SimpleNamespace(status=True, dRel=120.0, vLead=SLOW)),  # too far ahead to be holding us up
    dict(lead=SimpleNamespace(status=True, dRel=40.0, vLead=75 * CV.KPH_TO_MS)),  # pulling away at near the limit
    dict(v_ego=75 * CV.KPH_TO_MS),  # under the limit by less than 10 km/h
    dict(v_ego=20 * CV.KPH_TO_MS, lead=SimpleNamespace(status=True, dRel=15.0, vLead=20 * CV.KPH_TO_MS)),  # a queue
    dict(speed_limit=0.0),  # limit unknown
    dict(signalling=True),
    dict(lanes=1),
    dict(one_way=False, lanes=2),  # one lane each way: the left lane is the oncoming one
    dict(one_way=False, lanes=2, left_lane_prob=0.9, lane_width_left=3.5),  # and the camera seeing it doesn't count
  ])
  def test_not_suggested(self, change):
    assert not drive(OvertakeDetector(), HOLD_TIME + 1.0, **change)

  def test_two_lanes_each_way(self):
    assert drive(OvertakeDetector(), HOLD_TIME + 0.5, one_way=False, lanes=4)

  @pytest.mark.parametrize("road", [dict(lanes=0), dict(one_way=False, lanes=3), dict(mapd_alive=False)])
  def test_camera_decides_where_the_map_does_not(self, road):
    od = OvertakeDetector()
    assert not drive(od, HOLD_TIME + 0.5, **road)
    assert drive(od, HOLD_TIME + 0.5, left_lane_prob=0.8, lane_width_left=3.2, **road)
    assert od.lane_source == "left lane (camera)"
    assert not drive(OvertakeDetector(), HOLD_TIME + 0.5, left_lane_prob=0.8, lane_width_left=2.0, **road)  # too narrow

  def test_signalling_takes_it_down_at_once(self):
    od = OvertakeDetector()
    drive(od, HOLD_TIME + 0.5)
    assert not drive(od, DT_MDL, signalling=True)
