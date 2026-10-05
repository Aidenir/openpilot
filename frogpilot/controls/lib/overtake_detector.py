#!/usr/bin/env python3
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

# First step towards automatic overtaking: only works out when an overtake would make sense and says so on screen ("Initiate
# overtake?", frogpilotPlan.overtakeSuggested), so the detection can be checked on the road before anything acts on it.
#
# It makes sense when following a lead well under the speed limit on a road with another lane the same way to pull out into.
# Traffic keeps right here, so that is a lane on the left. mapd's "lanes" is OpenStreetMap's, both directions together, so
# a two-way road only counts with an even number of them. An odd one is usually a 2+1 road ("mötesfri väg", the middle lane
# alternating direction), and the tiles don't keep "lanes:forward". Where the map doesn't tell, the camera has to: the
# model sees the left lane's far line and the lane is wide enough. On a two-way road that is the oncoming lane, so the
# camera is only asked when the map has nothing to say.

BELOW_LIMIT = 10 * CV.KPH_TO_MS  # m/s, at least this far under the speed limit
MIN_SPEED = 30 * CV.KPH_TO_MS    # m/s, slower is a queue rather than a slow car
MAX_LEAD_DISTANCE = 80.0         # m
MIN_LANE_PROB = 0.5              # modelV2.laneLineProbs of the left lane's far line
MIN_LANE_WIDTH = 2.5             # m, frogpilotPlan.laneWidthLeft

HOLD_TIME = 3.0     # s the conditions have to hold before the suggestion shows
RELEASE_TIME = 1.5  # s they have to be gone before it is taken down, so a lead dropping out for a moment doesn't flicker it


def map_lanes_ahead(one_way, lanes):
  """Lanes in the car's direction from mapd's "oneWay" and "lanes", or None if the map doesn't tell."""
  if lanes <= 0:
    return None
  if one_way:
    return lanes
  if lanes % 2 == 0:
    return lanes // 2
  return None


class OvertakeDetector:
  def __init__(self):
    self.suggested = False
    self.lane_source = ""  # why the road counts as having a lane to overtake in, for the alert and the logs
    self.speed_limit = 0.0
    self.true_time = 0.0
    self.false_time = 0.0

  def reset(self):
    self.__init__()

  def update(self, enabled, v_ego, speed_limit, lead, mapd_alive, one_way, lanes, left_lane_prob, lane_width_left, signalling):
    """"speed_limit" in m/s, 0 if unknown; "lead" is radarState.leadOne; "signalling" is a blinker on or a lane change under way,
    when the driver has already made up their mind."""
    if not enabled:
      self.reset()
      return

    lane_source = self.lane_available(mapd_alive, one_way, lanes, left_lane_prob, lane_width_left)

    candidate = speed_limit >= 1 and MIN_SPEED <= v_ego <= speed_limit - BELOW_LIMIT
    candidate &= bool(lead.status) and lead.dRel <= MAX_LEAD_DISTANCE and lead.vLead <= speed_limit - BELOW_LIMIT
    candidate &= lane_source is not None
    candidate &= not signalling

    if candidate:
      self.true_time += DT_MDL
      self.false_time = 0.0
      self.lane_source = lane_source
      self.speed_limit = speed_limit
    else:
      self.true_time = 0.0
      self.false_time += DT_MDL

    if signalling:
      self.suggested = False
    elif self.true_time >= HOLD_TIME:
      self.suggested = True
    elif self.false_time >= RELEASE_TIME:
      self.suggested = False

    if not self.suggested and not candidate:
      self.lane_source = ""
      self.speed_limit = 0.0

  @staticmethod
  def lane_available(mapd_alive, one_way, lanes, left_lane_prob, lane_width_left):
    """A description of where the lane to overtake in was seen ("2 lanes (map)", "left lane (camera)"), or None if there isn't one."""
    lanes_ahead = map_lanes_ahead(one_way, lanes) if mapd_alive else None
    if lanes_ahead is not None:
      return f"{lanes_ahead} lanes (map)" if lanes_ahead >= 2 else None
    if left_lane_prob >= MIN_LANE_PROB and lane_width_left >= MIN_LANE_WIDTH:
      return "left lane (camera)"
    return None
