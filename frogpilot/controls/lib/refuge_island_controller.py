#!/usr/bin/env python3
import numpy as np

from openpilot.common.realtime import DT_MDL

# Keeps the car right of refuge islands, the kerbed islands in the middle of the road where pedestrians cross.
#
# The driving model keeps to the middle of the painted lane and does not treat an island's low kerb as an edge. Where the
# island sits on the centre line with nothing painted to lead traffic around it, the car passes far too close to it
# (Geijersgatan, Malmö, 2026-10-03: the driver had to pull right). mapd publishes the next island ahead ("nextRefugeIsland*",
# from OpenStreetMap and the driver's MARK ISLAND taps), and this moves the car "offset" metres right of the model's line for
# it: eased in over a couple of seconds, held past the island, eased back out.
#
# The model steers back towards its own line all the while. From 1060 frames of logged driving at 22-43 km/h with clear lane
# lines (00000261, 00000269), its desired curvature (right positive, like the model's y) is close to
#
#   curvature = road - A * e - B * e'
#
# with e the car's distance right of its line and e' = de/ds (the heading, s the distance travelled). R^2 0.84-0.93. So to
# follow e(s), the car needs e'' = curvature, and the curvature to add on top of the model's is
#
#   e'' + B * e' + A * e
#
# The car is led along a smooth S-curve (a quintic smoothstep, so the added curvature starts and ends at 0), and this is small:
# for 0.4 m over 32 m, at most ~0.004 1/m (0.45 m/s^2 at 38 km/h) and under 1 m/s^3 of lateral jerk. Islands close enough that
# easing back and over again would overlap are passed in one go.
#
# Nothing is added while the driver signals, in a lane change, or in a sharp turn (the island may be on the road being left),
# and an offset under way is then eased out along the same kind of curve. The same when the driver holds the wheel to the left,
# against it, for OVERRIDE_TIME: then the island is passed as the driver steers it. A hand resting on the wheel reads as
# "pressed" much of the time on this car (torque ~80 either way), and the driver pulling further right is welcome.
SPEED_BP = [7.5, 10.5]            # m/s, the middles of the two speed bands fitted
A_V = [0.0014, 0.0025]            # 1/m^2
B_V = [0.141, 0.107]              # 1/m

MIN_SPEED = 3.0                   # m/s, slower than this the car is not moved over
MAX_SPEED = 17.0                  # m/s (61 km/h), refuge islands are on town streets

HOLD_BEFORE = 6.0                 # m before the island's point (a crossing over it) to be all the way over
HOLD_AFTER = 6.0                  # m after it to stay over: islands run a few metres either side of the crossing
RAMP_IN_TIME = 3.0                # s to ease over
RAMP_OUT_TIME = 3.0               # s to ease back
RAMP_IN_LIMITS = (20.0, 50.0)     # m
RAMP_OUT_LIMITS = (20.0, 50.0)    # m
PREVIEW_TIME = 0.3                # s, the steering's lag behind the curvature asked for

# Room needed right of the car: the model's right road edge must be at least this far right of where the car's right side
# would end up
HALF_CAR_WIDTH = 1.0              # m
EDGE_MARGIN = 0.5                 # m
MAX_EDGE_STD = 1.0                # m, beyond this the model does not really know where the edge is
MIN_OFFSET = 0.1                  # m, less room than this and the car stays where it is

MIN_RAMP = 10.0                   # m, an island first seen with less than this to ease over in is not moved over for
OVERRIDE_TIME = 0.25              # s of the driver pushing the wheel left before letting go of the island

MATCH_DISTANCE = 15.0             # m, a reading within this of a tracked island is that island (GPS wanders)
TRACK_CORRECTION = 0.3            # how far each reading moves a tracked island's position
FREEZE_DISTANCE = 10.0            # m, closer than this a tracked island's position is no longer corrected
MAX_LOOKAHEAD = 150.0             # m, islands further ahead are not tracked yet

MAX_ROAD_CURVATURE = 0.02         # 1/m (50 m radius), sharper and the car is turning, not passing an island
MAX_CURVATURE = 0.008             # 1/m
MAX_LATERAL_ACCEL = 1.0           # m/s^2


def smoothstep(u):
  """f, f' and f'' of the quintic smoothstep 10u^3 - 15u^4 + 6u^5: 0 at u <= 0, 1 at u >= 1, and both derivatives 0 at the ends,
  so the curvature asked for starts and ends at 0 instead of jumping."""
  if u <= 0.0:
    return 0.0, 0.0, 0.0
  if u >= 1.0:
    return 1.0, 0.0, 0.0
  return u ** 3 * (10 - 15 * u + 6 * u ** 2), 30 * u ** 2 * (1 - u) ** 2, 60 * u * (1 - u) * (1 - 2 * u)


class TrackedIsland:
  """One island, or a run of them close together, by odometer reading (m): the car is over from HOLD_BEFORE before "start" to
  HOLD_AFTER after "end"."""
  def __init__(self, position, ramp_in, ramp_out):
    self.start = position
    self.end = position
    self.ramp_in = ramp_in    # m
    self.ramp_out = ramp_out  # m
    self.offset = None        # m right of the model's line to take the car, decided where the easing over starts

  def starts(self, s):
    return s >= self.start - HOLD_BEFORE - self.ramp_in

  def easing_back(self, s):
    return s > self.end + HOLD_AFTER

  def profile(self, s):
    """Offset and its first and second derivatives along the road, with the car at odometer reading s."""
    if not self.offset:
      return 0.0, 0.0, 0.0
    if s < self.start - HOLD_BEFORE:
      f, df, ddf = smoothstep((s - self.start + HOLD_BEFORE + self.ramp_in) / self.ramp_in)
      return self.offset * f, self.offset * df / self.ramp_in, self.offset * ddf / self.ramp_in ** 2
    if not self.easing_back(s):
      return self.offset, 0.0, 0.0
    f, df, ddf = smoothstep(1 - (s - self.end - HOLD_AFTER) / self.ramp_out)
    return self.offset * f, -self.offset * df / self.ramp_out, self.offset * ddf / self.ramp_out ** 2

  def finished(self, s):
    return s > self.end + HOLD_AFTER + self.ramp_out


class Fade:
  """Easing an offset back out, from "offset" at odometer "start" over "length" metres."""
  def __init__(self, start, offset, length):
    self.start = start
    self.offset = offset
    self.length = length

  def profile(self, s):
    f, df, ddf = smoothstep(1 - (s - self.start) / self.length)
    return self.offset * f, -self.offset * df / self.length, self.offset * ddf / self.length ** 2

  def finished(self, s):
    return s - self.start > self.length


class RefugeIslandController:
  def __init__(self):
    self.reset()

  def reset(self):
    self.odometer = 0.0
    self.islands = []
    self.ignored = []  # odometer positions of islands the driver took over for
    self.fade = None
    self.override_time = 0.0
    self.curvature = 0.0
    self.offset = 0.0

  def cancel(self, v_ego):
    """Stop: ease out of any offset now under way, and forget the islands being tracked. Called every frame the reason holds, so a
    fade already easing out is left to finish: restarting it each frame from where it had got to never eased back the other way,
    and held the car pulled left of the model's line, towards the island."""
    if self.offset > 0.0 and (self.islands or self.fade is None):
      self.fade = Fade(self.odometer, self.offset, float(np.clip(v_ego * RAMP_OUT_TIME, *RAMP_OUT_LIMITS)))
    self.ignored += [position for island in self.islands for position in (island.start, island.end)]
    self.islands = []

  def right_room(self, model):
    """How far right the car can be moved, from the model's right road edge; 0 if it is not known."""
    if len(model.roadEdges) < 2 or len(model.roadEdges[1].y) == 0:
      return 0.0
    if len(model.roadEdgeStds) >= 2 and model.roadEdgeStds[1] > MAX_EDGE_STD:
      return 0.0
    return model.roadEdges[1].y[0] - HALF_CAR_WIDTH - EDGE_MARGIN

  def update(self, enabled, lat_active, v_ego, island_found, island_distance, model, steering_pressed, steering_torque, blinker,
             lane_changing, road_curvature, offset_setting):
    """steering_torque: the driver's, left positive. road_curvature: the model's desired curvature (1/m), right positive.
    Sets self.curvature (right positive, to add to the model's) and self.offset (m right of the model's line)."""
    if not (enabled and lat_active):
      self.reset()
      return

    self.odometer += v_ego * DT_MDL

    pushing_left = steering_pressed and steering_torque > 0
    self.override_time = self.override_time + DT_MDL if pushing_left else 0.0

    if self.override_time >= OVERRIDE_TIME or blinker or lane_changing or abs(road_curvature) > MAX_ROAD_CURVATURE or \
       not MIN_SPEED <= v_ego <= MAX_SPEED:
      self.cancel(v_ego)
    elif island_found and island_distance <= MAX_LOOKAHEAD:
      self.track(island_distance, v_ego)

    self.islands = [island for island in self.islands if not island.finished(self.odometer)]
    self.ignored = [position for position in self.ignored if self.odometer - position <= HOLD_AFTER + RAMP_OUT_LIMITS[1]]
    if self.fade is not None and self.fade.finished(self.odometer):
      self.fade = None

    # Where the easing over starts, take the car over by the setting, or as far as there is room for on the right
    preview = self.odometer + v_ego * PREVIEW_TIME
    for island in self.islands:
      if island.offset is None and island.starts(preview):
        island.offset = min(offset_setting, self.right_room(model))
        if island.offset < MIN_OFFSET:
          island.offset = 0.0

    # The car is led along whichever island (or fade) wants it furthest over at this point
    best = (0.0, 0.0, 0.0)
    profiles = [island.profile(preview) for island in self.islands]
    if self.fade is not None:
      profiles.append(self.fade.profile(preview))
    for profile in profiles:
      if profile[0] > best[0]:
        best = profile
    e, de, dde = best

    a = float(np.interp(v_ego, SPEED_BP, A_V))
    b = float(np.interp(v_ego, SPEED_BP, B_V))
    max_curvature = min(MAX_CURVATURE, MAX_LATERAL_ACCEL / max(v_ego, 1.0) ** 2)
    self.curvature = float(np.clip(dde + b * de + a * e, -max_curvature, max_curvature))
    self.offset = float(e)

  def track(self, distance, v_ego):
    position = self.odometer + distance
    preview = self.odometer + v_ego * PREVIEW_TIME
    for island in self.islands:
      for attr in ("start", "end"):
        if abs(getattr(island, attr) - position) <= MATCH_DISTANCE:
          if distance > FREEZE_DISTANCE:
            setattr(island, attr, getattr(island, attr) + TRACK_CORRECTION * (position - getattr(island, attr)))
          return
    if any(abs(ignored - position) <= MATCH_DISTANCE for ignored in self.ignored):
      return

    # The next island close behind one being passed: stay over until past it too, rather than easing back and over again
    for island in self.islands:
      if position > island.end and position - island.end <= island.ramp_in + island.ramp_out and not island.easing_back(preview):
        island.end = position
        return

    # A new island. Seen late (just turned onto the road), the easing over is shortened to what is left, down to MIN_RAMP; any
    # later and the car stays where it is
    ramp_in = min(float(np.clip(v_ego * RAMP_IN_TIME, *RAMP_IN_LIMITS)), distance - HOLD_BEFORE - v_ego * PREVIEW_TIME)
    if ramp_in < MIN_RAMP:
      self.ignored.append(position)
      return
    ramp_out = float(np.clip(v_ego * RAMP_OUT_TIME, *RAMP_OUT_LIMITS))
    self.islands.append(TrackedIsland(position, ramp_in, ramp_out))
