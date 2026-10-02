#!/usr/bin/env python3
import dataclasses
import math

from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib.speed_bump_controller import CONFIRM_SAMPLES, SpeedBumpController

# "Force Stops": brake for a red light or stop sign the driving model is stopping for, to a point worked out from the model's
# plan, the way a speed bump is braked for (a jerk-limited deceleration request plus a cruise cap, see speed_bump_controller).
#
# FrogPilot's own force stop capped the cruise speed by the model's plan length. On Malmö's lights (drives 261-269, 2026-09-30
# and 10-01) the model saw the light 50-85 m out but planned to stop a median 8-20 m past where the driver stopped, coasting at
# 0.2-0.5 m/s^2 until ~30 m out and only then braking at ~2 m/s^2. Close in it can lose the light altogether: one approach still
# planned 10 m/s 5 m from the line. Swedish signal heads stand at the stop line and leave the camera's view as the car closes in,
# where American ones hang across the junction and stay in sight. Drivers stopped at 0.7-0.8 of the model's stop point however
# far out it was. Replayed with a stop point taken from where the plan slows below STOP_SPEED, scaled by STOP_POINT_SCALE and
# kept while the light is, braking started 28-60 m out at a peak of 1.4-2.7 m/s^2, instead of 20-35 m out at 2+ m/s^2 and the
# driver taking over. Five of the six stopped within 4 m of the driver's stop; one 9 m short, where the model planned to slow
# to a crawl well before the line.

STOP_SPEED = 2.0         # m/s, the model's plan counts as stopping where its speed first drops below this
STOP_POINT_SCALE = 0.85  # Where the model's stop point is braked for, as a fraction of its distance
SHRINK_TIME = 1.0        # s, time constant a stop point coming closer is followed with, so a few short plans don't stop the car early
GROW_TIME = 5.0          # s, and one moving further away
RELEASE_TIME = 0.5       # s, the stop has to be gone this long before the brakes come off, so one dropped sample doesn't release them

# Braking settings. The car's response time and the comfort jerk come from the speed bump settings
BRAKE_TIME = 5.0  # s at the current speed before the stop point where braking starts: ~55 m from 40 km/h, at ~1.2 m/s^2
MAX_DECEL = 3.0   # m/s^2
MARGIN = 1.5      # m, be stopped this far before the stop point


def model_stop_distance(position_x, velocity_x):
  """Distance (m) to where the model's plan first slows below STOP_SPEED, or None if it doesn't."""
  for x, v in zip(position_x, velocity_x, strict=False):
    if v < STOP_SPEED:
      return float(x)
  return None


def red_light_config(config):
  """The speed bump settings, stopping at the stop point and holding there until let go."""
  return dataclasses.replace(config, v_target=0.0, brake_time=BRAKE_TIME, max_decel=MAX_DECEL, strict=False, margin=MARGIN,
                             hold_distance=math.inf)


class RedLightController(SpeedBumpController):
  def reset(self):
    super().reset()
    self.stop_distance = None  # m, to the stop point being braked for
    self.clear_time = 0.0

  @property
  def stopping(self):
    """Braking for, or holding at, a stop point."""
    return self.stop_distance is not None and self.braking and not self.too_late

  def track(self, has_bump, distance, v_ego, dt=DT_MDL, length=0.0):
    # update() works the stop point out itself, so it is taken as it is
    if not has_bump:
      self.tracked_distance = None
      self.new_bump()
      return
    if self.tracked_distance is None:
      self.new_bump()
    self.tracked_distance = distance
    self.tracked_length = 0.0
    self.confirmed_samples = CONFIRM_SAMPLES

  def update(self, active, stop_wanted, position_x, velocity_x, v_ego, config, dt=DT_MDL):
    """"stop_wanted": a red light or stop sign is being stopped for. "position_x" and "velocity_x" are the model's plan.
    Returns (speed cap in m/s or None, requested deceleration in m/s^2 or 0.0)."""
    if self.stop_distance is not None:
      self.stop_distance -= v_ego * dt

    if stop_wanted:
      # A stop point is only taken from a plan that comes to a stop: one that only slows down is a speed bump or a turn as often as
      # a light. Once found it is kept while the light is: close in the model can lose the light and plan to drive on
      self.clear_time = 0.0
      estimate = model_stop_distance(position_x, velocity_x)
      if estimate is not None:
        estimate *= STOP_POINT_SCALE
        if self.stop_distance is None:
          self.stop_distance = estimate
        else:
          time_constant = SHRINK_TIME if estimate < self.stop_distance else GROW_TIME
          self.stop_distance += (estimate - self.stop_distance) * min(1.0, dt / time_constant)
    elif self.stop_distance is not None:
      self.clear_time += dt
      if self.clear_time >= RELEASE_TIME:
        self.stop_distance = None

    target, decel = super().update(active, self.stop_distance is not None, self.stop_distance or 0.0, v_ego, red_light_config(config), dt)
    if target is not None and self.stopping:
      # The speed bump braking eases off as it lands on its speed, it doesn't stop. Also cap the speed to what MAX_DECEL can still
      # stop from by the stop point, so the car is brought to a stop there however it got this close
      target = min(target, math.sqrt(2.0 * MAX_DECEL * max(0.0, self.tracked_distance - MARGIN)))
      self.target = target
    return target, decel
