#!/usr/bin/env python3
import dataclasses
import math

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.frogpilot.controls.lib.speed_bump_controller import CONFIRM_SAMPLES, SpeedBumpController

# "Force Stops": brake for a red light or stop sign the driving model is stopping for, to a point worked out from the model's
# plan, the way a speed bump is braked for (a jerk-limited deceleration request plus a cruise cap, see speed_bump_controller).
#
# FrogPilot's own force stop capped the cruise speed by the model's plan length. On Malmö's lights (drives 261-269) the model saw
# the light 50-85 m out but planned to stop past the line, coasting until ~30 m out and then braking at ~2 m/s^2, and close in it
# could lose the light (Swedish signal heads stand at the stop line and leave the camera's view). Far out the model's stop point is
# noisy, 15-20% short at some lights and 20-25% long at others (drives 269-275), so it is taken as the median of the last
# MEDIAN_TIME of plans, followed closer quickly and further away more slowly. Close in a plan that comes to rest is the better
# estimate: its end, less END_OFFSET, was within 2.5 m of where drivers stopped from 45 m in. Replayed on 15 stops from drives
# 269-275 this stopped a mean 3.2 m from the driver's stop, against 4.7 m for the stop point scaled by 0.85 it replaced (drive 275:
# Eriksfältsgatan 8.8 -> 4 m short, Ystadvägen x Nobelvägen 4.6 -> 1 m short and no longer braking from 75 m out for one short plan).

STOP_SPEED = 2.0         # m/s, a plan that doesn't come to rest is taken to stop where its speed first drops below this
SLOWING_SCALE = 0.9      # applied to the stop point of a plan that doesn't come to rest, which lands 5-20% long 45-60 m out
END_SPEED = 1.0          # m/s, a plan ending slower than this comes to rest within its horizon
END_OFFSET = 2.0         # m, drivers stopped this far short of where such a plan ends (drives 269-275: +2-6 m, it creeps up to the
                         # light), and it is the better estimate: within 2.5 m of the driver's stop from 45 m in
MEDIAN_TIME = 0.5        # s of estimates the stop point is taken from the median of, so one odd plan can't set it: drive 275 braked
                         # at 1.3 m/s^2 from 75 m out for a single plan that stopped 13 m short (Ystadvägen x Nobelvägen)
MIN_ESTIMATES = 1        # estimates before a stop point is taken: a red light seen late needs braking straight away
LOST_TIME = 1.5          # s without a planned stop after which a stop point still LOST_KEEP_DISTANCE off is let go: real lights had
LOST_KEEP_DISTANCE = 20.0  # m gaps of up to 1.3 s (one of 3.2 s 90-60 m out), a bump and a roundabout 7 s (drive 275, Lorensborgsgatan)
EXPLAINED_BEFORE = 8.0   # m, a planned stop this far before a mapped bump or roundabout entry, or EXPLAINED_AFTER past it, is for
EXPLAINED_AFTER = 15.0   # m   that, not a light
SIGNAL_OFFSET = 1.0      # m before a mapped traffic signal or stop sign that the stop point is put when one is near the model's (MARGIN
                         # comes off too): drive 275's red lights were stopped 1-6 m before the signal's node, ~3.5 m on average
SIGNAL_NEAR = 25.0       # m, closer than this the model's own stop point is used: OpenStreetMap puts signals 1-6 m past where drivers
                         # stopped, no better than the model this close in, which far out it is 15-25% out
SIGNAL_WINDOW = 15.0     # m, how near the model's stop point a mapped signal has to be for it to be used, or SIGNAL_WINDOW_SHARE of its
SIGNAL_WINDOW_SHARE = 0.35  # distance if that is more: far out the model's stop point is 15-25% out either way
SHRINK_TIME = 0.5        # s, time constant the stop point follows that median closer with: a red light found late needs braking at once
GROW_TIME = 2.0          # s, and further away with. It was 5 s, so one early short plan held the stop point short for most of the
                         # approach (drive 275, Eriksfältsgatan: 49 m for a stop 65 m out)
HOLD_DISTANCE = 15.0     # m, inside this the stop point no longer comes closer: a slow car's plan drops below STOP_SPEED almost at
                         # once and its stop point collapses towards it (drive 274: 2-4 m short and a late squeeze)
RELEASE_TIME = 0.5       # s, the stop has to be gone this long before the brakes come off, so one dropped sample doesn't release them

# Braking settings. The car's response time and the comfort jerk come from the speed bump settings
BRAKE_TIME = 5.0  # s at the current speed before the stop point where braking starts: ~55 m from 40 km/h, at ~1.2 m/s^2
MAX_DECEL = 3.0   # m/s^2
MARGIN = 2.5      # m, be stopped this far before the stop point


def model_stop_distance(position_x, velocity_x):
  """Distance (m) to where the model's plan stops the car: END_OFFSET short of its end if it comes to rest, else where it first slows
  below STOP_SPEED, or None if it does neither."""
  if not len(position_x):
    return None
  if velocity_x[-1] < END_SPEED:
    return max(0.0, float(position_x[-1]) - END_OFFSET)
  for x, v in zip(position_x, velocity_x, strict=False):
    if v < STOP_SPEED:
      return float(x) * SLOWING_SCALE
  return None


def red_light_config(config):
  """The speed bump settings, stopping at the stop point and holding there until let go."""
  return dataclasses.replace(config, v_target=0.0, brake_time=BRAKE_TIME, max_decel=MAX_DECEL, strict=False, margin=MARGIN,
                             hold_distance=math.inf)


class RedLightController(SpeedBumpController):
  def reset(self):
    super().reset()
    self.stop_distance = None  # m, to the stop point being braked for
    self.estimates = []        # the latest MEDIAN_TIME of stop point estimates (None for none), dead-reckoned to now
    self.missing_time = 0.0    # s since the plan last came to a stop
    self.clear_time = 0.0

  @property
  def stopping(self):
    """Braking for, or holding at, a stop point."""
    return self.stop_distance is not None and self.braking and not self.too_late

  def track(self, has_bump, distance, v_ego, dt=DT_MDL, length=0.0, severity=-1.0):
    # update() works the stop point out itself, so it is taken as it is. A stop point has no learned severity, so the speed bump
    # "mild bump" extra speed never applies to it
    if not has_bump:
      self.tracked_distance = None
      self.new_bump()
      return
    if self.tracked_distance is None:
      self.new_bump()
    self.tracked_distance = distance
    self.tracked_length = 0.0
    self.tracked_severity = -1.0
    self.confirmed_samples = CONFIRM_SAMPLES

  def update(self, active, stop_wanted, position_x, velocity_x, v_ego, config, dt=DT_MDL, slowdowns=(), signal=None):
    """"stop_wanted": a red light or stop sign is being stopped for. "position_x" and "velocity_x" are the model's plan.
    "slowdowns": distances (m) to mapped speed bumps and roundabout entries ahead, which the model slows for too.
    "signal": distance (m) to the next mapped traffic signal or stop sign facing this way, None if none (or no map).
    Returns (speed cap in m/s or None, requested deceleration in m/s^2 or 0.0)."""
    if self.stop_distance is not None:
      self.stop_distance -= v_ego * dt
    self.estimates = [None if e is None else e - v_ego * dt for e in self.estimates]

    if stop_wanted:
      # A stop point is only taken from a plan that comes to a stop: one that only slows down is a speed bump or a turn as often as
      # a light. Once found it is kept while the light is: close in the model can lose the light and plan to drive on
      self.clear_time = 0.0
      estimate = model_stop_distance(position_x, velocity_x)
      if estimate is not None and any(-EXPLAINED_BEFORE <= estimate - s <= EXPLAINED_AFTER for s in slowdowns):
        # The model is stopping for a mapped bump or roundabout, which have their own slowdown (drive 275, Lorensborgsgatan: braked
        # at 1.3 m/s^2 for a bump and a roundabout with no light there)
        estimate = None
      if estimate is not None and signal is not None:
        # The model is stopping near a mapped signal or stop sign: that places the stop far better than the model can from far out.
        # The map only ever helps: with no signal mapped (OpenStreetMap has Ystadvägen x Heleneholmsstigen's lights as an
        # uncontrolled crossing) the model's stop point is used as it is
        anchored = max(0.0, signal - SIGNAL_OFFSET)
        if anchored > SIGNAL_NEAR and abs(estimate - anchored) <= max(SIGNAL_WINDOW, SIGNAL_WINDOW_SHARE * anchored):
          estimate = anchored
      self.estimates = (self.estimates + [estimate])[-max(1, round(MEDIAN_TIME / dt)):]
      self.missing_time = 0.0 if estimate is not None else self.missing_time + dt

      found = [e for e in self.estimates if e is not None]
      if len(found) >= MIN_ESTIMATES:
        median = float(np.median(found))
        if self.stop_distance is None:
          self.stop_distance = median
        elif not (median < self.stop_distance < HOLD_DISTANCE):
          time_constant = SHRINK_TIME if median < self.stop_distance else GROW_TIME
          self.stop_distance += (median - self.stop_distance) * min(1.0, dt / time_constant)

      if self.stop_distance is not None and self.missing_time >= LOST_TIME and self.stop_distance > LOST_KEEP_DISTANCE:
        # Still far off and the plan hasn't come to a stop for a while: it was slowing for something else (drive 275: 7 s of no
        # stop in the plan, braked at 1.3 m/s^2 all the same). Near the line it is kept, the model can lose the light there
        self.stop_distance = None
    else:
      self.estimates = []
      self.missing_time = 0.0
      if self.stop_distance is not None:
        self.clear_time += dt
        if self.clear_time >= RELEASE_TIME:
          self.stop_distance = None

    target, decel = super().update(active, self.stop_distance is not None, self.stop_distance or 0.0, v_ego, red_light_config(config), dt)
    if self.too_late and not self.overridden:
      # Found too late to stop for (an amber light, or one seen late): let go of it until the light is gone. For a bump a lowered
      # cruise speed slows the car gently, but here it is 0, so the MPC kept slowing towards a stop in or past the junction, and
      # with no stop being forced the driver's gas couldn't override it. Conditional Experimental Mode still has the model in
      # charge, which stops for a light it means to stop for
      self.overridden = True
      self.decel = 0.0
      self.last_target = self.release_cap = None
      self.target = target = None
      decel = 0.0
    if target is not None and self.stopping:
      # The speed bump braking eases off as it lands on its speed, it doesn't stop. Also cap the speed to what MAX_DECEL can still
      # stop from by the stop point, so the car is brought to a stop there however it got this close
      target = min(target, math.sqrt(2.0 * MAX_DECEL * max(0.0, self.tracked_distance - MARGIN)))
      self.target = target
    return target, decel
