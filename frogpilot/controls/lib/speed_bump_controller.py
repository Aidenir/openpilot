#!/usr/bin/env python3
import math
from dataclasses import dataclass

import numpy as np

from openpilot.common.realtime import DT_MDL

# mapd publishes "nextSpeedBumpDistance" (metres along the road to the MIDDLE of the next mapped
# speed bump) and "hasNextSpeedBump". This turns that into a cruise speed cap plus a requested
# deceleration: braking starts a set time before the bump at the current speed, and is just firm
# enough to be at the bump speed as the bump starts. The bump speed is held until it's over.
#
# The requested deceleration is needed because a lowered cruise speed alone never makes the MPC brake
# harder than ~1.2-2.4 m/s^2, and it only ramps into that over about a second: slowing from 50 to
# 20 km/h in ~25 m takes ~3 m/s^2. It is published as frogpilotPlan.speedBumpDecel and applied by
# the longitudinal planner as an upper bound on the acceleration it outputs.

CONFIRM_SAMPLES = 3     # Consecutive consistent samples before a bump is acted on (~0.15s), so one bad sample can't brake
SAME_BUMP_TOLERANCE = 10.0  # m, how far a new sample may disagree with the dead-reckoned position and still be the same bump
PASSED_WINDOW = 10.0    # m, inside this a confirmed bump that vanishes or jumps further away is taken as being passed
STALE_TIMEOUT = 2.0     # s, forget a bump mapd stopped reporting while it was still well ahead
CLAMPED_DISTANCE = 1.0  # m, a report this close right after crossing a bump is mapd's clamped distance to that bump
PASSED_MEMORY = 50.0    # m, how far past a crossed bump mapd's reports of it are still ignored

MIN_STOP_DISTANCE = 0.5  # m, floor on the remaining distance when working out the deceleration, to keep it finite
JERK_BP = [5.0, 20.0]   # m/s, how fast the braking may build up and ease off: 1.5 m/s^3 at low speed down to 1.2 m/s^3 above
JERK_V = [1.5, 1.2]     # 20 m/s, scaled by SpeedBumpConfig.jerk_scale. ISO 15622's 5 m/s^3 / 2.5 m/s^3 envelope is what
                        # ACC may do, not what is comfortable: on the car it hit the brakes at ~13 m/s^3 and lurched on release
RELEASE_LAG_EXTRA = 0.2  # s, easing off the brakes takes this much longer than applying them (the brakes' own smoothing);
                        # with the default 0.3s response time this gives the 0.5s tuned in closed-loop sim against the MPC
TOO_LATE_FACTOR = 1.5   # A bump first confirmed well past its braking point that would need more than this times the max
                        # deceleration is not braked hard for; the cruise speed is still lowered, which brakes gently
ON_TIME_SLACK = 0.5     # s, a bump confirmed within this long after its braking point still counts as on time
RELEASE_RAMP = 3.0      # m/s per second the cap is raised by once a bump is over, so the car pulls away rather than lurching.
                        # 1 m/s per second held the pull-away back noticeably (7.4 s instead of 4.5 s from 14 to 35 km/h);
                        # at 3 it is within 0.2 s of no ramp, and the planner's eased hand-over keeps the jerk at 1.5 m/s^3
RELEASE_DONE = 1.5      # m/s, the raised cap is dropped once it is this far above the car's speed and no longer holds it back

@dataclass
class SpeedBumpConfig:
  """The user's settings, in SI units. Defaults match the params' defaults."""
  v_target: float = 20 / 3.6  # m/s, speed over the bump
  brake_time: float = 3.0     # s at the current speed before the bump where braking starts
  max_decel: float = 3.0      # m/s^2, firmest braking allowed
  strict: bool = False        # True: always start at "brake_time", even if that means arriving faster than "v_target"
  response_lag: float = 0.3   # s, how long the car takes to act on a new acceleration request
  margin: float = 2.0         # m, be at "v_target" this far before the bump's start (its middle if its length is unknown)
  hold_distance: float = 6.0  # m, keep "v_target" until this far past the end of the bump (its middle if its length is unknown)
  jerk_scale: float = 1.0     # scales the comfort jerk envelope; lower is a softer ramp in and out

DEFAULT_CONFIG = SpeedBumpConfig()

def max_jerk(v_ego, config=DEFAULT_CONFIG):
  return float(np.interp(v_ego, JERK_BP, JERK_V)) * config.jerk_scale

def braking_distance(v_from, v_to, decel):
  """Distance (m) to slow from "v_from" to "v_to" at a constant "decel" (m/s^2)."""
  return max(0.0, v_from**2 - v_to**2) / (2.0 * decel)

def feasible_start_distance(v_ego, config=DEFAULT_CONFIG):
  """The latest distance to the bump's middle (m) from which "max_decel" still reaches "v_target" by the margin,
  counting the distance covered before the car responds and while the braking ramps in and back out at the jerk limit."""
  speed_drop = v_ego - config.v_target
  if speed_drop <= 0:
    return config.margin
  # Braking ramps up and back down at the jerk limit, so a small speed drop never reaches "max_decel": ramping to a
  # peak and back costs peak^2 / jerk of speed. Each ramp is worth half its duration at constant braking
  jerk = max_jerk(v_ego, config)
  peak = min(config.max_decel, math.sqrt(jerk * speed_drop))
  ramp_in = v_ego * (config.response_lag + peak / jerk / 2)
  ease_out = config.v_target * peak / jerk / 2
  return config.margin + ramp_in + braking_distance(v_ego, config.v_target, peak) + ease_out

def brake_start_distance(v_ego, config=DEFAULT_CONFIG):
  """Distance to the bump's middle (m) at which braking starts: "brake_time" seconds out at the current speed. Unless
  strict, never so late that "max_decel" couldn't reach "v_target" in time; braking then starts earlier instead."""
  time_point = v_ego * config.brake_time
  if config.strict:
    return time_point
  return max(time_point, feasible_start_distance(v_ego, config))

def required_decel(distance, v_ego, config=DEFAULT_CONFIG, ease_distance=0.0):
  """Constant deceleration (m/s^2, positive) that reaches "v_target" "margin" before the bump's middle,
  allowing for the "response_lag" seconds travelled before the car acts on it and "ease_distance" metres
  covered while the brakes come off at the jerk limit."""
  remaining = distance - config.margin - v_ego * config.response_lag - ease_distance
  return braking_distance(v_ego, config.v_target, 1.0) / max(remaining, MIN_STOP_DISTANCE)

class SpeedBumpController:
  def __init__(self):
    self.reset()

  def reset(self):
    self.hold_distance = SpeedBumpConfig.hold_distance

    # The last cap set for a bump, and the cap being raised from it once the bump is over
    self.last_target = None
    self.release_cap = None

    # Distances are to the START of a bump: mapd reports its middle, so half its length comes off. The length is kept
    # with the bump it belongs to, since mapd reports the next bump's length as soon as this one's middle is crossed
    self.tracked_distance = None
    self.tracked_length = 0.0
    self.confirmed_samples = 0
    self.unseen_time = 0.0

    # A further bump mapd has moved on to while we're still holding for the one being crossed, so
    # bumps in quick succession are chained without a gap in the cap
    self.pending_distance = None
    self.pending_length = 0.0
    self.pending_samples = 0

    # The bump we last finished holding for. mapd dead-reckons between position fixes and clamps a
    # passed bump's distance to 0 until it re-evaluates, so that has to be recognised and not re-adopted
    self.passed_distance = None

    self.new_bump()

    self.decel = 0.0
    self.target = None

  def new_bump(self):
    # Per-bump braking state: whether braking has started for it, and whether it was found too late to brake hard for
    self.braking = False
    self.too_late = False
    self.cap = None

  @property
  def hold_end(self):
    # How far past its start the tracked bump is held for: all of it, then "hold_distance"
    return self.tracked_length + self.hold_distance

  def forget(self):
    if self.tracked_distance is not None and self.tracked_distance < -self.hold_end:
      self.passed_distance = self.tracked_distance

    self.tracked_distance = self.pending_distance
    self.tracked_length = self.pending_length
    self.confirmed_samples = self.pending_samples
    self.unseen_time = 0.0

    self.pending_distance = None
    self.pending_length = 0.0
    self.pending_samples = 0

    self.new_bump()

  @property
  def confirmed(self):
    return self.tracked_distance is not None and self.confirmed_samples >= CONFIRM_SAMPLES

  def is_passed_bump(self, distance):
    # Either where the dead-reckoned passed bump is, or mapd's clamped "0" for it however far past it we are
    return self.passed_distance is not None and (distance <= self.passed_distance + SAME_BUMP_TOLERANCE or distance <= CLAMPED_DISTANCE)

  def track(self, has_bump, distance, v_ego, dt=DT_MDL, length=0.0):
    # Dead-reckon the bumps we know of, so short dropouts, a frozen mapd distance, and the moment mapd
    # moves on to the next bump after we cross this one don't make us lose it
    if self.tracked_distance is not None:
      self.tracked_distance -= v_ego * dt
    if self.pending_distance is not None:
      self.pending_distance -= v_ego * dt
    if self.passed_distance is not None:
      self.passed_distance -= v_ego * dt
      if self.passed_distance < -PASSED_MEMORY:
        self.passed_distance = None

    # Once braking for it or this close, a bump that vanishes or jumps away has most likely just been passed, so keep holding for it
    committed = self.confirmed and (self.braking or self.tracked_distance <= PASSED_WINDOW)

    if has_bump and math.isfinite(distance):
      if self.tracked_distance is not None and abs(distance - self.tracked_distance) <= SAME_BUMP_TOLERANCE:
        # Same bump; trust whichever is closer so a lagging mapd can only make us slow down earlier, not later
        self.tracked_distance = min(distance, self.tracked_distance)
        self.tracked_length = length
        self.confirmed_samples += 1
        self.unseen_time = 0.0
      elif committed and distance > self.tracked_distance:
        # mapd has moved on to a further bump while we're still on this one, so keep holding this one
        # and start confirming the next in the background
        self.unseen_time += dt
        if self.pending_distance is not None and abs(distance - self.pending_distance) <= SAME_BUMP_TOLERANCE:
          self.pending_distance = min(distance, self.pending_distance)
          self.pending_samples += 1
        else:
          self.pending_distance = distance
          self.pending_samples = 1
        self.pending_length = length
      elif self.is_passed_bump(distance):
        # mapd still reporting the bump we've already crossed
        if self.tracked_distance is not None:
          self.unseen_time += dt
      else:
        # A new (or re-placed) bump; it has to be seen consistently before it's acted on
        self.tracked_distance = distance
        self.tracked_length = length
        self.confirmed_samples = 1
        self.unseen_time = 0.0
        self.pending_distance = None
        self.pending_samples = 0
        self.new_bump()
    elif self.tracked_distance is not None:
      self.unseen_time += dt

    if self.tracked_distance is not None and not committed and self.unseen_time > 0 and (not self.confirmed or self.unseen_time > STALE_TIMEOUT):
      self.forget()

    if self.tracked_distance is not None and self.tracked_distance < -self.hold_end:
      self.forget()

  def update(self, active, has_bump, distance, v_ego, config, dt=DT_MDL, length=0.0):
    """"distance" is mapd's distance to the middle of the next bump and "length" its length along the road (0 if
    unknown), so braking aims for the bump's start, "length / 2" before the middle, and holds until it's crossed."""
    length = max(0.0, length) if has_bump else 0.0
    target, decel = self.update_bump(active, has_bump, distance - length / 2, v_ego, config, dt, length)
    if target is not None:
      self.last_target = target
      self.release_cap = None
    elif active and self.last_target is not None:
      # The bump is over. Dropping the cap in one step made the MPC jump straight to its full acceleration; raise it instead
      self.release_cap = (self.release_cap or self.last_target) + RELEASE_RAMP * dt
      if self.release_cap > v_ego + RELEASE_DONE:
        self.last_target = self.release_cap = None
      else:
        target = self.release_cap
    else:
      self.last_target = self.release_cap = None
    self.target = target
    return target, decel

  def update_bump(self, active, has_bump, distance, v_ego, config, dt=DT_MDL, length=0.0):
    """Returns (speed cap in m/s or None, requested deceleration in m/s^2 or 0.0).

    Nothing happens until the car is "brake_time" seconds (at its current speed) from the bump, or, unless strict,
    further out if "max_decel" couldn't otherwise reach "v_target" in time. From there the cruise speed is capped to
    "v_target" and the deceleration that reaches it "margin" before the bump's middle is requested. It is recomputed
    every step, so any lag in the car's response raises it (up to "max_decel") instead of arriving fast, and it is
    jerk-limited both ways, easing off as the speed nears "v_target" so it lands on it rather than dipping below.
    The cap is held until "hold_distance" past the end of the bump."""
    self.hold_distance = config.hold_distance
    self.track(has_bump, distance, v_ego, dt, length)

    previous_decel = self.decel
    self.decel = 0.0
    self.target = None

    if not self.confirmed:
      return None, 0.0

    d = self.tracked_distance
    start = brake_start_distance(v_ego, config)
    if not self.braking and d <= start:
      self.braking = True
      on_time = d >= start - v_ego * ON_TIME_SLACK
      self.too_late = not on_time and required_decel(d, v_ego, config) > config.max_decel * TOO_LATE_FACTOR

    if not active or not self.braking:
      return None, 0.0

    v_target = config.v_target
    jerk = max_jerk(v_ego, config)
    desired = 0.0
    if not self.too_late and v_ego > v_target and d > config.margin:
      # The most deceleration that can still be eased off at the jerk limit by "v_target", allowing for the speed still lost
      # while the car responds to the easing, so it lands on the bump speed rather than dipping below it
      release_lag = config.response_lag + RELEASE_LAG_EXTRA
      landing = math.sqrt(2.0 * jerk * max(0.0, v_ego - v_target - previous_decel * release_lag))
      # Easing off from the current deceleration takes "previous_decel / jerk" seconds at about the bump speed, worth half
      # that at full braking; that distance isn't available for braking, or the car is still easing off as it reaches the bump
      ease_distance = v_target * previous_decel / jerk / 2
      desired = min(required_decel(d, v_ego, config, ease_distance), config.max_decel, landing)
    # Eased off as gently as it is applied: releasing twice as fast made the car lurch as the bump speed was reached
    self.decel = float(np.clip(desired, previous_decel - jerk * dt, previous_decel + jerk * dt))
    self.decel = max(self.decel, 0.0)

    if self.too_late:
      # No deceleration is requested for a bump found too late, so the lowered cruise speed is what slows the car, gently
      self.target = v_target
    else:
      # The requested deceleration does the slowing, so the cruise cap only has to stop the MPC accelerating: it follows the
      # car's speed down to "v_target". Dropping it straight to "v_target" made the MPC brake hard on its own the moment the
      # braking started, a step in the command on top of the jerk-limited request
      self.cap = v_ego if self.cap is None else min(self.cap, v_ego)
      self.target = max(v_target, self.cap)
    return self.target, self.decel
