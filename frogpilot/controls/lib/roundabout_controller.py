#!/usr/bin/env python3
import dataclasses
import math

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

# mapd publishes "nextRoundaboutDistance" (metres along the road to where it meets the ring's centreline) and
# "nextRoundaboutDiameter", the ring's centreline diameter fitted to its OpenStreetMap ways (0 for a mini roundabout).
# The car is slowed for the entry by a SpeedBumpController with the speed bump settings, a speed from the diameter and no
# hold: once in the ring the cap is lifted at the bump release rate and the usual curve handling takes over.
#
# How fast a roundabout can be entered depends mostly on its size. The speeds are set by two Malmö roundabouts:
# Ärtholmsvägen x Bellevuevägen, 18.4 m across, entered at 15 km/h, and Agnesfridsvägen x Arrievägen, 57.6 m, at 30 km/h.
# Malmö's run from 14 m to 213 m, half of them 22-39 m (19-23 km/h here). Beyond MAX_DIAMETER they are large interchanges,
# usually signalled, driven like any other junction.
DIAMETER_BP = [12.0, 18.4, 57.6, 100.0]                        # m
SPEED_V = [v * CV.KPH_TO_MS for v in [12.0, 15.0, 30.0, 40.0]]  # m/s
MAX_DIAMETER = 100.0  # m, larger rings are not slowed for
MINI_SPEED = 20 * CV.KPH_TO_MS  # m/s, a painted mini roundabout (no ring to measure) that can be driven over

# The give-way line is before the ring's centreline by about half the ring's carriageway, so brake for that
ENTRY_OFFSET = 4.0  # m

def roundabout_speed(diameter):
  """Entry speed (m/s) for a roundabout "diameter" metres across (0 = mini roundabout), or None if it's too big to slow for."""
  if diameter <= 0:
    return MINI_SPEED
  if diameter > MAX_DIAMETER:
    return None
  return float(np.interp(diameter, DIAMETER_BP, SPEED_V))

def roundabout_config(config, v_target):
  """The speed bump settings, aiming for "v_target" and letting go at the entry."""
  return dataclasses.replace(config, v_target=v_target, hold_distance=0.0)


# Inside a ring mapd can report the roundabout the car is driving round as one still ahead. On Limhamnsvägen heading south-west
# (drive 27d, 2026-10-05) it reported the 22 m ring being entered as one 74.7 m ahead the moment the car was in it, the car was
# braked to 19 km/h for it past the exit, and mapd dropped it 44 m before it got there. OpenStreetMap has one ring there.
REACHED_DISTANCE = 10.0  # m to the ring's centreline, a roundabout last reported closer than this and then gone (or further) was reached
RING_EXTRA = 30.0        # m on top of the ring's circumference that reports of it are ignored for: mapd's 74.7 m was 5.6 m more
ROUNDABOUT_STALE_TIMEOUT = 0.5  # s, a roundabout mapd stops reporting while still ahead is let go of after this (bumps: STALE_TIMEOUT)
SAME_RING = 0.5          # m, a roundabout whose fitted diameter is this close to the one just entered is taken to be that ring


class RingFilter:
  """Hides mapd's reports of the ring the car has just entered, until it has driven that ring's circumference and RING_EXTRA past
  the entry. The fitted diameter tells that ring apart from the next roundabout along the road, which is reported as usual, as
  are mini roundabouts (no ring)."""
  def __init__(self):
    self.quiet = 0.0           # m left before reports of the ring just entered count again
    self.ring_diameter = 0.0   # m, of the ring just entered
    self.last_distance = None  # m, mapd's last distance to the ring's centreline, None if no roundabout reported
    self.last_diameter = 0.0

  def update(self, has_roundabout, distance, diameter, v_ego, dt=DT_MDL):
    """mapd's "hasNextRoundabout", "nextRoundaboutDistance" (to the ring's centreline) and "nextRoundaboutDiameter". Returns whether
    the roundabout reported should be slowed for."""
    self.quiet = max(0.0, self.quiet - v_ego * dt)
    reached = self.last_distance is not None and self.last_distance < REACHED_DISTANCE and \
              (not has_roundabout or distance > self.last_distance + REACHED_DISTANCE)
    if reached and self.last_diameter > 0:
      self.quiet = math.pi * self.last_diameter + RING_EXTRA
      self.ring_diameter = self.last_diameter
    if has_roundabout:
      self.last_distance, self.last_diameter = distance, diameter
    else:
      self.last_distance = None
    same_ring = self.quiet > 0 and abs(diameter - self.ring_diameter) < SAME_RING and distance < self.quiet
    return has_roundabout and not same_ring
