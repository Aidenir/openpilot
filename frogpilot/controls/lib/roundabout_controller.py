#!/usr/bin/env python3
import dataclasses

import numpy as np

from openpilot.common.constants import CV

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
