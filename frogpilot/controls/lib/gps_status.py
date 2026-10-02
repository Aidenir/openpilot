#!/usr/bin/env python3
# How the GPS is doing, for the onroad GPS indicator (frogpilotPlan.gpsStatus). On 2026-10-02 the modem twice went a whole drive
# without a fix, once sending nothing at all and once searching with a wrong time, and nothing on screen said so.
#
# qcomgpsd publishes gpsLocation only for position reports, and qcomGnss for the measurements the modem sends several times a
# second whether or not it has a fix, so the two tell "searching" apart from "the modem has gone quiet".

GPS_STATUS_UNKNOWN = 0    # Not worked out yet
GPS_STATUS_NO_GPS = 1     # Nothing from the GPS for STALE_TIME
GPS_STATUS_SEARCHING = 2  # The GPS is running but has no fix
GPS_STATUS_WEAK = 3       # A fix, but a poor one
GPS_STATUS_GOOD = 4

STALE_TIME = 3.0          # s
WEAK_ACCURACY = 15.0      # m, a horizontal accuracy worse than this is weak (u-blox reports it)
WEAK_VDOP = 3.0           # qcomgpsd reports no horizontal accuracy, only the VDOP, as "verticalAccuracy": 1.0-1.4 in Malmö


def gps_status(now, location, location_time, measurement_time):
  """"location" is the latest gpsLocation(External) and "location_time" when it came (s, None if never), "measurement_time" when
  the GPS last sent anything else (qcomGnss or ubloxGnss)."""
  def fresh(t):
    return t is not None and now - t < STALE_TIME

  if fresh(location_time) and location.hasFix:
    if location.horizontalAccuracy > 0:
      weak = location.horizontalAccuracy > WEAK_ACCURACY
    else:
      weak = location.verticalAccuracy > WEAK_VDOP
    return GPS_STATUS_WEAK if weak else GPS_STATUS_GOOD

  if fresh(location_time) or fresh(measurement_time):
    return GPS_STATUS_SEARCHING

  return GPS_STATUS_NO_GPS
