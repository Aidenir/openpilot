#!/usr/bin/env python3
import gc

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring, DRIVER_MONITOR_SETTINGS


def dmonitoringd_thread():
  gc.disable()
  set_realtime_priority(2)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])
  sm = messaging.SubMaster(['driverStateV2', 'liveCalibration', 'carState', 'controlsState', 'modelV2', 'carControl'], poll='driverStateV2')

  # Create settings with user-configured DM timing values
  settings = DRIVER_MONITOR_SETTINGS(params=params)
  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), settings=settings, always_on=params.get_bool("AlwaysOnDM"))

  # FrogPilot variables
  driver_view_enabled = params.get_bool("IsDriverViewEnabled")

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()
    if not sm.updated['driverStateV2']:
      # iterate when model has new output
      continue

    valid = sm.all_checks()
    if valid:
      DM.run_step(sm)
    elif driver_view_enabled:
      DM.face_detected = sm['driverStateV2'].leftDriverData.faceProb > DM.settings._FACE_THRESHOLD or sm['driverStateV2'].rightDriverData.faceProb > DM.settings._FACE_THRESHOLD

    # publish
    dat = DM.get_state_packet(valid=valid or driver_view_enabled)
    pm.send('driverMonitoringState', dat)

    # load live always-on toggle and DM timing settings
    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")
      # Reload DM timing settings every ~2 seconds to allow live updates
      DM.settings = DRIVER_MONITOR_SETTINGS(params=params)
      DM._update_thresholds()

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and
     DM.wheelpos_learner.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
     DM.wheel_on_right == (DM.wheelpos_learner.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool_nonblocking("IsRhdDetected", DM.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
