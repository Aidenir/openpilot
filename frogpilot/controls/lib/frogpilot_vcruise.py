#!/usr/bin/env python3
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED
from openpilot.frogpilot.controls.lib.curve_speed_controller import CurveSpeedController
from openpilot.frogpilot.controls.lib.red_light_controller import RedLightController
from openpilot.frogpilot.controls.lib.roundabout_controller import ENTRY_OFFSET, roundabout_config, roundabout_speed
from openpilot.frogpilot.controls.lib.speed_bump_controller import SpeedBumpConfig, SpeedBumpController
from openpilot.frogpilot.controls.lib.speed_limit_controller import SpeedLimitController

OVERRIDE_FORCE_STOP_TIMER = 10

class FrogPilotVCruise:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.csc = CurveSpeedController(self)
    self.sbc = SpeedBumpController()
    self.rbc = SpeedBumpController()
    self.rlc = RedLightController()
    self.slc = SpeedLimitController(self)

    self.forcing_stop = False
    self.override_force_stop = False
    self.tracked_model_length = 0.0

    self.override_force_stop_timer = 0

  def update(self, long_control_active, now, time_validated, v_cruise, v_ego, sm, frogpilot_toggles):
    # Stopping for a red light or stop sign (see red_light_controller). Conditional Experimental Mode's stop light detection isn't
    # updated at a standstill, so there the model has to still be planning to stay put, or a green light wouldn't let go
    stop_wanted = self.frogpilot_planner.frogpilot_cem.stop_light_detected and frogpilot_toggles.force_stops
    stop_wanted &= self.frogpilot_planner.model_stopped or not sm["carState"].standstill
    stop_wanted &= self.override_force_stop_timer <= 0

    self.override_force_stop |= sm["carState"].gasPressed
    self.override_force_stop |= sm["frogpilotCarState"].accelPressed
    self.override_force_stop &= self.forcing_stop

    if self.override_force_stop:
      self.override_force_stop_timer = OVERRIDE_FORCE_STOP_TIMER
    elif self.override_force_stop_timer > 0:
      self.override_force_stop_timer -= DT_MDL

    v_cruise_cluster = max(sm["carState"].vCruiseCluster * CV.KPH_TO_MS, v_cruise)
    v_cruise_diff = v_cruise_cluster - v_cruise

    v_ego_cluster = max(sm["carState"].vEgoCluster, v_ego)
    v_ego_diff = v_ego_cluster - v_ego

    # FrogsGoMoo's Curve Speed Controller
    if long_control_active and v_ego > CRUISING_SPEED and self.frogpilot_planner.road_curvature_detected and frogpilot_toggles.curve_speed_controller:
      self.csc.update_target(v_ego)

      self.csc_controlling_speed = True

      self.csc_target = self.csc.target
    else:
      self.csc.log_data(long_control_active, v_ego, sm)

      self.csc_controlling_speed = False
      self.csc.target_set = False

      self.csc_target = v_cruise

    # Pfeiferj's Speed Limit Controller
    self.slc.frogpilot_toggles = frogpilot_toggles

    if frogpilot_toggles.speed_limit_controller:
      self.slc.update_limits(sm["frogpilotCarState"].dashboardSpeedLimit, now, time_validated, v_cruise, v_ego, sm)
      self.slc.update_override(v_cruise, v_cruise_diff, v_ego, v_ego_diff, sm)

      self.slc_offset = self.slc.offset
      self.slc_target = self.slc.target
    elif frogpilot_toggles.show_speed_limits:
      self.slc.update_limits(sm["frogpilotCarState"].dashboardSpeedLimit, now, time_validated, v_cruise, v_ego, sm)

      self.slc_offset = 0
      self.slc_target = self.slc.target
    else:
      self.slc_offset = 0
      self.slc_target = 0

    targets = [self.csc_target, v_cruise]
    if frogpilot_toggles.speed_limit_controller:
      targets.append(max(self.slc.overridden_speed, self.slc_target + self.slc_offset) - v_ego_diff)
    v_cruise = min([target if target >= CRUISING_SPEED else v_cruise for target in targets])

    # Red lights and stop signs. Like the speed bumps below, it only ever lowers the cruise speed, and the deceleration it asks
    # for ("self.rlc.decel") goes out in frogpilotPlan.speedBumpDecel (see frogpilot_planner)
    stop_target, _ = self.rlc.update(long_control_active, stop_wanted and not self.override_force_stop, sm["modelV2"].position.x,
                                     sm["modelV2"].velocity.x, v_ego, self.speed_bump_config(frogpilot_toggles))
    if stop_target is not None:
      v_cruise = min(v_cruise, stop_target)

    self.forcing_stop = long_control_active and self.rlc.stopping
    self.tracked_model_length = max(self.rlc.stop_distance, 0.0) if self.rlc.stopping else self.frogpilot_planner.model_length

    # Speed bumps from mapd. Applied on its own rather than in "targets" above since the bump speed may be
    # set below "CRUISING_SPEED", and it only ever lowers the cruise speed, so it's safe on top of a force stop.
    # The deceleration it asks for ("self.sbc.decel") is published in frogpilotPlan.speedBumpDecel
    if frogpilot_toggles.speed_bump_slowdown:
      mapd_alive = sm.alive["mapdOut"] and sm.valid["mapdOut"]
      has_bump = mapd_alive and sm["mapdOut"].hasNextSpeedBump
      bump_distance = sm["mapdOut"].nextSpeedBumpDistance if mapd_alive else 0.0
      # mapd's distance is to the bump's middle. Raised tables can be tens of metres long, so brake for where it starts: the length
      # OpenStreetMap gives, else the user's table length for tables. Other bumps are short enough for the arrival margin to cover
      bump_length = sm["mapdOut"].nextSpeedBumpLength if mapd_alive else 0.0
      if has_bump and bump_length <= 0 and "table" in sm["mapdOut"].nextSpeedBumpType:
        bump_length = frogpilot_toggles.speed_bump_slowdown_table_length

      config = self.speed_bump_config(frogpilot_toggles)
      # How harsh this bump was felt to be on earlier drives, in this direction; -1 until mapd has learned it
      bump_severity = sm["mapdOut"].nextSpeedBumpSeverity if has_bump and sm["mapdOut"].nextSpeedBumpLearned else -1.0
      speed_bump_target, _ = self.sbc.update(long_control_active, has_bump, bump_distance, v_ego, config, length=bump_length,
                                             severity=bump_severity)
      if speed_bump_target is not None:
        v_cruise = min(v_cruise, speed_bump_target)

      # Roundabouts, with the same settings: slowed for by the give-way line at a speed from the ring's size, then let go
      # (see roundabout_controller). Its deceleration request is combined with the bump's in frogpilot_planner
      has_roundabout = mapd_alive and sm["mapdOut"].hasNextRoundabout
      entry_speed = roundabout_speed(sm["mapdOut"].nextRoundaboutDiameter) if has_roundabout else None
      roundabout_distance = sm["mapdOut"].nextRoundaboutDistance - ENTRY_OFFSET if has_roundabout else 0.0
      roundabout_target, _ = self.rbc.update(long_control_active, entry_speed is not None, roundabout_distance, v_ego,
                                             roundabout_config(config, entry_speed or config.v_target))
      if roundabout_target is not None:
        v_cruise = min(v_cruise, roundabout_target)
    else:
      for controller in (self.sbc, self.rbc):
        if controller.tracked_distance is not None or controller.target is not None:
          controller.reset()

    return v_cruise

  @staticmethod
  def speed_bump_config(frogpilot_toggles):
    return SpeedBumpConfig(
      v_target=frogpilot_toggles.speed_bump_slowdown_speed,
      brake_time=frogpilot_toggles.speed_bump_slowdown_time,
      max_decel=frogpilot_toggles.speed_bump_slowdown_max_decel,
      strict=frogpilot_toggles.speed_bump_slowdown_strict,
      response_lag=frogpilot_toggles.speed_bump_slowdown_response_time,
      margin=frogpilot_toggles.speed_bump_slowdown_margin,
      hold_distance=frogpilot_toggles.speed_bump_slowdown_hold,
      jerk_scale=frogpilot_toggles.speed_bump_slowdown_jerk,
      mild_extra=frogpilot_toggles.speed_bump_mild_extra_speed,
    )
