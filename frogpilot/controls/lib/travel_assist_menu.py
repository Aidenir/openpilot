#!/usr/bin/env python3
from opendbc.car import DT_CTRL, ButtonType, structs
from opendbc.car.common.conversions import Conversions as CV

# An onroad menu on the steering wheel. openpilot has no use for VW's Travel Assist button, so a press opens a cross on screen
# whose four arms are chosen with the cruise buttons: + (up), - (down), SET (left) and RES (right). Another Travel Assist press,
# or TIMEOUT without a cruise button, closes it.
#
# While it is open the cruise buttons belong to the menu: their events are taken out of carState before the set speed
# (VCruiseHelper) or engagement (buttonEnable, on SET / RES release) see them, and that holds for any button pressed while it was
# open until it is let go, even if the menu has closed by then, or letting go of SET would engage openpilot. Cancel, the brake
# and the gas are never touched, so openpilot disengages exactly as it always does. The panda still sees the car's own
# GRA_ACC_01 and allows controls on a SET / RES release, which only permits openpilot to engage, it doesn't engage it.

ARMS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.setCruise, ButtonType.resumeCruise)  # +, -, SET, RES

# What each arm does, by arm; None leaves it empty. "camera" steps the onroad view through CAMERA_MODES: openpilot's own choice,
# the wide road camera, the narrow one, and back (the UI does it, see AnnotatedCameraWidget). Its arm shows the mode it goes to.
# "speed_limit" makes the limit the car's sign recognition shows the map's limit for the road the car is on (mapd's
# user_speed_limits.json), for when OpenStreetMap hasn't caught up with a new sign. Its arm shows the reading it would use
ITEMS = ("refuge_island", "camera", "speed_limit", None)
LABELS = {"refuge_island": "MARK ISLAND"}

CAMERA_MODES = ("default", "wide", "narrow")  # frogpilotCarState.travelAssistMenu.cameraMode, by index
CAMERA_LABELS = {"default": "DEFAULT CAM", "wide": "WIDE CAM", "narrow": "NARROW CAM"}

TIMEOUT = 5.0        # s without a cruise button press before the menu closes on its own
SELECTED_TIME = 1.0  # s the chosen arm stays lit after the menu closes


class TravelAssistMenu:
  def __init__(self):
    self.open = False
    self.idle_time = 0.0
    self.selected = -1
    self.selected_time = 0.0
    self.travel_assist_prev = False
    self.down = set()      # cruise buttons held, whether the menu was open or not
    self.captured = set()  # cruise buttons pressed while the menu was open, until they are let go
    self.press_ms = {}     # wall-clock ms each captured button was pressed at
    self.choice_ms = 0     # when the arm chosen last was pressed: a mark is placed where the car was then, not at the release
    self.camera_mode = 0   # index into CAMERA_MODES, until card restarts (the next drive)
    self.sign_limit = 0.0  # m/s, the car's sign recognition (frogpilotCarState.signSpeedLimit), set by the caller; 0 if none

  def close(self):
    self.open = False
    self.idle_time = 0.0

  def update(self, CS, travel_assist_pressed, now_ms):
    """Takes the menu's cruise button events out of "CS" (a carState builder) and returns the item chosen this step, or None.
    "now_ms": wall-clock time in ms."""
    choice = None

    if travel_assist_pressed and not self.travel_assist_prev:
      if self.open:
        self.close()
      elif not self.down:
        # Not with a cruise button already held: its release would reach the set speed and engagement logic with no press before it
        self.open = True
        self.idle_time = 0.0
        self.selected = -1
    self.travel_assist_prev = travel_assist_pressed

    kept = []
    taken = False
    for be in CS.buttonEvents:
      if be.type in ARMS:
        if be.pressed:
          self.down.add(be.type)
        else:
          self.down.discard(be.type)

      if be.type in ARMS and (self.open or be.type in self.captured):
        taken = True
        if be.pressed:
          self.captured.add(be.type)
          self.press_ms[be.type] = now_ms
          self.idle_time = 0.0
        else:
          self.captured.discard(be.type)
          if self.open:
            arm = ARMS.index(be.type)
            if ITEMS[arm] is not None:
              choice = ITEMS[arm]
              self.choice_ms = self.press_ms.get(be.type, now_ms)
              if choice == "camera":
                self.camera_mode = (self.camera_mode + 1) % len(CAMERA_MODES)
              self.selected = arm
              self.selected_time = 0.0
              self.close()
        continue
      kept.append(be)

    if taken:
      CS.buttonEvents = [structs.CarState.ButtonEvent(pressed=be.pressed, type=be.type) for be in kept]
      # buttonEnable was worked out from the SET / RES release just taken out
      CS.buttonEnable = False
    elif self.open:
      CS.buttonEnable = False

    if self.open:
      self.idle_time += DT_CTRL
      if self.idle_time >= TIMEOUT:
        self.close()

    if self.selected >= 0:
      self.selected_time += DT_CTRL
      if self.selected_time >= SELECTED_TIME and not self.open:
        self.selected = -1

    return choice

  def publish(self, menu):
    """Fills a FrogPilotCarState.TravelAssistMenu builder"""
    menu.open = self.open
    menu.items = [self.label(item) for item in ITEMS]
    menu.selected = self.selected
    menu.cameraMode = self.camera_mode

  def label(self, item):
    if item == "speed_limit":
      return f"LIMIT {round(self.sign_limit * CV.MS_TO_KPH)}" if self.sign_limit > 0 else "NO SIGN"
    if item == "camera":
      # Open, the mode it would switch to; just chosen, the one it switched to
      mode = self.camera_mode if not self.open else (self.camera_mode + 1) % len(CAMERA_MODES)
      return CAMERA_LABELS[CAMERA_MODES[mode]]
    return LABELS.get(item, "") if item else ""
