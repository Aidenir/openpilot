from cereal import custom
from opendbc.car import DT_CTRL, ButtonType, structs

from openpilot.frogpilot.controls.lib.travel_assist_menu import ITEMS, SELECTED_TIME, TIMEOUT, TravelAssistMenu

PLUS, MINUS, SET, RES = ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.setCruise, ButtonType.resumeCruise


def car_state(*events, button_enable=False):
  CS = structs.CarState.new_message()
  CS.buttonEvents = [structs.CarState.ButtonEvent(pressed=pressed, type=t) for t, pressed in events]
  CS.buttonEnable = button_enable
  return CS


class Driver:
  def __init__(self):
    self.menu = TravelAssistMenu()
    self.travel_assist = False
    self.now_ms = 1_000_000

  def step(self, *events, button_enable=False):
    CS = car_state(*events, button_enable=button_enable)
    choice = self.menu.update(CS, self.travel_assist, self.now_ms)
    self.now_ms += int(DT_CTRL * 1000)
    return CS, choice

  def press_travel_assist(self):
    self.travel_assist = True
    self.step()
    self.travel_assist = False
    self.step()

  def idle(self, seconds):
    for _ in range(round(seconds / DT_CTRL)):
      self.step()


def types(CS):
  return [(be.type, be.pressed) for be in CS.buttonEvents]


def test_closed_menu_leaves_buttons_alone():
  d = Driver()
  CS, choice = d.step((SET, False), button_enable=True)
  assert types(CS) == [(SET, False)] and CS.buttonEnable and choice is None


def test_plus_marks_an_island_and_takes_the_buttons():
  assert ITEMS[0] == "refuge_island"
  d = Driver()
  d.press_travel_assist()
  assert d.menu.open
  CS, choice = d.step((PLUS, True))
  assert types(CS) == [] and choice is None
  pressed_at = d.now_ms - int(DT_CTRL * 1000)
  d.idle(0.2)
  CS, choice = d.step((PLUS, False))
  assert types(CS) == [] and choice == "refuge_island"
  assert d.menu.choice_ms == pressed_at  # where the car was at the press
  assert not d.menu.open and d.menu.selected == 0


def test_set_and_res_never_engage_while_open():
  d = Driver()
  d.press_travel_assist()
  for button in (SET, RES):
    d.step((button, True))
    CS, choice = d.step((button, False), button_enable=True)
    assert not CS.buttonEnable and types(CS) == [] and choice is None
  assert d.menu.open  # empty arms do nothing


def test_a_release_after_the_menu_closed_is_still_taken():
  # SET pressed in the menu, the menu closed by another Travel Assist press before SET is let go: the release must not engage
  d = Driver()
  d.press_travel_assist()
  d.step((SET, True))
  d.press_travel_assist()
  assert not d.menu.open
  CS, _ = d.step((SET, False), button_enable=True)
  assert not CS.buttonEnable and types(CS) == []
  # and the next SET is the driver's again
  d.step((SET, True))
  CS, _ = d.step((SET, False), button_enable=True)
  assert CS.buttonEnable and types(CS) == [(SET, False)]


def test_does_not_open_with_a_cruise_button_held():
  d = Driver()
  d.step((PLUS, True))
  d.press_travel_assist()
  assert not d.menu.open
  CS, _ = d.step((PLUS, False))
  assert types(CS) == [(PLUS, False)]


def test_cancel_and_other_buttons_pass_through():
  d = Driver()
  d.press_travel_assist()
  CS, _ = d.step((ButtonType.cancel, True), (ButtonType.gapAdjustCruise, True), (PLUS, True))
  assert types(CS) == [(ButtonType.cancel, True), (ButtonType.gapAdjustCruise, True)]


def test_closes_after_the_timeout_and_on_a_second_press():
  d = Driver()
  d.press_travel_assist()
  d.idle(TIMEOUT - 0.1)
  assert d.menu.open
  d.idle(0.2)
  assert not d.menu.open
  d.press_travel_assist()
  d.press_travel_assist()
  assert not d.menu.open


def test_a_button_press_keeps_it_open():
  d = Driver()
  d.press_travel_assist()
  d.idle(TIMEOUT - 1.0)
  d.step((SET, True))
  d.step((SET, False))
  d.idle(TIMEOUT - 1.0)
  assert d.menu.open


def test_published_state():
  d = Driver()
  d.press_travel_assist()
  fpcs = custom.FrogPilotCarState.new_message()
  d.menu.publish(fpcs.travelAssistMenu)
  assert fpcs.travelAssistMenu.open and fpcs.travelAssistMenu.selected == -1
  assert list(fpcs.travelAssistMenu.items) == ["MARK ISLAND", "CAMERA", "", ""]
  d.step((PLUS, True))
  d.step((PLUS, False))
  d.menu.publish(fpcs.travelAssistMenu)
  assert not fpcs.travelAssistMenu.open and fpcs.travelAssistMenu.selected == 0
  d.idle(SELECTED_TIME + 0.1)
  d.menu.publish(fpcs.travelAssistMenu)
  assert fpcs.travelAssistMenu.selected == -1


def test_minus_switches_the_camera():
  assert ITEMS[1] == "camera"
  d = Driver()
  fpcs = custom.FrogPilotCarState.new_message()
  for switches in (1, 2):
    d.press_travel_assist()
    d.step((MINUS, True))
    CS, choice = d.step((MINUS, False))
    assert choice == "camera" and types(CS) == [] and not d.menu.open
    d.menu.publish(fpcs.travelAssistMenu)
    assert fpcs.travelAssistMenu.cameraSwitches == switches
  # - with the menu closed is the driver's set speed again
  CS, _ = d.step((MINUS, True))
  assert types(CS) == [(MINUS, True)]
