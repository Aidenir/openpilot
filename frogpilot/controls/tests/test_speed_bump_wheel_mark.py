from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.frogpilot.common.frogpilot_variables import BUTTON_FUNCTIONS
from openpilot.frogpilot.controls.frogpilot_card import FrogPilotCard


def toggles(key, function, one_way=False):
  t = SimpleNamespace(speed_bump_mark_one_way=one_way)
  for k in ("distance", "distance_long", "distance_very_long", "lkas"):
    for name in ("experimental_mode", "force_coast", "pause_lateral", "pause_longitudinal", "traffic_mode", "mark_speed_bump", "undo_speed_bump"):
      setattr(t, f"{name}_via_{k}", False)
  setattr(t, f"{function}_via_{key}", True)
  return t


def card():
  c = FrogPilotCard.__new__(FrogPilotCard)  # only what handle_button_event needs
  c.params_memory = Params(memory=True)
  c.speed_bump_request_id = 0
  return c


def test_distance_button_marks_and_undoes_a_bump():
  assert BUTTON_FUNCTIONS["MARK_SPEED_BUMP"] == 7 and BUTTON_FUNCTIONS["UNDO_SPEED_BUMP"] == 8
  c = card()
  sm = {"carControl": SimpleNamespace(longActive=True)}
  c.handle_button_event("distance", sm, toggles("distance", "mark_speed_bump", one_way=True))
  request = Params(memory=True).get("UserSpeedBumpRequest")
  assert request["action"] == "mark" and request["source"] == "wheel" and request["oneWay"] is True
  first = request["id"]
  c.handle_button_event("distance_long", sm, toggles("distance_long", "undo_speed_bump"))
  request = Params(memory=True).get("UserSpeedBumpRequest")
  assert request["action"] == "undo" and request["id"] > first
