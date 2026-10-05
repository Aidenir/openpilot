from types import SimpleNamespace

from openpilot.frogpilot.controls.frogpilot_card import FrogPilotCard


class FakeParams:
  def __init__(self):
    self.sent = []

  def put(self, key, value):
    self.sent.append((key, value))


def card():
  # Only what the speed limit correction uses
  c = SimpleNamespace(params_memory=FakeParams(), speed_limit_request_id=0, sign_limit=0.0, sign_limit_since_ms=0, mapping_limit_id=None)
  c.request_speed_limit = lambda tap_ms: FrogPilotCard.request_speed_limit(c, tap_ms)
  c.track_sign_limit = lambda limit, now_ms: FrogPilotCard.track_sign_limit(c, limit, now_ms)
  return c


def test_set_starts_at_the_sign_and_the_next_sign_ends_it():
  c = card()
  c.track_sign_limit(30 / 3.6, 1000)  # the car passes a 30 sign
  c.track_sign_limit(30 / 3.6, 5000)  # same reading: nothing changes
  c.request_speed_limit(9000)         # the driver takes it from the menu 8 s later
  key, req = c.params_memory.sent[-1]
  assert key == "UserSpeedLimitRequest" and req["action"] == "set"
  assert req["signMs"] == 1000 and req["tapMs"] == 9000 and abs(req["speedMs"] - 30 / 3.6) < 1e-6
  set_id = req["id"]

  c.track_sign_limit(50 / 3.6, 20000)  # the next sign
  key, req = c.params_memory.sent[-1]
  assert req["action"] == "end" and req["setId"] == set_id and req["endMs"] == 20000 and req["id"] > set_id
  assert c.mapping_limit_id is None

  n = len(c.params_memory.sent)
  c.track_sign_limit(30 / 3.6, 30000)  # later changes end nothing
  assert len(c.params_memory.sent) == n


def test_without_a_reading_nothing_is_recorded():
  c = card()
  c.request_speed_limit(9000)
  _, req = c.params_memory.sent[-1]
  assert req["speedMs"] == 0.0  # mapd answers "No speed limit sign read"
  c.track_sign_limit(30 / 3.6, 10000)
  assert len(c.params_memory.sent) == 1  # no end for a correction that never started
