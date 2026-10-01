import re
from pathlib import Path

from openpilot.common.basedir import BASEDIR

BASE = Path(BASEDIR)

# Processes that never run in a car alongside the normal onroad stack
NOT_IN_CAR = {"joystickd", "maneuversd", "webjoystick", "joystick", "webrtcd", "webcamerad", "bridge", "stream_encoderd"}

# Native readers of carState: the Qt UI's SubMaster (selfdrive/ui/ui.cc) and loggerd, which subscribes to every service
NATIVE_CARSTATE_READERS = {"ui", "loggerd"}


def num_readers():
  return int(re.search(r"#define NUM_READERS (\d+)", (BASE / "msgq_repo/msgq/msgq.h").read_text()).group(1))


def python_processes():
  config = (BASE / "system/manager/process_config.py").read_text()
  return re.findall(r'PythonProcess\("(\w+)", "([\w.]+)"', config)


def carstate_readers(source):
  subs = sum(1 for m in re.finditer(r"SubMaster\(\s*\[(.*?)\]", source, re.S) if re.search(r"['\"]carState['\"]", m.group(1)))
  return subs + len(re.findall(r"sub_sock\(\s*['\"]carState['\"]", source))


def test_carstate_readers_fit_in_msgq():
  # msgq gives each service NUM_READERS slots. One reader too many and msgq evicts every reader, again and again:
  # calibrationd keeps missing carState, liveCalibration goes invalid with everything downstream of it, and
  # openpilot refuses to engage with commIssue. That is what an extra carState SubMaster in speed_bump_detector did.
  readers = {}
  for name, module in python_processes():
    if name in NOT_IN_CAR or module == "selfdrive.ui.ui":  # the Python UI only replaces the Qt one off-device
      continue
    path = BASE / (module.replace(".", "/") + ".py")
    if path.exists() and (count := carstate_readers(path.read_text())):
      readers[name] = count
  for name in NATIVE_CARSTATE_READERS:
    readers[name] = 1

  total = sum(readers.values())
  assert total <= num_readers(), f"{total} carState readers onroad, msgq allows {num_readers()}: {sorted(readers)}"


def test_speed_bump_detector_does_not_read_carstate():
  source = (BASE / "frogpilot/system/speed_bump_detector.py").read_text()
  assert carstate_readers(source) == 0
