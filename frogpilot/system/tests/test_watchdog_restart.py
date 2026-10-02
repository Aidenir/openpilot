import time
from types import SimpleNamespace

import openpilot.system.manager.process as process
from openpilot.system.manager.process import NativeProcess


def frozen_ui(monkeypatch):
  p = NativeProcess("ui", "selfdrive/ui", ["./ui"], lambda *a: True, watchdog_max_dt=5)
  p.proc = SimpleNamespace(pid=12345, exitcode=None)
  p.watchdog_seen = True
  p.last_watchdog_time = 0
  events = []
  monkeypatch.setattr(p, "restart", lambda: events.append(("restart", time.monotonic())))
  def slow_dump(dt, pid=None, exitcode=None, gdb=True):
    time.sleep(0.5)
    events.append(("dump", pid, gdb))
  monkeypatch.setattr(p, "dump_watchdog_diagnostics", slow_dump)
  monkeypatch.setattr(process, "ENABLE_WATCHDOG", True)
  return p, events


def test_onroad_restarts_at_once_and_dumps_in_the_background(monkeypatch):
  # Inline, the dump held the manager loop for ~8 s on 2026-10-02 and selfdrived raised commIssue (managerState not alive)
  p, events = frozen_ui(monkeypatch)
  t = time.monotonic()
  p.check_watchdog(started=True)
  assert time.monotonic() - t < 0.2
  assert events[0][0] == "restart"
  time.sleep(0.8)
  assert ("dump", 12345, False) in events


def test_offroad_keeps_the_full_dump_first(monkeypatch):
  p, events = frozen_ui(monkeypatch)
  p.check_watchdog(started=False)
  assert [e[0] for e in events] == ["dump", "restart"]
  assert events[0][2] is True
