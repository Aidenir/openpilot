import datetime
import os
import sys
import time
import types
from types import SimpleNamespace

import pytest

# timed imports timezonefinder, which the device has but a PC may not
sys.modules.setdefault("timezonefinder", types.SimpleNamespace(TimezoneFinder=object))

import openpilot.system.timed as timed
from openpilot.frogpilot.controls.lib import gps_status as gs
from openpilot.system.qcomgpsd.qcomgpsd import GPS_EPOCH, GPS_LEAP_SECONDS, modem_time_error

gps_status = gs.gps_status
GPS_STATUS_GOOD, GPS_STATUS_NO_GPS, GPS_STATUS_SEARCHING, GPS_STATUS_WEAK = gs.GPS_STATUS_GOOD, gs.GPS_STATUS_NO_GPS, \
                                                                            gs.GPS_STATUS_SEARCHING, gs.GPS_STATUS_WEAK


@pytest.fixture
def stockholm():
  # FrogPilot sets the timezone from the GPS position, which put the clock 2 h out (2026-10-02)
  old = os.environ.get("TZ")
  os.environ["TZ"] = "Europe/Stockholm"
  time.tzset()
  yield
  if old is None:
    del os.environ["TZ"]
  else:
    os.environ["TZ"] = old
  time.tzset()


class TestTimed:
  def test_sets_utc_with_a_local_timezone(self, stockholm, monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(timed.subprocess, "run", lambda cmd, **kw: calls.append(cmd))
    monkeypatch.setattr(timed, "GPS_TIME_MARKER", tmp_path / "marker")
    gps_ms = (datetime.datetime.now(datetime.UTC) - datetime.timedelta(hours=1)).timestamp() * 1e3
    gps_time = datetime.datetime.fromtimestamp(gps_ms / 1e3, datetime.UTC).replace(tzinfo=None)
    timed.set_time(gps_time)
    assert len(calls) == 1
    assert calls[0] == f"TZ=UTC date -s '{gps_time}'"
    assert (tmp_path / "marker").exists()

  def test_a_right_clock_is_left_alone_but_trusted(self, stockholm, monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(timed.subprocess, "run", lambda cmd, **kw: calls.append(cmd))
    monkeypatch.setattr(timed, "GPS_TIME_MARKER", tmp_path / "marker")
    timed.set_time(datetime.datetime.now(datetime.UTC).replace(tzinfo=None))
    assert calls == []
    assert (tmp_path / "marker").exists()


class TestModemTime:
  def modem_time(self, offset_s):
    t = datetime.datetime.now(datetime.UTC) + datetime.timedelta(seconds=offset_s + GPS_LEAP_SECONDS) - GPS_EPOCH
    week, rest = divmod(t, datetime.timedelta(weeks=1))
    return week, int(rest.total_seconds() * 1000)

  @pytest.mark.parametrize("offset", [0.0, 7200.0, -10800.0])
  def test_error(self, offset):
    assert modem_time_error(*self.modem_time(offset)) == pytest.approx(offset, abs=1.0)

  def test_unknown_time(self):
    assert modem_time_error(0, 12345) is None


def loc(fix, hacc=0.0, vacc=1.1):
  return SimpleNamespace(hasFix=fix, horizontalAccuracy=hacc, verticalAccuracy=vacc)


class TestGpsStatus:
  def test_good_and_weak(self):
    assert gps_status(10.0, loc(True), 9.5, 9.9) == GPS_STATUS_GOOD
    assert gps_status(10.0, loc(True, vacc=4.0), 9.5, 9.9) == GPS_STATUS_WEAK
    assert gps_status(10.0, loc(True, hacc=30.0), 9.5, None) == GPS_STATUS_WEAK
    assert gps_status(10.0, loc(True, hacc=5.0, vacc=9.0), 9.5, None) == GPS_STATUS_GOOD

  def test_searching(self):
    # The modem sending measurements but no position, as on 2026-10-02 with its time 2 h out
    assert gps_status(10.0, loc(False), None, 9.9) == GPS_STATUS_SEARCHING
    assert gps_status(10.0, loc(False, vacc=500.0), 9.8, 9.9) == GPS_STATUS_SEARCHING
    # a stale fix doesn't count
    assert gps_status(10.0, loc(True), 5.0, 9.9) == GPS_STATUS_SEARCHING

  def test_no_gps(self):
    assert gps_status(10.0, loc(False), None, None) == GPS_STATUS_NO_GPS
    assert gps_status(10.0, loc(True), 2.0, 3.0) == GPS_STATUS_NO_GPS
