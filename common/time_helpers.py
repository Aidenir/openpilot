import datetime
from pathlib import Path

MIN_DATE = datetime.datetime(year=2025, month=2, day=21)

def min_date():
  # on systemd systems, the default time is the systemd build time
  systemd_path = Path("/lib/systemd/systemd")
  if systemd_path.exists():
    d = datetime.datetime.fromtimestamp(systemd_path.stat().st_mtime)
    return max(MIN_DATE, d + datetime.timedelta(days=1))
  return MIN_DATE

def system_time_valid():
  return datetime.datetime.now() > min_date()


# FrogPilot: the device has no working RTC, so at boot the clock is the last time systemd-timesyncd saved, stale by however long
# the device was off. That passes system_time_valid(), and handed to the modem as qcomgpsd's GPS time hint it kept it from ever
# getting a fix (2026-10-02, the clock 2 h out). Trusted means set this boot from NTP, or from GPS by timed (which writes
# GPS_TIME_MARKER)
NTP_SYNCED_MARKER = Path("/run/systemd/timesync/synchronized")
GPS_TIME_MARKER = Path("/dev/shm/time_from_gps")

def system_time_trusted():
  return system_time_valid() and (NTP_SYNCED_MARKER.exists() or GPS_TIME_MARKER.exists())
