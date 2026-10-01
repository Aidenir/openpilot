#!/usr/bin/env python3
"""Suggest unmapped speed bumps from the device IMU.

A speed bump pitches the car nose-up then nose-down in well under a second,
which shows up clearly in the gyroscope's pitch rate (device axis 1) once slow
changes (hills, braking dive) are removed. On 50 minutes of recorded drives in
Malmö the high-passed |pitch rate| peaked at a median 0.19 rad/s crossing mapped
bumps against a background median of 0.03.

Detections are only *suggestions*: they go to mapd through the
SpeedBumpSuggestRequest memory param, and mapd clusters them in
suggested_speed_bumps.json. A location only becomes an active bump (warned for
and slowed down for) once it has been seen on SpeedBumpDetectPromoteDrives
separate drives.

SpeedBumpDetector is a pure class with no messaging, so the same code runs here,
in the unit tests and in tools/detect_bumps_from_rlogs.py over old drives.
"""
from collections import deque
from dataclasses import dataclass

import numpy as np

# The high-pass subtracts a centred moving average over this many samples
# (1 s at the sensors' ~104 Hz). Centring needs half a window of look-ahead, so
# the filtered signal lags the raw one by HP_WINDOW // 2 samples (~0.5 s). The
# event timestamp is the sample's own, so the lag does not move the bump.
HP_WINDOW = 104

DEFAULT_THRESHOLD = 0.15  # rad/s of high-passed pitch rate
MIN_SPEED = 1.5  # m/s; below this, parking manoeuvres and kerbs
PEAK_WINDOW = 1.5  # s after the first crossing to look for the peak
REFRACTORY = 2.5  # s after a trigger before another can start
AZ_PP_BEFORE = 1.0  # s before the trigger included in the vertical accel peak-to-peak
AZ_HISTORY = 4.0  # s of filtered vertical accel kept for that


@dataclass
class BumpEvent:
  t: float  # sensor time of the pitch peak (s, time.monotonic clock)
  pitch: float  # peak |high-passed pitch rate|, rad/s
  az_pp: float  # peak-to-peak high-passed vertical accel around the event, m/s^2
  v_ego: float  # speed when it triggered, m/s


class CentredHighPass:
  """x[i] - mean(x[i - n/2 : i + n/2]), emitted n/2 samples late."""

  def __init__(self, n=HP_WINDOW):
    self.n = n
    self.buf = deque()
    self.total = 0.0

  def add(self, t, x):
    self.buf.append((t, x))
    self.total += x
    if len(self.buf) > self.n:
      self.total -= self.buf.popleft()[1]
    if len(self.buf) < self.n:
      return None
    tc, xc = self.buf[self.n // 2]
    return tc, xc - self.total / self.n


class SpeedBumpDetector:
  def __init__(self, threshold=DEFAULT_THRESHOLD, min_speed=MIN_SPEED, az_pp_floor=0.0):
    self.threshold = threshold
    self.min_speed = min_speed
    self.az_pp_floor = az_pp_floor

    self.pitch_hp = CentredHighPass()
    self.az_hp = CentredHighPass()
    self.az_recent = deque()

    self.v_ego = 0.0
    self.refractory_until = float("-inf")
    self.open = None  # [trigger time, peak time, peak value, speed at trigger]

  def set_speed(self, v_ego):
    self.v_ego = v_ego

  def add_accel(self, t, vertical):
    out = self.az_hp.add(t, vertical)
    if out is not None:
      self.az_recent.append(out)
      while self.az_recent and self.az_recent[0][0] < out[0] - AZ_HISTORY:
        self.az_recent.popleft()

  def add_gyro(self, t, pitch_rate):
    """Feed one raw pitch rate sample. Returns a BumpEvent when one completes."""
    out = self.pitch_hp.add(t, pitch_rate)
    if out is None:
      return None
    tc, hp = out
    mag = abs(hp)

    if self.open is not None:
      if mag > self.open[2]:
        self.open[1], self.open[2] = tc, mag
      if tc - self.open[0] >= PEAK_WINDOW:
        return self._close()
      return None

    if mag > self.threshold and self.v_ego > self.min_speed and tc >= self.refractory_until:
      self.open = [tc, tc, mag, self.v_ego]
      self.refractory_until = tc + REFRACTORY
    return None

  def _close(self):
    t0, tp, peak, v = self.open
    self.open = None
    window = [a for ta, a in self.az_recent if t0 - AZ_PP_BEFORE <= ta <= t0 + PEAK_WINDOW]
    az_pp = (max(window) - min(window)) if window else 0.0
    if az_pp < self.az_pp_floor:
      return None
    return BumpEvent(t=tp, pitch=peak, az_pp=az_pp, v_ego=v)


REFINE_WINDOW = 10.0  # s either side of a mark's tap to look for the bump in
REFINE_THRESHOLD = 0.09  # rad/s; lower than the detector's own, since the driver has already said there is a bump here
REFINE_BUFFER = 30.0  # s of raw pitch rate kept for that


def find_bump_near(samples, tap_t, window=REFINE_WINDOW, threshold=REFINE_THRESHOLD, min_speed=MIN_SPEED):
  """The time of the strongest jolt within "window" seconds of "tap_t", or None.

  "samples" are (t, pitch_rate, v_ego) in time order. The pitch rate is high-passed the same way as for detection (a
  centred moving average subtracted), and only samples while moving count. The bump is the strongest peak, not the one
  nearest the tap: a driver reaches for the button before or after a bump, rarely on it."""
  if len(samples) < HP_WINDOW:
    return None
  t = np.array([s[0] for s in samples])
  pitch = np.array([s[1] for s in samples])
  v = np.array([s[2] for s in samples])
  hp = np.abs(pitch - np.convolve(pitch, np.ones(HP_WINDOW) / HP_WINDOW, mode="same"))
  # The moving average is only valid away from the buffer's ends
  valid = np.zeros(len(t), dtype=bool)
  valid[HP_WINDOW // 2:len(t) - HP_WINDOW // 2] = True
  candidates = valid & (np.abs(t - tap_t) <= window) & (v > min_speed) & (hp >= threshold)
  if not candidates.any():
    return None
  i = int(np.argmax(np.where(candidates, hp, -1.0)))
  return float(t[i]), float(hp[i])


def main():
  import time

  import cereal.messaging as messaging
  from openpilot.common.params import Params
  from openpilot.common.realtime import Ratekeeper

  params = Params(return_defaults=True)
  params_memory = Params(memory=True)

  gyro_sock = messaging.sub_sock("gyroscope", conflate=False)
  accel_sock = messaging.sub_sock("accelerometer", conflate=False)
  # Speed comes from livePose, not carState: carState already has all 15 of its msgq reader slots taken onroad, and
  # a 16th reader makes msgq evict every reader over and over. calibrationd then keeps missing carState, so
  # liveCalibration and everything downstream go invalid and openpilot refuses to engage (commIssue)
  sm = messaging.SubMaster(["livePose"])

  def settings():
    threshold = min(max(float(params.get("SpeedBumpDetectThreshold")), 0.08), 0.4)
    return threshold, int(params.get("SpeedBumpDetectPromoteDrives"))

  threshold, promote_drives = settings()
  detect = params.get_bool("SpeedBumpDetect")
  detector = SpeedBumpDetector(threshold=threshold)
  last_settings_check = time.monotonic()
  last_id = 0

  # Marks to refine: (mark id, tap time on the monotonic clock), searched once REFINE_WINDOW has passed after the tap
  recent = deque()  # (t, pitch rate, v_ego) for the last REFINE_BUFFER seconds
  pending_marks = deque()
  last_mark_id = None
  last_mark_check = 0.0
  refine_id = 0

  # 20 Hz is plenty: drain_sock hands over every sample queued since the last
  # loop, so the detector still sees all ~104 Hz of both sensors.
  rk = Ratekeeper(20, print_delay_threshold=None)
  while True:
    sm.update(0)
    detector.set_speed(sm["livePose"].velocityDevice.x)  # device x is forward

    for msg in messaging.drain_sock(accel_sock):
      a = msg.accelerometer
      if a.which() == "acceleration":
        detector.add_accel(a.timestamp * 1e-9, a.acceleration.v[0])  # device x is vertical

    for msg in messaging.drain_sock(gyro_sock):
      g = msg.gyroscope
      if g.which() != "gyroUncalibrated":
        continue
      t_sample = g.timestamp * 1e-9
      recent.append((t_sample, g.gyroUncalibrated.v[1], detector.v_ego))
      event = detector.add_gyro(t_sample, g.gyroUncalibrated.v[1])  # device y is pitch
      if event is None or not detect:
        continue
      # The sample clock is time.monotonic (sensord converts to it), so the
      # event's age turns straight into a wall-clock time mapd can back-project from.
      event_ms = int(time.time() * 1000 - (time.monotonic() - event.t) * 1000)  # noqa: TID251
      last_id = max(event_ms, last_id + 1)
      params_memory.put("SpeedBumpSuggestRequest", {
        "id": last_id,
        "eventMs": event_ms,
        "pitch": round(event.pitch, 4),
        "azpp": round(event.az_pp, 3),
        "vEgo": round(event.v_ego, 2),
        "promoteDrives": promote_drives,
      })

    now = time.monotonic()
    while recent and recent[0][0] < now - REFINE_BUFFER:
      recent.popleft()

    # A new mark (onroad button or steering wheel): remember it and refine it from the IMU once both sides of the tap are in
    if now - last_mark_check > 0.25:
      last_mark_check = now
      request = params_memory.get("UserSpeedBumpRequest") or {}
      mark_id = request.get("id")
      if mark_id is not None and mark_id != last_mark_id:
        if last_mark_id is not None and request.get("action") == "mark":
          tap_mono = now - (time.time() * 1000 - request.get("tapMs", time.time() * 1000)) / 1000  # noqa: TID251
          pending_marks.append((mark_id, tap_mono))
        last_mark_id = mark_id

    while pending_marks and now >= pending_marks[0][1] + REFINE_WINDOW + 0.5:
      mark_id, tap_mono = pending_marks.popleft()
      found = find_bump_near(list(recent), tap_mono)
      if found is None:
        continue  # no clear jolt either side of the tap: the mark stays where it was tapped
      event_t, pitch = found
      refine_id = max(int(time.time() * 1000), refine_id + 1)  # noqa: TID251
      params_memory.put("SpeedBumpRefineRequest", {
        "id": refine_id,
        "markId": mark_id,
        "eventMs": int(time.time() * 1000 - (now - event_t) * 1000),  # noqa: TID251
        "pitch": round(pitch, 4),
      })

    if now - last_settings_check > 5:
      last_settings_check = now
      threshold, promote_drives = settings()
      detector.threshold = threshold
      detect = params.get_bool("SpeedBumpDetect")

    rk.keep_time()


if __name__ == "__main__":
  main()
