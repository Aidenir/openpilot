#!/usr/bin/env python3
import time

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from cereal import car
from openpilot.system.micd import SAMPLE_RATE, SAMPLE_BUFFER

FEEDBACK_MAX_DURATION = 10.0
ButtonType = car.CarState.ButtonEvent.Type


class ReportRequests:
  """The Travel Assist menu's REPORT arm (frogpilot_card) writes UserReportRequest, a memory param, with a new id each time: this
  process publishes userBookmark, so it sends the bookmark. Requests made before it started are not sent."""
  POLL_INTERVAL = 0.2  # s; the loop runs at carState's 100 Hz

  def __init__(self, params_memory):
    self.params_memory = params_memory
    self.last_id = self.read_id()
    self.last_poll = 0.0

  def read_id(self):
    request = self.params_memory.get("UserReportRequest")
    return request.get("id") if isinstance(request, dict) else None

  def new(self, now):
    if now - self.last_poll < self.POLL_INTERVAL:
      return False
    self.last_poll = now
    request_id = self.read_id()
    if request_id is None or request_id == self.last_id:
      return False
    self.last_id = request_id
    return True


def main():
  params = Params()
  reports = ReportRequests(Params(memory=True))
  pm = messaging.PubMaster(['userBookmark', 'audioFeedback'])
  sm = messaging.SubMaster(['rawAudioData', 'bookmarkButton', 'carState'])
  should_record_audio = False
  block_num = 0
  waiting_for_release = False
  early_stop_triggered = False

  while True:
    sm.update()
    should_send_bookmark = False

    # TODO: https://github.com/commaai/openpilot/issues/36015
    if False and sm.updated['carState'] and sm['carState'].canValid:
      for be in sm['carState'].buttonEvents:
        if be.type == ButtonType.lkas:
          if be.pressed:
            if not should_record_audio:
              if params.get_bool("RecordAudioFeedback"):  # Start recording on first press if toggle set
                should_record_audio = True
                block_num = 0
                waiting_for_release = False
                early_stop_triggered = False
                cloudlog.info("LKAS button pressed - starting 10-second audio feedback")
              else:
                should_send_bookmark = True  # immediately send bookmark if toggle false
                cloudlog.info("LKAS button pressed - bookmarking")
            elif should_record_audio and not waiting_for_release:  # Wait for release of second press to stop recording early
              waiting_for_release = True
          elif waiting_for_release:  # Second press released
            waiting_for_release = False
            early_stop_triggered = True
            cloudlog.info("LKAS button released - ending recording early")

    if should_record_audio and sm.updated['rawAudioData']:
      raw_audio = sm['rawAudioData']
      msg = messaging.new_message('audioFeedback', valid=True)
      msg.audioFeedback.audio.data = raw_audio.data
      msg.audioFeedback.audio.sampleRate = raw_audio.sampleRate
      msg.audioFeedback.blockNum = block_num
      block_num += 1
      if (block_num * SAMPLE_BUFFER / SAMPLE_RATE) >= FEEDBACK_MAX_DURATION or early_stop_triggered:  # Check for timeout or early stop
        should_send_bookmark = True  # send bookmark at end of audio segment
        should_record_audio = False
        early_stop_triggered = False
        cloudlog.info("10-second recording completed or second button press - stopping audio feedback")
      pm.send('audioFeedback', msg)

    if sm.updated['bookmarkButton']:
      cloudlog.info("Bookmark button pressed!")
      should_send_bookmark = True

    if reports.new(time.monotonic()):
      cloudlog.info("Report from the Travel Assist menu")
      should_send_bookmark = True

    if should_send_bookmark:
      msg = messaging.new_message('userBookmark', valid=True)
      pm.send('userBookmark', msg)


if __name__ == '__main__':
  main()
