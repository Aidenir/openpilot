from openpilot.common.params import Params
from openpilot.selfdrive.ui.feedback.feedbackd import ReportRequests


def test_report_requests_round_trip():
  # The real memory param, as frogpilot_card writes it: a JSON dict
  params = Params(memory=True)
  params.remove("UserReportRequest")
  reports = ReportRequests(params)
  assert not reports.new(now=1.0)  # nothing written yet

  params.put("UserReportRequest", {"id": 1_790_000_000_000, "tapMs": 1_790_000_000_000})
  assert params.get("UserReportRequest") == {"id": 1_790_000_000_000, "tapMs": 1_790_000_000_000}
  assert reports.new(now=2.0)
  assert not reports.new(now=3.0)  # the same request is sent once

  params.put("UserReportRequest", {"id": 1_790_000_000_001, "tapMs": 1_790_000_000_001})
  assert not reports.new(now=3.1)  # polled at most every 0.2 s
  assert reports.new(now=3.3)


def test_request_from_before_start_is_not_sent():
  params = Params(memory=True)
  params.put("UserReportRequest", {"id": 5, "tapMs": 5})
  assert not ReportRequests(params).new(now=1.0)
