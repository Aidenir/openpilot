#include "selfdrive/ui/qt/onroad/alerts.h"

#include <QFontMetrics>
#include <QPainter>
#include <algorithm>
#include <map>

#include "selfdrive/ui/qt/util.h"

// FrogPilot variables
// A single line of alert text drawn in the largest size up to "pixel_size" that fits "max_width", so a long
// message (mapd's speed limit corrections) stays inside the box instead of running off both sides
static void drawFittedText(QPainter &p, const QRect &rect, int flags, const QString &text, int pixel_size, QFont::Weight weight, int max_width) {
  const int min_size = pixel_size / 2;
  QFont font = InterFont(pixel_size, weight);
  int width = QFontMetrics(font).horizontalAdvance(text);
  if (width > max_width) {
    font.setPixelSize(std::max(min_size, pixel_size * max_width / width));
    while (font.pixelSize() > min_size && QFontMetrics(font).horizontalAdvance(text) > max_width) {
      font.setPixelSize(font.pixelSize() - 2);
    }
  }
  p.setFont(font);
  p.drawText(rect, flags, QFontMetrics(font).elidedText(text, Qt::ElideRight, max_width));
}

void OnroadAlerts::updateState(const UIState &s, const FrogPilotUIState &fs) {
  Alert a = getAlert(*(s.sm), *(fs.sm), s.scene.started_frame);
  if (!alert.equal(a)) {
    if (alert.status == cereal::SelfdriveState::AlertStatus::NORMAL && frogpilot_toggles.value("hide_alerts").toBool()) {
      clear();
    } else {
      alert = a;
      update();
    }
  }

  // FrogPilot variables
  sidebarsOpen = fs.frogpilot_scene.sidebars_open;
}

void OnroadAlerts::clear() {
  alert = {};
  update();

  // FrogPilot variables
  alertHeight = 0;
}

OnroadAlerts::Alert OnroadAlerts::getAlert(const SubMaster &sm, const SubMaster &fpsm, uint64_t started_frame) {
  const cereal::SelfdriveState::Reader &ss = sm["selfdriveState"].getSelfdriveState();
  const uint64_t selfdrive_frame = sm.rcv_frame("selfdriveState");

  // FrogPilot variables
  const cereal::FrogPilotSelfdriveState::Reader &fpss = fpsm["frogpilotSelfdriveState"].getFrogpilotSelfdriveState();

  Alert a = {};
  static QString crash_log_path = "/data/error_logs/error.txt";
  if (QFile::exists(crash_log_path)) {
    if (frogpilot_toggles.value("random_events").toBool()) {
      a = {tr("openpilot crashed 💩"),
           tr("Please post the \"Error Log\" in the FrogPilot Discord!"),
           "openpilotCrashedRandomEvent",
           cereal::SelfdriveState::AlertSize::MID,
           cereal::SelfdriveState::AlertStatus::CRITICAL};
    } else {
      a = {tr("openpilot crashed"),
           tr("Please post the \"Error Log\" in the FrogPilot Discord!"),
           "openpilotCrashed",
           cereal::SelfdriveState::AlertSize::MID,
           cereal::SelfdriveState::AlertStatus::CRITICAL};
    }
    return a;
  } else if (selfdrive_frame >= started_frame) {  // Don't get old alert.
    a = {ss.getAlertText1().cStr(), ss.getAlertText2().cStr(),
         ss.getAlertType().cStr(), ss.getAlertSize(), ss.getAlertStatus()};

    // FrogPilot variables
    if (a.size == cereal::SelfdriveState::AlertSize::NONE) {
      a = {fpss.getAlertText1().cStr(), fpss.getAlertText2().cStr(),
           fpss.getAlertType().cStr(), static_cast<cereal::SelfdriveState::AlertSize>(fpss.getAlertSize()), static_cast<cereal::SelfdriveState::AlertStatus>(fpss.getAlertStatus())};
    }
  }

  if (!sm.updated("selfdriveState") && (sm.frame - started_frame) > 5 * UI_FREQ && !frogpilot_toggles.value("force_onroad").toBool()) {
    const int SELFDRIVE_STATE_TIMEOUT = 5;
    const int ss_missing = (nanos_since_boot() - sm.rcv_time("selfdriveState")) / 1e9;

    // Handle selfdrive timeout
    if (selfdrive_frame < started_frame) {
      // car is started, but selfdriveState hasn't been seen at all
      a = {tr("openpilot Unavailable"), tr("Waiting to start"),
           "selfdriveWaiting", cereal::SelfdriveState::AlertSize::MID,
           cereal::SelfdriveState::AlertStatus::NORMAL};
    } else if (ss_missing > SELFDRIVE_STATE_TIMEOUT && !Hardware::PC()) {
      // car is started, but selfdrive is lagging or died
      if (ss.getEnabled() && (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10) {
        a = {tr("TAKE CONTROL IMMEDIATELY"), tr("System Unresponsive"),
             "selfdriveUnresponsive", cereal::SelfdriveState::AlertSize::FULL,
             cereal::SelfdriveState::AlertStatus::CRITICAL};
      } else {
        a = {tr("System Unresponsive"), tr("Reboot Device"),
             "selfdriveUnresponsivePermanent", cereal::SelfdriveState::AlertSize::MID,
             cereal::SelfdriveState::AlertStatus::NORMAL};
      }
    }
  }
  return a;
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::SelfdriveState::AlertSize::NONE) {
    // FrogPilot variables
    alertHeight = 0;
    return;
  }
  static std::map<cereal::SelfdriveState::AlertSize, const int> alert_heights = {
    {cereal::SelfdriveState::AlertSize::SMALL, 271},
    {cereal::SelfdriveState::AlertSize::MID, 420},
    {cereal::SelfdriveState::AlertSize::FULL, height()},
  };
  // FrogPilot variables
  alertHeight = alert_heights[alert.size];
  int h = alertHeight;

  int margin = 40;
  int radius = 30;
  if (alert.size == cereal::SelfdriveState::AlertSize::FULL) {
    margin = 0;
    radius = 0;
  }
  // FrogPilot variables
  alertHeight -= margin;
  QRect r = QRect(0 + margin, height() - h + margin, width() - margin*2, h - margin*2);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  p.setBrush(QBrush(frogpilot_alert_colors[static_cast<cereal::FrogPilotSelfdriveState::AlertStatus>(alert.status)]));
  p.drawRoundedRect(r, radius, radius);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.drawRoundedRect(r, radius, radius);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  const int text_width = r.width() - 2 * 40;
  if (alert.size == cereal::SelfdriveState::AlertSize::SMALL) {
    bool long_alert1 = alert.text1.length() > 40;
    drawFittedText(p, r, Qt::AlignCenter, alert.text1, long_alert1 && sidebarsOpen ? 64 : 74, QFont::DemiBold, text_width);
  } else if (alert.size == cereal::SelfdriveState::AlertSize::MID) {
    bool long_alert1 = alert.text1.length() > 30;
    drawFittedText(p, QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1, long_alert1 && sidebarsOpen ? 78 : 88, QFont::Bold, text_width);
    bool long_alert2 = alert.text2.length() > 40;
    drawFittedText(p, QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2, long_alert2 && sidebarsOpen ? 56 : 66, QFont::Normal, text_width);
  } else if (alert.size == cereal::SelfdriveState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    p.setFont(InterFont(l ? 132 : 177, QFont::Bold));
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    p.setFont(InterFont(88));
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}
