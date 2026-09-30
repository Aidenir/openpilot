#include "frogpilot/ui/qt/onroad/frogpilot_buttons.h"

#include <algorithm>

#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QTimer>

#include "selfdrive/ui/qt/util.h"

DrivingPersonalityButton::DrivingPersonalityButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size + UI_BORDER_SIZE, btn_size);

  QObject::connect(frogpilotUIState(), &FrogPilotUIState::themeUpdated, this, &DrivingPersonalityButton::updateTheme);
  QObject::connect(this, &QPushButton::pressed, [this] {params_memory.putBool("OnroadDistanceButtonPressed", true);});
  QObject::connect(this, &QPushButton::released, [this] {params_memory.putBool("OnroadDistanceButtonPressed", false);});
}

void DrivingPersonalityButton::showEvent(QShowEvent *event) {
  updateTheme();
}

void DrivingPersonalityButton::updateTheme() {
  for (QMap<int, QPair<QPixmap, QSharedPointer<QMovie>>>::iterator it = icon_map.begin(); it != icon_map.end(); ++it) {
    QSharedPointer<QMovie> &movie = it.value().second;
    if (!movie.isNull()) {
      QObject::disconnect(movie.data(), nullptr, this, nullptr);
      movie->stop();
    }
  }

  icon_map.clear();

  QPixmap traffic_img, aggressive_img, standard_img, relaxed_img;
  QSharedPointer<QMovie> traffic_gif, aggressive_gif, standard_gif, relaxed_gif;

  loadImage("../../frogpilot/assets/active_theme/distance_icons/traffic", traffic_img, traffic_gif, QSize(btn_size, btn_size), this);
  loadImage("../../frogpilot/assets/active_theme/distance_icons/aggressive", aggressive_img, aggressive_gif, QSize(btn_size, btn_size), this);
  loadImage("../../frogpilot/assets/active_theme/distance_icons/standard", standard_img, standard_gif, QSize(btn_size, btn_size), this);
  loadImage("../../frogpilot/assets/active_theme/distance_icons/relaxed", relaxed_img, relaxed_gif, QSize(btn_size, btn_size), this);

  icon_map.insert(0, qMakePair(traffic_img, traffic_gif));
  icon_map.insert(1, qMakePair(aggressive_img, aggressive_gif));
  icon_map.insert(2, qMakePair(standard_img, standard_gif));
  icon_map.insert(3, qMakePair(relaxed_img, relaxed_gif));

  theme_updated = true;
}

void DrivingPersonalityButton::updateState(const UIState &s, const FrogPilotUIState &fs) {
  const UIScene &scene = s.scene;

  const SubMaster &fpsm = *(fs.sm);

  const cereal::FrogPilotCarState::Reader &frogpilotCarState = fpsm["frogpilotCarState"].getFrogpilotCarState();

  bool new_traffic_mode_active = frogpilotCarState.getTrafficModeEnabled();

  int new_personality = static_cast<int>(scene.personality) + 1;

  bool state_changed = (traffic_mode_active != new_traffic_mode_active) ||
                       (personality != new_personality && !new_traffic_mode_active);

  if (!state_changed && !theme_updated) {
    return;
  }

  traffic_mode_active = new_traffic_mode_active;

  personality = new_personality;

  theme_updated = false;

  QPair<QPixmap, QSharedPointer<QMovie>> icon = icon_map.value(traffic_mode_active ? 0 : personality);
  currentImg = icon.first;
  currentGif = icon.second.data();
}

void DrivingPersonalityButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  drawIcon(p, rect().center() + QPoint(UI_BORDER_SIZE / 2, 0), currentGif ? currentGif->currentPixmap() : currentImg, Qt::transparent, 1.0);
}

// How long to wait for mapd's reply before saying it did not answer. mapd polls every
// 50 ms, so anything near this means it is not running.
static constexpr int MARK_REPLY_TIMEOUT_MS = 2000;
static const QColor MARK_COLOR(255, 200, 50);

SpeedBumpMarkButton::SpeedBumpMarkButton(QWidget *parent) : QPushButton(parent) {
  // Big enough to hit with a gloved thumb without looking for long
  setFixedSize(260, 180);

  holdTimer = new QTimer(this);
  holdTimer->setSingleShot(true);
  holdTimer->setInterval(UNDO_HOLD_MS);

  QObject::connect(holdTimer, &QTimer::timeout, [this] {
    holdFired = true;
    sendRequest("undo", QDateTime::currentMSecsSinceEpoch());
  });
  QObject::connect(this, &QPushButton::pressed, [this] {
    // The press, not the release, is the moment the driver is on the bump
    pressMs = QDateTime::currentMSecsSinceEpoch();
    holdFired = false;
    holdTimer->start();
    update();
  });
  QObject::connect(this, &QPushButton::released, [this] {
    holdTimer->stop();
    if (!holdFired) {
      sendRequest("mark", pressMs);
    }
    update();
  });
}

void SpeedBumpMarkButton::sendRequest(const QString &action, qint64 tapMs) {
  // Unique per request even for two taps in the same millisecond. Written by hand
  // rather than through QJsonDocument, which stores integers as doubles and could
  // print a 13 digit id in exponent form that mapd's integer parse rejects.
  qint64 id = std::max(tapMs, lastRequestId + 1);
  lastRequestId = id;

  QString request = QString("{\"id\":%1,\"action\":\"%2\",\"tapMs\":%3}").arg(id).arg(action).arg(tapMs);
  params_memory.put("UserSpeedBumpRequest", request.toStdString());

  pendingId = id;
  pendingAction = action;
  pendingSince = QDateTime::currentMSecsSinceEpoch();
  showFeedback(action == "undo" ? tr("UNDOING") : tr("MARKING"), QString(), MARK_COLOR, MARK_REPLY_TIMEOUT_MS + 500);
}

void SpeedBumpMarkButton::showFeedback(const QString &title, const QString &detail, const QColor &color, int durationMs) {
  feedbackTitle = title;
  feedbackDetail = detail;
  feedbackColor = color;
  feedbackUntil = QDateTime::currentMSecsSinceEpoch() + durationMs;
  update();
}

void SpeedBumpMarkButton::updateState(bool showSuggestions) {
  qint64 now = QDateTime::currentMSecsSinceEpoch();

  // mapd's verdict on each IMU detection. Polled at ~4 Hz; a toast a quarter of a
  // second late does not matter, and a tap to confirm has 10 s.
  if (showSuggestions && pendingId == 0 && ++suggestionPollCounter >= 5) {
    suggestionPollCounter = 0;
    QJsonObject result = QJsonDocument::fromJson(QByteArray::fromStdString(params_memory.get("SpeedBumpSuggestResult"))).object();
    qint64 id = static_cast<qint64>(result.value("id").toDouble());
    if (lastSuggestionId >= 0 && id != lastSuggestionId && result.value("ok").toBool()) {
      QString status = result.value("status").toString();
      if (status == "new" || status == "merged") {
        showFeedback(tr("BUMP\nNOTED"), tr("tap to confirm"), MARK_COLOR, 6000);
      } else if (status == "promoted") {
        showFeedback(tr("BUMP\nLEARNED"), tr("hold to reject"), QColor(51, 224, 255), 6000);
      }
    }
    lastSuggestionId = id;
  }

  if (pendingId != 0) {
    // Only read while a reply is outstanding: at most ~2 s of 20 Hz reads per tap
    QJsonObject result = QJsonDocument::fromJson(QByteArray::fromStdString(params_memory.get("UserSpeedBumpResult"))).object();
    if (static_cast<qint64>(result.value("id").toDouble()) == pendingId) {
      bool ok = result.value("ok").toBool();
      QString message = result.value("message").toString();
      int total = result.value("total").toInt();

      if (!ok) {
        showFeedback(tr("NOT SAVED"), message, QColor(255, 80, 80), 4000);
      } else if (pendingAction == "undo") {
        showFeedback(message == "Detection rejected" ? tr("REJECTED") : tr("UNDONE"), tr("%1 saved").arg(total), QColor(200, 200, 200), 2500);
      } else if (message == "Bump confirmed") {
        showFeedback(tr("CONFIRMED"), tr("hold to undo"), QColor(80, 220, 100), 4000);
      } else {
        showFeedback(message == "Already marked" ? tr("ALREADY\nMARKED") : tr("MARKED"), tr("hold to undo"), QColor(80, 220, 100), 4000);
      }
      pendingId = 0;
    } else if (now - pendingSince > MARK_REPLY_TIMEOUT_MS) {
      showFeedback(tr("NOT SAVED"), tr("no reply from mapd"), QColor(255, 80, 80), 4000);
      pendingId = 0;
    }
  }

  if (!feedbackTitle.isEmpty() && now > feedbackUntil) {
    feedbackTitle.clear();
    feedbackDetail.clear();
    update();
  }
}

void SpeedBumpMarkButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  bool feedback = !feedbackTitle.isEmpty();
  QColor color = feedback ? feedbackColor : MARK_COLOR;

  QRect box = rect().adjusted(4, 4, -4, -4);
  p.setBrush(isDown() ? QColor(60, 60, 60, 230) : QColor(0, 0, 0, 180));
  p.setPen(QPen(color, 6));
  p.drawRoundedRect(box, 30, 30);

  p.setPen(color);
  if (feedback && !feedbackDetail.isEmpty()) {
    p.setFont(InterFont(feedbackTitle.contains('\n') ? 38 : 50, QFont::Bold));
    p.drawText(box.adjusted(10, 10, -10, -60), Qt::AlignCenter, feedbackTitle);
    p.setFont(InterFont(30, QFont::DemiBold));
    p.drawText(box.adjusted(10, box.height() - 70, -10, -10), Qt::AlignCenter | Qt::TextWordWrap, feedbackDetail);
  } else {
    p.setFont(InterFont(50, QFont::Bold));
    p.drawText(box, Qt::AlignCenter, feedback ? feedbackTitle : tr("MARK\nBUMP"));
  }
}

