#pragma once

#include "selfdrive/ui/qt/onroad/buttons.h"

class DrivingPersonalityButton : public QPushButton {
  Q_OBJECT

public:
  explicit DrivingPersonalityButton(QWidget *parent = 0);

  void updateState(const UIState &s, const FrogPilotUIState &fs);

private:
  void paintEvent(QPaintEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void updateTheme();

  bool theme_updated;
  bool traffic_mode_active;

  int personality;

  Params params_memory{"", true};

  QMap<int, QPair<QPixmap, QSharedPointer<QMovie>>> icon_map;

  QMovie *currentGif;

  QPixmap currentImg;
};

// Marks a speed bump at the car's position, for bumps missing from the map. A tap
// asks mapd to record one, holding for UNDO_HOLD_MS asks it to remove the last one.
// The request goes to mapd as a memory param stamped with the time of the press, so
// mapd can place the bump where the car was at the tap rather than where it is when
// the request is read. mapd's reply is shown on the button itself.
class SpeedBumpMarkButton : public QPushButton {
  Q_OBJECT

public:
  explicit SpeedBumpMarkButton(QWidget *parent = 0);

  // showSuggestions: also flash the IMU detector's findings (SpeedBumpDetect on)
  void updateState(bool showSuggestions = false);

  static constexpr int UNDO_HOLD_MS = 800;

private:
  void paintEvent(QPaintEvent *event) override;
  void sendRequest(const QString &action, qint64 tapMs);
  void showFeedback(const QString &title, const QString &detail, const QColor &color, int durationMs);

  bool holdFired = false;

  qint64 feedbackUntil = 0;
  qint64 lastRequestId = 0;
  qint64 pendingId = 0;
  qint64 lastSuggestionId = -1;  // -1: not read yet, so a stale result is not shown on start
  int suggestionPollCounter = 0;
  qint64 pendingSince = 0;
  qint64 pressMs = 0;

  Params params;
  Params params_memory{"", true};

  QColor feedbackColor;

  QString feedbackDetail;
  QString feedbackTitle;
  QString pendingAction;

  QTimer *holdTimer;
};

// Marks a refuge island (a kerbed island in the middle of the road) at the car's position, for islands missing from the map,
// the way SpeedBumpMarkButton marks bumps: a tap asks mapd to record one, holding for UNDO_HOLD_MS to remove the last one.
// openpilot keeps right of the islands mapd knows about ("Keep Right Of Refuge Islands")
class RefugeIslandMarkButton : public QPushButton {
  Q_OBJECT

public:
  explicit RefugeIslandMarkButton(QWidget *parent = 0);

  void updateState();

  static constexpr int UNDO_HOLD_MS = SpeedBumpMarkButton::UNDO_HOLD_MS;

private:
  void paintEvent(QPaintEvent *event) override;
  void sendRequest(const QString &action, qint64 tapMs);
  void showFeedback(const QString &title, const QString &detail, const QColor &color, int durationMs);

  bool holdFired = false;

  qint64 feedbackUntil = 0;
  qint64 lastRequestId = 0;
  qint64 pendingId = 0;
  qint64 pendingSince = 0;
  qint64 pressMs = 0;

  Params params_memory{"", true};

  QColor feedbackColor;

  QString feedbackDetail;
  QString feedbackTitle;
  QString pendingAction;

  QTimer *holdTimer;
};

// Shown with the "No GPS Signal" alert. The modem's GPS sometimes stays silent for a whole drive (2026-10-02) and only a reboot
// has been seen to bring it back. A tap asks for confirmation first, since rebooting drops openpilot until it is back up
class GpsRebootButton : public QPushButton {
  Q_OBJECT

public:
  explicit GpsRebootButton(QWidget *parent = 0);

  // Whether the "No GPS Signal" alert is the one on screen
  static bool alertShown(const UIState &s);

private:
  void paintEvent(QPaintEvent *event) override;

  Params params;
};
