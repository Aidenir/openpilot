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

  void updateState();

  static constexpr int UNDO_HOLD_MS = 800;

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

