#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class AidenirsSettingsPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit AidenirsSettingsPanel(FrogPilotSettingsWindow *parent, bool forceOpen = false);

signals:
  void openSubPanel();

protected:
  void showEvent(QShowEvent *event) override;

private:
  QString categoryOf(const QString &key) const;
  void updateDelayRanges();
  void updateToggles();

  bool forceOpenDescriptions;

  std::map<float, QString> secondLabels;

  std::map<QString, AbstractControl*> toggles;

  QSet<QString> driverMonitoringDelayKeys {"DMBeepingDelay", "DMCriticalDelay", "DMGreenAlertDelay"};
  QSet<QString> speedBumpSlowdownKeys {"SpeedBumpSlowdownHold", "SpeedBumpSlowdownJerk", "SpeedBumpSlowdownMargin", "SpeedBumpSlowdownMaxDecel",
                                       "SpeedBumpMildExtraSpeed", "SpeedBumpSlowdownCEMDelay", "SpeedBumpSlowdownResponseTime", "SpeedBumpSlowdownSpeed", "SpeedBumpSlowdownStrict", "SpeedBumpSlowdownTableLength",
                                       "SpeedBumpSlowdownTime"};

  // The submenus, by the key of the button that opens them. "SpeedBumpSlowdown" is a real toggle that also opens its settings;
  // the others are only buttons
  std::map<QString, QSet<QString>> categoryKeys {
    {"AidenirsDriverMonitoring", {"DMAwarenessBar", "DMBeepingDelay", "DMCriticalDelay", "DMGreenAlertDelay", "HideDMIcon"}},
    {"AidenirsDrivingScreen", {"CEMStatusTop", "MassageReminder"}},
    {"AidenirsSpeedBumps", {"SpeedBumpAlertDistance", "SpeedBumpApproachIcon", "SpeedBumpApproachIconDistance", "SpeedBumpDetect",
                            "SpeedBumpDetectPromoteDrives", "SpeedBumpDetectThreshold", "SpeedBumpLearn", "SpeedBumpMarkOneWay",
                            "SpeedBumpTileCountUI", "SpeedBumpUI", "UserSpeedBumpButton"}},
    {"SpeedBumpSlowdown", speedBumpSlowdownKeys},
    {"AidenirsRefugeIslands", {"RefugeIslandNudge", "RefugeIslandOffset", "UserRefugeIslandButton"}},
    {"AidenirsOvertaking", {"OvertakeSuggestion"}},
  };

  FrogPilotSettingsWindow *parent;

  Params params;
};
