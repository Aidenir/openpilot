#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class AidenirsSettingsPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit AidenirsSettingsPanel(FrogPilotSettingsWindow *parent, bool forceOpen = false);

protected:
  void showEvent(QShowEvent *event) override;

private:
  void updateDelayRanges();
  void updateToggles();

  bool forceOpenDescriptions;

  std::map<float, QString> secondLabels;

  std::map<QString, AbstractControl*> toggles;

  QSet<QString> driverMonitoringDelayKeys {"DMBeepingDelay", "DMCriticalDelay", "DMGreenAlertDelay"};

  FrogPilotSettingsWindow *parent;

  Params params;
};
