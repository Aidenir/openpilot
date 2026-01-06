#include "frogpilot/ui/qt/offroad/driver_monitoring_settings.h"

FrogPilotDriverMonitoringPanel::FrogPilotDriverMonitoringPanel(QWidget *parent)
  : FrogPilotListWidget(parent) {

  const std::vector<std::tuple<QString, QString, QString, QString>> dmTimingToggles {
    {"DMGreenAlertDelay",
     tr("Green Alert Delay"),
     tr("Seconds after looking away before the green 'Pay Attention' visual alert appears."),
     ""},

    {"DMBeepingDelay",
     tr("Beeping Alert Delay"),
     tr("Seconds after looking away before the orange alert and beeping starts."),
     ""},

    {"DMCriticalDelay",
     tr("Critical Alert Delay"),
     tr("Seconds after looking away before the red 'DISENGAGE IMMEDIATELY' alert appears."),
     ""},
  };

  for (const auto &[param, title, desc, icon] : dmTimingToggles) {
    FrogPilotParamValueControl *dmToggle;

    if (param == "DMGreenAlertDelay") {
      dmToggle = new FrogPilotParamValueControl(
        param, title, desc, icon,
        1.0f, 30.0f,  // Min 1s, Max 30s
        tr(" seconds"), std::map<float, QString>(), 1.0f
      );
    } else if (param == "DMBeepingDelay") {
      dmToggle = new FrogPilotParamValueControl(
        param, title, desc, icon,
        1.0f, 60.0f,  // Min 1s, Max 60s
        tr(" seconds"), std::map<float, QString>(), 1.0f
      );
    } else if (param == "DMCriticalDelay") {
      dmToggle = new FrogPilotParamValueControl(
        param, title, desc, icon,
        5.0f, 120.0f,  // Min 5s, Max 120s (2 minutes)
        tr(" seconds"), std::map<float, QString>(), 1.0f
      );
    }

    toggles[param.toStdString()] = dmToggle;
    addItem(dmToggle);
  }
}
