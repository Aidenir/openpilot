#include "frogpilot/ui/qt/offroad/aidenirs_settings.h"

AidenirsSettingsPanel::AidenirsSettingsPanel(QWidget *parent)
  : FrogPilotListWidget(parent) {

  const std::vector<std::tuple<QString, QString, QString, QString>> aidenirsToggles {
    {"DMGreenAlertDelay",
     tr("Green Alert Delay"),
     tr("<b>Seconds after looking away before the green 'Pay Attention' visual alert appears.</b>"),
     ""},

    {"DMBeepingDelay",
     tr("Beeping Alert Delay"),
     tr("<b>Seconds after looking away before the orange alert and beeping starts.</b>"),
     ""},

    {"DMCriticalDelay",
     tr("Critical Alert Delay"),
     tr("<b>Seconds after looking away before the red 'DISENGAGE IMMEDIATELY' alert appears.</b>"),
     ""},

    {"MassageReminder",
     tr("Massage Reminder"),
     tr("<b>Enable periodic reminders to turn on the massage function.</b> A gentle prompt will appear every 10 minutes while driving."),
     ""},

    {"SpeedBumpUI",
     tr("Speed Bump Display"),
     tr("<b>Display the distance to the next mapped speed bump ahead.</b> Requires map data from <b>mapd</b>."),
     ""},

    {"SpeedBumpTileCountUI",
     tr("Speed Bump Tile Count (Debug)"),
     tr("<b>Display a developer banner with the number of speed bumps in the freshly loaded map tile.</b> This is diagnostic output, not a driving aid."),
     ""},
  };

  for (const auto &[param, title, desc, icon] : aidenirsToggles) {
    AbstractControl *toggle;

    if (param == "DMGreenAlertDelay") {
      toggle = new FrogPilotParamValueControl(
        param, title, desc, icon,
        1.0f, 30.0f,  // Min 1s, Max 30s
        tr(" seconds"), std::map<float, QString>(), 1.0f
      );
    } else if (param == "DMBeepingDelay") {
      toggle = new FrogPilotParamValueControl(
        param, title, desc, icon,
        1.0f, 60.0f,  // Min 1s, Max 60s
        tr(" seconds"), std::map<float, QString>(), 1.0f
      );
    } else if (param == "DMCriticalDelay") {
      toggle = new FrogPilotParamValueControl(
        param, title, desc, icon,
        5.0f, 120.0f,  // Min 5s, Max 120s (2 minutes)
        tr(" seconds"), std::map<float, QString>(), 1.0f
      );
    } else {
      // MassageReminder, SpeedBumpUI and SpeedBumpTileCountUI are simple boolean toggles
      toggle = new ParamControl(param, title, desc, icon);
    }

    toggles[param.toStdString()] = toggle;
    addItem(toggle);
  }
}
