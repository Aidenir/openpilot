#include "frogpilot/ui/qt/offroad/aidenirs_settings.h"

// "selfdrive/monitoring/helpers.py" clamps the three driver monitoring delays, in this order, to:
//
//   critical = clip(critical, 5, 120)
//   green    = clip(green, 1, critical - 2)
//   beeping  = clip(beeping, green + 1, critical - 1)
//
// so the values are interdependent and always end up as "green < beeping < critical". The controls
// below derive their ranges from each other using the exact same rules, so the panel can never offer
// a combination that the driver monitoring code would silently rewrite behind the user's back.
static constexpr int DM_CRITICAL_DELAY_MIN = 5;
static constexpr int DM_CRITICAL_DELAY_MAX = 120;
static constexpr int DM_GREEN_DELAY_MIN = 1;

AidenirsSettingsPanel::AidenirsSettingsPanel(FrogPilotSettingsWindow *parent, bool forceOpen) : FrogPilotListWidget(parent), parent(parent) {
  forceOpenDescriptions = forceOpen;

  for (int i = DM_GREEN_DELAY_MIN; i <= DM_CRITICAL_DELAY_MAX; ++i) {
    secondLabels[i] = i == 1 ? tr("1 second") : QString::number(i) + tr(" seconds");
  }

  const std::vector<std::tuple<QString, QString, QString, QString>> aidenirsToggles {
    {"DMGreenAlertDelay",
     tr("Green Alert Delay"),
     tr("<b>Seconds after looking away before the green \"Pay Attention\" visual alert appears.</b><br><br>"
        "Always kept at least 2 seconds below the \"Critical Alert Delay\", so its range moves with the other two delays."),
     ""},

    {"DMBeepingDelay",
     tr("Beeping Alert Delay"),
     tr("<b>Seconds after looking away before the orange alert and beeping starts.</b><br><br>"
        "Always kept above the \"Green Alert Delay\" and at least 1 second below the \"Critical Alert Delay\", so its range moves with the other two delays."),
     ""},

    {"DMCriticalDelay",
     tr("Critical Alert Delay"),
     tr("<b>Seconds after looking away before the red \"DISENGAGE IMMEDIATELY\" alert appears.</b><br><br>"
        "This is the delay the other two are measured against, so lowering it pulls them down with it."),
     ""},

    {"MassageReminder",
     tr("Massage Reminder"),
     tr("<b>Enable periodic reminders to turn on the massage function.</b> A gentle prompt will appear every 10 minutes while driving."),
     ""},

    {"SpeedBumpUI",
     tr("Speed Bump Display"),
     tr("<b>Display the distance to the next mapped speed bump ahead.</b> Requires map data from <b>mapd</b>."),
     ""},

    {"SpeedBumpAlertDistance",
     tr("Speed Bump Warning Distance"),
     tr("<b>How far before a speed bump to warn.</b> The map marks the middle of the bump, so this is measured to that point. At 50 km/h, 10 metres is under a second of warning."),
     ""},

    {"SpeedBumpTileCountUI",
     tr("Speed Bump Tile Count (Debug)"),
     tr("<b>Display a developer banner with the number of speed bumps in the freshly loaded map tile.</b> This is diagnostic output, not a driving aid."),
     ""},
  };

  for (const auto &[param, title, desc, icon] : aidenirsToggles) {
    AbstractControl *aidenirsToggle;

    if (param == "SpeedBumpAlertDistance") {
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 5, 100, tr(" meters"), std::map<float, QString>(), 5, true);

    } else if (driverMonitoringDelayKeys.contains(param)) {
      // The ranges are placeholders; "updateDelayRanges" gives every delay its real range below
      FrogPilotParamValueControl *delayToggle = new FrogPilotParamValueControl(param, title, desc, icon, DM_GREEN_DELAY_MIN, DM_CRITICAL_DELAY_MAX, QString(), secondLabels, 1, true);
      QObject::connect(delayToggle, &FrogPilotParamValueControl::valueChanged, [key = param, this](float value) {
        // Commit the new value before recalculating, since the other two delays are derived from what's stored
        params.putInt(key.toStdString(), static_cast<int>(std::lround(value)));

        updateDelayRanges();
      });
      aidenirsToggle = delayToggle;

    } else {
      aidenirsToggle = new ParamControl(param, title, desc, icon);
    }

    toggles[param] = aidenirsToggle;

    addItem(aidenirsToggle);

    QObject::connect(aidenirsToggle, &AbstractControl::hideDescriptionEvent, [this]() {
      update();
    });
    QObject::connect(aidenirsToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  updateDelayRanges();

  openDescriptions(forceOpenDescriptions, toggles);
}

void AidenirsSettingsPanel::showEvent(QShowEvent *event) {
  updateToggles();
}

void AidenirsSettingsPanel::updateDelayRanges() {
  int storedCriticalDelay = params.getInt("DMCriticalDelay");
  int storedGreenDelay = params.getInt("DMGreenAlertDelay");
  int storedBeepingDelay = params.getInt("DMBeepingDelay");

  int criticalDelay = std::clamp(storedCriticalDelay, DM_CRITICAL_DELAY_MIN, DM_CRITICAL_DELAY_MAX);
  int greenDelay = std::clamp(storedGreenDelay, DM_GREEN_DELAY_MIN, criticalDelay - 2);
  int beepingDelay = std::clamp(storedBeepingDelay, greenDelay + 1, criticalDelay - 1);

  // Store what the driver monitoring code would use anyways, so the panel, the params, and the alerts all agree
  if (storedCriticalDelay != criticalDelay) {
    params.putInt("DMCriticalDelay", criticalDelay);
  }
  if (storedGreenDelay != greenDelay) {
    params.putInt("DMGreenAlertDelay", greenDelay);
  }
  if (storedBeepingDelay != beepingDelay) {
    params.putInt("DMBeepingDelay", beepingDelay);
  }

  static_cast<FrogPilotParamValueControl*>(toggles["DMCriticalDelay"])->updateControl(DM_CRITICAL_DELAY_MIN, DM_CRITICAL_DELAY_MAX, secondLabels);
  static_cast<FrogPilotParamValueControl*>(toggles["DMGreenAlertDelay"])->updateControl(DM_GREEN_DELAY_MIN, criticalDelay - 2, secondLabels);
  static_cast<FrogPilotParamValueControl*>(toggles["DMBeepingDelay"])->updateControl(greenDelay + 1, criticalDelay - 1, secondLabels);
}

void AidenirsSettingsPanel::updateToggles() {
  updateDelayRanges();

  for (auto &[key, toggle] : toggles) {
    bool setVisible = parent->tuningLevel >= parent->frogpilotToggleLevels[key].toDouble();

    toggle->setVisible(setVisible);
  }

  openDescriptions(forceOpenDescriptions, toggles);

  update();
}
