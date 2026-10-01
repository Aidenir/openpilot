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

    {"DMAwarenessBar",
     tr("Attention Budget Bar"),
     tr("<b>Show driver monitoring's attention budget as a bar next to the face icon.</b> It drains while you look away and refills while you watch the road.<br><br>"
        "The two marks show where the green \"Pay Attention\" alert and the orange beeping start, and the number is the seconds left before the red alert. "
        "When your face can't be seen it switches to the steering-wheel budget, which always lasts 30 seconds."),
     ""},

    {"MassageReminder",
     tr("Massage Reminder"),
     tr("<b>Enable periodic reminders to turn on the massage function.</b> A gentle prompt will appear every 10 minutes while driving."),
     ""},

    {"SpeedBumpUI",
     tr("Speed Bump Display"),
     tr("<b>Display the distance to the next mapped speed bump ahead.</b> Requires map data from <b>mapd</b>."),
     ""},

    {"UserSpeedBumpButton",
     tr("Mark Speed Bump Button"),
     tr("<b>Show an onroad button that marks a speed bump at the car's position.</b> For bumps missing from the map: "
        "tap it as you drive over one and <b>mapd</b> saves it to \"user_speed_bumps.json\" and warns for it from then on, in both directions. "
        "Hold the button to undo the last mark."),
     ""},

    {"SpeedBumpDetect",
     tr("Learn Unmapped Speed Bumps"),
     tr("<b>Notice bumps the map is missing from how the car pitches over them.</b> Each one is only a suggestion until it has been "
        "felt on enough separate drives; then it is treated like a marked bump (warning and slowdown). "
        "With the \"Mark Speed Bump Button\" on, the button briefly shows \"BUMP NOTED\": tap it to confirm straight away, "
        "hold it right after \"BUMP LEARNED\" to reject the detection for good."),
     ""},

    {"SpeedBumpDetectThreshold",
     tr("Bump Detection Sensitivity"),
     tr("<b>How sharp a pitch (rad/s) counts as a bump.</b> Lower finds gentler bumps and cushions but also more potholes and "
        "tram tracks. 0.15 caught every mapped bump that was driven at normal speed on the recorded drives."),
     ""},

    {"SpeedBumpDetectPromoteDrives",
     tr("Drives Before a Bump Is Learned"),
     tr("<b>On how many separate drives a suggested bump must be felt before it becomes active.</b> A drive ends after 10 minutes stopped. "
        "0 never learns automatically; bumps then only become active when confirmed with the button."),
     ""},

    {"SpeedBumpAlertDistance",
     tr("Speed Bump Warning Distance"),
     tr("<b>How far before a speed bump to warn.</b> The map marks the middle of the bump, so this is measured to that point. At 50 km/h, 10 metres is under a second of warning."),
     ""},

    {"SpeedBumpSlowdown",
     tr("Slow Down for Speed Bumps"),
     tr("<b>Slow down for mapped speed bumps ahead.</b> Uses the same <b>mapd</b> map data as the speed bump display, so bumps that are "
        "missing from OpenStreetMap are not slowed for. It only ever slows the car and never goes above your set speed. "
        "Pressing the gas overrides it as usual. Requires openpilot longitudinal control."),
     ""},

    {"SpeedBumpSlowdownSpeed",
     tr("Speed Bump Speed"),
     tr("<b>The speed to drive over speed bumps at.</b> If your set speed is already lower, your set speed is kept."),
     ""},

    {"SpeedBumpSlowdownTime",
     tr("Speed Bump Braking Point"),
     tr("<b>Where braking starts: this many seconds before the bump, at the speed you're driving.</b> This is the knob that moves the braking point. "
        "The distance grows with speed: 3 seconds is about 25 metres at 30 km/h and 42 metres at 50 km/h. "
        "Braking is then just firm enough to be at the \"Speed Bump Speed\" as the bump starts, so a later braking point means firmer braking. "
        "If that would need more than the \"Speed Bump Max Braking\", what happens depends on \"Strict Braking Point\". "
        "The speed is held until the car is past the bump (\"Speed Bump Hold Distance\")."),
     ""},

    {"SpeedBumpSlowdownMaxDecel",
     tr("Speed Bump Max Braking"),
     tr("<b>The firmest braking allowed when slowing for a bump.</b> A safety limit, not the usual braking: normally braking is only as firm as the "
        "braking point needs. Lower it for gentler stops; braking then starts earlier at higher speeds. "
        "Getting from 50 to 20 km/h needs about 39 metres at 3.0 m/s² and 50 metres at 2.0 m/s². 3.5 m/s² is openpilot's own braking limit."),
     ""},

    {"SpeedBumpSlowdownStrict",
     tr("Strict Braking Point"),
     tr("<b>What to do when the braking point is too close to reach the bump speed within \"Speed Bump Max Braking\".</b><br><br>"
        "<b>Off (default):</b> always reach the \"Speed Bump Speed\" at the bump. Braking starts earlier than the braking point when it has to, "
        "e.g. about 39 metres instead of 28 at 50 km/h with the defaults.<br><br>"
        "<b>On:</b> never start braking before the braking point. Braking is at most \"Speed Bump Max Braking\", so at higher speeds the car "
        "reaches the bump faster than the \"Speed Bump Speed\" (about 35 km/h from 50 km/h with the defaults)."),
     ""},

    {"SpeedBumpSlowdownResponseTime",
     tr("Speed Bump Car Response Time"),
     tr("<b>How long the car takes to react to a braking request.</b> Braking is planned this much earlier to make up for it, and eased off a "
        "little earlier still. Raise it if the car reaches bumps faster than the \"Speed Bump Speed\"; lower it if it slows down too early "
        "or dips below it."),
     ""},

    {"SpeedBumpSlowdownMargin",
     tr("Speed Bump Arrival Margin"),
     tr("<b>How far before the middle of the bump to already be at the \"Speed Bump Speed\".</b> The map marks the middle of the bump, so "
        "about half a bump's length (2 metres) means being slow as the bump starts. Raise it to be slow earlier."),
     ""},

    {"SpeedBumpSlowdownHold",
     tr("Speed Bump Hold Distance"),
     tr("<b>How far past the middle of the bump to keep the \"Speed Bump Speed\" before speeding up again.</b> "
        "The default of 6 metres gets the rear wheels over a typical bump."),
     ""},

    {"SpeedBumpSlowdownJerk",
     tr("Speed Bump Braking Smoothness"),
     tr("<b>How quickly braking builds up and eases off.</b> 1.0 follows the usual comfort limits for cruise control (faster at low speed, "
        "slower at high speed). Lower is smoother but needs an earlier braking point for the same result; higher is more abrupt."),
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

    } else if (param == "SpeedBumpSlowdownSpeed") {
      // Always km/h: the param is stored and read as km/h regardless of the unit setting
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 5, 50, tr(" km/h"), std::map<float, QString>(), 1, true);

    } else if (param == "SpeedBumpSlowdownTime") {
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0.5, 5, tr(" seconds"), std::map<float, QString>(), 0.1);

    } else if (param == "SpeedBumpSlowdownMaxDecel") {
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 1, 3.5, tr(" m/s²"), std::map<float, QString>(), 0.1);

    } else if (param == "SpeedBumpSlowdownResponseTime") {
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 1, tr(" seconds"), std::map<float, QString>(), 0.05);

    } else if (param == "SpeedBumpSlowdownMargin") {
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 10, tr(" meters"), std::map<float, QString>(), 0.5);

    } else if (param == "SpeedBumpSlowdownHold") {
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 20, tr(" meters"), std::map<float, QString>(), 1);

    } else if (param == "SpeedBumpSlowdownJerk") {
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0.5, 2, tr("x"), std::map<float, QString>(), 0.1);

    } else if (param == "SpeedBumpDetectThreshold") {
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0.08, 0.4, tr(" rad/s"), std::map<float, QString>(), 0.01);

    } else if (param == "SpeedBumpDetectPromoteDrives") {
      std::map<float, QString> driveLabels{{0, tr("Never")}};
      aidenirsToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 10, tr(" drives"), driveLabels, 1);

    } else if (param == "SpeedBumpDetect") {
      ParamControl *detectToggle = new ParamControl(param, title, desc, icon);
      QObject::connect(detectToggle, &ToggleControl::toggleFlipped, this, &AidenirsSettingsPanel::updateToggles);
      aidenirsToggle = detectToggle;

    } else if (param == "SpeedBumpSlowdown") {
      ParamControl *slowdownToggle = new ParamControl(param, title, desc, icon);
      QObject::connect(slowdownToggle, &ToggleControl::toggleFlipped, this, &AidenirsSettingsPanel::updateToggles);
      aidenirsToggle = slowdownToggle;

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

    if (key == "SpeedBumpDetectThreshold" || key == "SpeedBumpDetectPromoteDrives") {
      setVisible &= params.getBool("SpeedBumpDetect");
    } else if (key == "SpeedBumpSlowdown") {
      setVisible &= parent->hasOpenpilotLongitudinal;
    } else if (speedBumpSlowdownKeys.contains(key)) {
      setVisible &= parent->hasOpenpilotLongitudinal && params.getBool("SpeedBumpSlowdown");
    }

    toggle->setVisible(setVisible);
  }

  openDescriptions(forceOpenDescriptions, toggles);

  update();
}
