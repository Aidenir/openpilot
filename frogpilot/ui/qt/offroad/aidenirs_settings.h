#pragma once

#include "frogpilot/ui/qt/widgets/frogpilot_controls.h"

class AidenirsSettingsPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit AidenirsSettingsPanel(QWidget *parent = nullptr);

private:
  std::map<std::string, AbstractControl*> toggles;
};
