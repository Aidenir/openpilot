#pragma once

#include "frogpilot/ui/qt/widgets/frogpilot_controls.h"

class FrogPilotDriverMonitoringPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotDriverMonitoringPanel(QWidget *parent = nullptr);

signals:
  void openParentToggle();

private:
  std::map<std::string, AbstractControl*> toggles;
};
