#pragma once

#include "cereal/messaging/messaging.h"

#include "frogpilot/ui/qt/widgets/frogpilot_controls.h"

struct FrogPilotUIScene {
  bool enabled;
  bool frogpilot_panel_active;
  bool online;
  bool parked;

  int started_timer;

  QJsonObject frogpilot_toggles;
};

class FrogPilotUIState : public QObject {
  Q_OBJECT

public:
  explicit FrogPilotUIState(QObject *parent = nullptr);

  void update();

  std::unique_ptr<SubMaster> sm;

  FrogPilotUIScene frogpilot_scene;

  Params params_memory{"", false, true};

signals:
};

FrogPilotUIState *frogpilotUIState();
