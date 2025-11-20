#pragma once

#include <QPushButton>
#include <QLabel>

#include "system/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"

// SSH enable toggle
class SshToggle : public ToggleControl {
  Q_OBJECT

public:
  SshToggle() : ToggleControl(tr("Enable SSH"), "", "", Hardware::get_ssh_enabled()) {
    QObject::connect(this, &SshToggle::toggleFlipped, [=](bool state) {
      Hardware::set_ssh_enabled(state);
    });
  }
};

// SSH key management widget
class SshControl : public ButtonControl {
  Q_OBJECT

public:
  SshControl();

private:
  Params params;

  void refresh();
  void getUserKeys(const QString &username);

  // FrogPilot variables
  Params params_cache{"/cache/params"};
};

// Remote control password widget
class GoranConnectPasswordControl : public ButtonControl {
  Q_OBJECT

public:
  GoranConnectPasswordControl();

private:
  Params params;
  QLabel password_label;

  void refresh();
};
