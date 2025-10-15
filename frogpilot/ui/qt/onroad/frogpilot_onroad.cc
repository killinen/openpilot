#include "frogpilot/ui/qt/onroad/frogpilot_onroad.h"

FrogPilotOnroadWindow::FrogPilotOnroadWindow(QWidget *parent) : QWidget(parent) {
}

void FrogPilotOnroadWindow::updateState(const UIState &s, const FrogPilotUIState &fs) {
  const FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  const QJsonObject &frogpilot_toggles = frogpilot_scene.frogpilot_toggles;
  const SubMaster &fpsm = *(fs.sm);

  const cereal::CarState::Reader &carState = fpsm["carState"].getCarState();
  const cereal::CarControl::Reader &carControl = fpsm["carControl"].getCarControl();
}

void FrogPilotOnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);

  QRect rect = this->rect();

  QRegion marginRegion;
  marginRegion += QRegion(0, 0, rect.width(), UI_BORDER_SIZE);
  marginRegion += QRegion(0, rect.height() - UI_BORDER_SIZE, rect.width(), UI_BORDER_SIZE);
  marginRegion += QRegion(0, UI_BORDER_SIZE, UI_BORDER_SIZE, rect.height() - 2 * UI_BORDER_SIZE);
  marginRegion += QRegion(rect.width() - UI_BORDER_SIZE, UI_BORDER_SIZE, UI_BORDER_SIZE, rect.height() - 2 * UI_BORDER_SIZE);
  p.setClipRegion(marginRegion);
}
