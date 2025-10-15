#include "frogpilot/ui/qt/onroad/frogpilot_onroad.h"

FrogPilotOnroadWindow::FrogPilotOnroadWindow(QWidget *parent) : QWidget(parent) {
  signalTimer = new QTimer(this);

  QObject::connect(signalTimer, &QTimer::timeout, [this] {
    flickerActive = !flickerActive;
  });
}

void FrogPilotOnroadWindow::updateState(const UIState &s, const FrogPilotUIState &fs) {
  const FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  const QJsonObject &frogpilot_toggles = frogpilot_scene.frogpilot_toggles;
  const SubMaster &fpsm = *(fs.sm);

  const cereal::CarState::Reader &carState = fpsm["carState"].getCarState();
  const cereal::CarControl::Reader &carControl = fpsm["carControl"].getCarControl();

  blindSpotLeft = carState.getLeftBlindspot();
  blindSpotRight = carState.getRightBlindspot();
  turnSignalLeft = carState.getLeftBlinker();
  turnSignalRight = carState.getRightBlinker();

  showBlindspot = (blindSpotLeft || blindSpotRight) && frogpilot_toggles.value("blind_spot_metrics").toBool();
  showSignal = (turnSignalLeft || turnSignalRight) && frogpilot_toggles.value("signal_metrics").toBool();

  if (showBlindspot || showSignal) {
    update();
  }
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

  if (showBlindspot || showSignal) {
    int interval = showBlindspot ? 250 : 500;

    if (!signalTimer->isActive()) {
      signalTimer->start(interval);
    } else if (signalTimer->interval() != interval) {
      signalTimer->stop();
      signalTimer->start(interval);
    }

    paintTurnSignalBorder(p, rect);
  } else if (signalTimer->isActive()) {
    signalTimer->stop();
  }
}

void FrogPilotOnroadWindow::paintTurnSignalBorder(QPainter &p, const QRect &rect) {
  p.save();

  std::function<QColor(bool, bool)> getBorderColor = [&](bool blindSpot, bool turnSignal) {
    if (turnSignal && showSignal) {
      if (blindSpot) {
        return flickerActive ? bg_colors[STATUS_TRAFFIC_MODE_ENABLED] : bg_colors[STATUS_CONDITIONAL_OVERRIDDEN];
      } else {
        return flickerActive ? bg_colors[STATUS_CONDITIONAL_OVERRIDDEN] : bg;
      }
    } else if (blindSpot && showBlindspot) {
      return bg_colors[STATUS_TRAFFIC_MODE_ENABLED];
    } else {
      return bg;
    }
  };

  QColor borderColorLeft = getBorderColor(blindSpotLeft, turnSignalLeft);
  QColor borderColorRight = getBorderColor(blindSpotRight, turnSignalRight);

  p.fillRect(rect.x(), rect.y(), rect.width() / 2, rect.height(), borderColorLeft);
  p.fillRect(rect.x() + rect.width() / 2, rect.y(), rect.width() / 2, rect.height(), borderColorRight);

  p.restore();
}
