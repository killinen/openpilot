#pragma once

#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

const int widget_size = img_size + (UI_BORDER_SIZE / 2);

class FrogPilotAnnotatedCameraWidget : public QWidget {
  Q_OBJECT

public:
  explicit FrogPilotAnnotatedCameraWidget(QWidget *parent = 0);

  void paintBlindSpotPath(QPainter &p, const cereal::CarState::Reader &carState, const FrogPilotUIScene &frogpilot_scene);
  void paintFrogPilotWidgets(QPainter &p, UIState &s, FrogPilotUIState &fs, SubMaster &sm, SubMaster &fpsm, QJsonObject &frogpilot_toggles);
  void updateState(const FrogPilotUIState &fs, const QJsonObject &frogpilot_toggles);

  bool hideBottomIcons;
  bool rightHandDM;

  float distanceConversion;
  float setSpeed;
  float speed;
  float speedConversion;
  float speedConversionMetrics;

  QString accelerationUnit;
  QString leadDistanceUnit;
  QString leadSpeedUnit;
  QString speedUnit;

protected:
  void showEvent(QShowEvent *event) override;

private:
  void paintCEMStatus(QPainter &p, const cereal::FrogPilotPlan::Reader &frogpilotPlan, FrogPilotUIScene &frogpilot_scene, SubMaster &sm);
  void paintCompass(QPainter &p, QJsonObject &frogpilot_toggles);

  Params params;
  Params params_memory{"", false, true};

  QPoint cemStatusPosition;
  QPoint compassPosition;

  QSharedPointer<QMovie> cemCurveIcon;
  QSharedPointer<QMovie> cemLeadIcon;
  QSharedPointer<QMovie> cemSpeedIcon;
  QSharedPointer<QMovie> cemStopIcon;
  QSharedPointer<QMovie> cemTurnIcon;
  QSharedPointer<QMovie> chillModeIcon;
  QSharedPointer<QMovie> experimentalModeIcon;
};
