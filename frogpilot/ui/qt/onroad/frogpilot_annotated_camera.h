#pragma once

#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

const int widget_size = img_size + (UI_BORDER_SIZE / 2);

class FrogPilotAnnotatedCameraWidget : public QWidget {
  Q_OBJECT

public:
  explicit FrogPilotAnnotatedCameraWidget(QWidget *parent = 0);

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
  Params params;
  Params params_memory{"", false, true};
};
