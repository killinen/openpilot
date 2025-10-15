#include "frogpilot/ui/qt/onroad/frogpilot_annotated_camera.h"

FrogPilotAnnotatedCameraWidget::FrogPilotAnnotatedCameraWidget(QWidget *parent) : QWidget(parent) {
}

void FrogPilotAnnotatedCameraWidget::showEvent(QShowEvent *event) {
  UIState &s = *uiState();
  UIScene &scene = s.scene;

  FrogPilotUIState &fs = *frogpilotUIState();
  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;

  if (scene.is_metric || frogpilot_toggles.value("use_si_metrics").toBool()) {
    accelerationUnit = tr(" m/s²");
    leadDistanceUnit = tr(" meters");
    leadSpeedUnit = frogpilot_toggles.value("use_si_metrics").toBool() ? tr(" m/s") : tr(" km/h");

    distanceConversion = 1.0f;
    speedConversion = scene.is_metric ? MS_TO_KPH : MS_TO_MPH;
    speedConversionMetrics = frogpilot_toggles.value("use_si_metrics").toBool() ? 1.0f : MS_TO_KPH;
  } else {
    accelerationUnit = tr(" ft/s²");
    leadDistanceUnit = tr(" feet");
    leadSpeedUnit = tr(" mph");

    distanceConversion = METER_TO_FOOT;
    speedConversion = MS_TO_MPH;
    speedConversionMetrics = MS_TO_MPH;
  }
}

void FrogPilotAnnotatedCameraWidget::updateState(const FrogPilotUIState &fs) {
  const FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  const SubMaster &fpsm = *(fs.sm);

  update();
}

void FrogPilotAnnotatedCameraWidget::paintFrogPilotWidgets(QPainter &p, UIState &s, FrogPilotUIState &fs, SubMaster &sm, SubMaster &fpsm) {
  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;

  const cereal::CarState::Reader &carState = fpsm["carState"].getCarState();
}
