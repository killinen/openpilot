#include "frogpilot/ui/qt/onroad/frogpilot_annotated_camera.h"

FrogPilotAnnotatedCameraWidget::FrogPilotAnnotatedCameraWidget(QWidget *parent) : QWidget(parent) {
}

void FrogPilotAnnotatedCameraWidget::showEvent(QShowEvent *event) {
  UIState &s = *uiState();
  UIScene &scene = s.scene;

  FrogPilotUIState &fs = *frogpilotUIState();
  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  QJsonObject &frogpilot_toggles = frogpilot_scene.frogpilot_toggles;

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

void FrogPilotAnnotatedCameraWidget::updateState(const FrogPilotUIState &fs, const QJsonObject &frogpilot_toggles) {
  const FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  const SubMaster &fpsm = *(fs.sm);

  const cereal::FrogPilotPlan::Reader &frogpilotPlan = fpsm["frogpilotPlan"].getFrogpilotPlan();

  update();
}

void FrogPilotAnnotatedCameraWidget::paintFrogPilotWidgets(QPainter &p, UIState &s, FrogPilotUIState &fs, SubMaster &sm, SubMaster &fpsm, QJsonObject &frogpilot_toggles) {
  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;

  const cereal::CarState::Reader &carState = fpsm["carState"].getCarState();
  const cereal::FrogPilotCarState::Reader &frogpilotCarState = fpsm["frogpilotCarState"].getFrogpilotCarState();
  const cereal::FrogPilotPlan::Reader &frogpilotPlan = fpsm["frogpilotPlan"].getFrogpilotPlan();

  if (!hideBottomIcons && frogpilot_toggles.value("compass").toBool()) {
    paintCompass(p, frogpilot_toggles);
  } else {
    compassPosition.setX(0);
    compassPosition.setY(0);
  }
}

void FrogPilotAnnotatedCameraWidget::paintCompass(QPainter &p, QJsonObject &frogpilot_toggles) {
  p.save();

  compassPosition.rx() = rightHandDM ? UI_BORDER_SIZE + widget_size / 2 : width() - UI_BORDER_SIZE - btn_size;
  if (rightHandDM) {
    compassPosition.rx() += btn_size - UI_BORDER_SIZE;
  } else {
    compassPosition.rx() -= btn_size + UI_BORDER_SIZE;
  }
  compassPosition.ry() = dmIconPosition.y() - widget_size / 2;

  QRect compassWidget(compassPosition, QSize(widget_size, widget_size));

  p.setBrush(blackColor(166));
  p.setPen(QPen(blackColor(), 10));
  p.drawRoundedRect(compassWidget, 24, 24);

  QPainterPath clipPath;
  clipPath.addRoundedRect(compassWidget.adjusted(5, 5, -5, -5), 24, 24);
  p.setClipPath(clipPath);

  QFont font = InterFont(65, QFont::Bold);
  QFontMetrics fm(font);
  p.setFont(font);
  p.setPen(QPen(whiteColor()));

  const double pixelsPerDegree = 2.5;

  const int baseRibbonWidth = qRound(360 * pixelsPerDegree);

  static QPixmap ribbonPixmap;
  if (!ribbonPixmap) {
    int ribbonHeight = compassWidget.height();
    int ribbonWidth = baseRibbonWidth * 2;

    ribbonPixmap = QPixmap(ribbonWidth, ribbonHeight);
    ribbonPixmap.fill(Qt::transparent);

    QPainter ribbonPainter(&ribbonPixmap);
    ribbonPainter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
    ribbonPainter.setFont(font);

    QMap<int, QString> directionLabels = {{0, "N"}, {45, "NE"}, {90, "E"}, {135, "SE"}, {180, "S"}, {225, "SW"}, {270, "W"}, {315, "NW"}, {360, "N"}};

    for (int i = 0; i < 2; ++i) {
      for (int degree = 0; degree < 360; ++degree) {
        int x = qRound((i * 360 + degree) * pixelsPerDegree);

        if (directionLabels.contains(degree)) {
          const QString &label = directionLabels[degree];
          int textX = x - fm.horizontalAdvance(label) / 2;

          ribbonPainter.setPen(whiteColor());
          ribbonPainter.drawText(textX, fm.ascent(), label);
        }

        int notchHeight = (degree % 45 == 0) ? 35 : (degree % 15 == 0) ? 25 : 15;
        int notchWidth = (degree % 45 == 0) ? 5 : (degree % 15 == 0) ? 4 : 3;

        ribbonPainter.setPen(QPen(whiteColor(), notchWidth));
        ribbonPainter.drawLine(x, ribbonHeight - notchHeight - 5, x, ribbonHeight);
      }
    }
  }

  double rawBearing = QJsonDocument::fromJson(QString::fromStdString(params_memory.get("LastGPSPosition")).toUtf8()).object().value("bearing").toDouble(0.0);

  int bearing = qRound(fmod(rawBearing + 360.0, 360.0));
  int offset = qRound(bearing * pixelsPerDegree) % baseRibbonWidth;
  int drawX = compassWidget.center().x() - offset;

  p.drawPixmap(drawX - baseRibbonWidth, compassWidget.top() + 5, ribbonPixmap);
  p.drawPixmap(drawX, compassWidget.top() + 5, ribbonPixmap);

  int triangleSize = 40;
  int triangleX = compassWidget.center().x();
  int triangleY = compassWidget.bottom() - triangleSize;

  QPolygon triangle(QVector<QPoint>{
    QPoint(triangleX, triangleY - triangleSize),
    QPoint(triangleX - triangleSize / 1.5, triangleY),
    QPoint(triangleX + triangleSize / 1.5, triangleY)
  });

  p.setBrush(whiteColor());
  p.setPen(Qt::NoPen);
  p.drawPolygon(triangle);

  p.restore();
}

void FrogPilotAnnotatedCameraWidget::paintBlindSpotPath(QPainter &p, const cereal::CarState::Reader &carState, const FrogPilotUIScene &frogpilot_scene) {
  p.save();

  QLinearGradient bs(0, height(), 0, 0);
  bs.setColorAt(0.0f, QColor::fromHslF(0 / 360.0f, 0.75f, 0.5f, 0.6f));
  bs.setColorAt(0.5f, QColor::fromHslF(0 / 360.0f, 0.75f, 0.5f, 0.4f));
  bs.setColorAt(1.0f, QColor::fromHslF(0 / 360.0f, 0.75f, 0.5f, 0.2f));

  p.setBrush(bs);
  if (frogpilot_scene.lane_width_left != 0 && carState.getLeftBlindspot()) {
    p.drawPolygon(frogpilot_scene.track_adjacent_vertices[0]);
  }
  if (frogpilot_scene.lane_width_right != 0 && carState.getRightBlindspot()) {
    p.drawPolygon(frogpilot_scene.track_adjacent_vertices[1]);
  }

  p.restore();
}
