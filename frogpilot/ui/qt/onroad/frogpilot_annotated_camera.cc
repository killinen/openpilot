#include "frogpilot/ui/qt/onroad/frogpilot_annotated_camera.h"

FrogPilotAnnotatedCameraWidget::FrogPilotAnnotatedCameraWidget(QWidget *parent) : QWidget(parent) {
  animationTimer = new QTimer(this);

  loadGif("../../frogpilot/assets/other_images/curve_icon.gif", cemCurveIcon, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/lead_icon.gif", cemLeadIcon, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/speed_icon.gif", cemSpeedIcon, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/light_icon.gif", cemStopIcon, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/turn_icon.gif", cemTurnIcon, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/chill_mode_icon.gif", chillModeIcon, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/experimental_mode_icon.gif", experimentalModeIcon, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/weather_clear_day.gif", weather_clear_day, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/weather_clear_night.gif", weather_clear_night, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/weather_rain.gif", weather_rain, QSize(btn_size / 2, btn_size / 2), this);
  loadGif("../../frogpilot/assets/other_images/weather_snow.gif", weather_snow, QSize(btn_size / 2, btn_size / 2), this);

  QObject::connect(animationTimer, &QTimer::timeout, [this] {
    animationFrameIndex = (animationFrameIndex + 1) % totalFrames;
  });
  QObject::connect(frogpilotUIState(), &FrogPilotUIState::themeUpdated, this, &FrogPilotAnnotatedCameraWidget::updateSignals);
  QObject::connect(uiState(), &UIState::offroadTransition, [this] {
    standstillTimer.invalidate();

    QJsonObject stats = QJsonDocument::fromJson(QString::fromStdString(params.get("FrogPilotStats")).toUtf8()).object();
    stats["FrogHops"] = stats.value("FrogHops").toInt(0) + frogHopCount;
    params.putNonBlocking("FrogPilotStats", QJsonDocument(stats).toJson(QJsonDocument::Compact).toStdString());

    frogHopCount = 0;
  });
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

  updateSignals();
}

void FrogPilotAnnotatedCameraWidget::updateSignals() {
  QVector<QPixmap>().swap(blindspotImages);
  QVector<QPixmap>().swap(signalImages);

  bool isGif = false;

  QFileInfoList files = QDir("../../frogpilot/assets/active_theme/signals/").entryInfoList(QDir::Files | QDir::NoDotAndDotDot, QDir::Name);
  for (const QFileInfo &fileInfo : files) {
    QString fileName = fileInfo.fileName();
    QString filePath = fileInfo.absoluteFilePath();

    if (fileName.endsWith(".gif", Qt::CaseInsensitive)) {
      isGif = true;

      QMovie movie(filePath);
      movie.setCacheMode(QMovie::CacheNone);
      movie.start();

      int frameCount = movie.frameCount();
      signalImages.reserve(frameCount);

      for (int i = 0; i < frameCount; ++i) {
        movie.jumpToFrame(i);

        QImage image = movie.currentPixmap().toImage().convertToFormat(QImage::Format_Indexed8);
        QPixmap frame = QPixmap::fromImage(image);
        signalImages.append(frame);
      }

      movie.stop();
    } else if (fileName.endsWith(".png", Qt::CaseInsensitive)) {
      QVector<QPixmap> &targetList = fileName.contains("blindspot", Qt::CaseInsensitive) ? blindspotImages : signalImages;
      targetList.append(QPixmap::fromImage(QImage(filePath).convertToFormat(QImage::Format_Indexed8)));
    } else {
      QStringList parts = fileName.split('_');
      if (parts.size() == 2) {
        signalStyle = parts[0];
        signalAnimationLength = parts[1].toInt();
      }
    }
  }

  if (!signalImages.isEmpty()) {
    QPixmap &firstImage = signalImages.front();
    signalHeight = firstImage.height();
    signalWidth = firstImage.width();
    totalFrames = signalImages.size();

    if (isGif && signalStyle == "traditional") {
      signalMovement = (width() + signalWidth * 2) / totalFrames;

      signalStyle = "traditional_gif";
    } else {
      signalMovement = 0;
    }
  } else {
    signalAnimationLength = 0;
    signalHeight = 0;
    signalMovement = 0;
    signalWidth = 0;
    totalFrames = 0;

    signalStyle = "None";
  }
}

void FrogPilotAnnotatedCameraWidget::updateState(const FrogPilotUIState &fs, const QJsonObject &frogpilot_toggles) {
  const FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;
  const SubMaster &fpsm = *(fs.sm);

  const cereal::FrogPilotPlan::Reader &frogpilotPlan = fpsm["frogpilotPlan"].getFrogpilotPlan();

  if (!hideBottomIcons && frogpilot_toggles.value("cem_status").toBool()) {
    paintCEMStatus(p, frogpilotPlan, frogpilot_scene, sm);
  } else {
    cemStatusPosition.setX(0);
    cemStatusPosition.setY(0);
  }

  if (!frogpilotPlan.getSpeedLimitChanged() && !(signalStyle == "static" && carState.getLeftBlinker()) && frogpilot_toggles.value("csc_status").toBool()) {
    if (frogpilotPlan.getCscTraining()) {
      paintSmartControllerTraining(p, frogpilotPlan);
    } else {
      glowTimer.invalidate();

      if (isCruiseSet && frogpilotPlan.getCscControllingSpeed()) {
        paintCurveSpeedControl(p, frogpilotPlan);
      }
    }
  } else {
    glowTimer.invalidate();
  }

  static int lastFrameIndex;
  if (lastFrameIndex > animationFrameIndex && frogpilot_toggles.value("signal_icons").toString() == "frog") {
    frogHopCount++;
  }
  lastFrameIndex = animationFrameIndex;

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

  if ((carState.getLeftBlinker() || carState.getRightBlinker()) && signalStyle != "None") {
    if (!animationTimer->isActive()) {
      animationTimer->start(signalAnimationLength);
    }
    paintTurnSignals(p, carState);
  } else if (animationTimer->isActive()) {
    animationTimer->stop();
  }

  if (!hideBottomIcons) {
    paintWeather(p, frogpilotPlan, frogpilot_scene);
  }
}

void FrogPilotAnnotatedCameraWidget::paintCEMStatus(QPainter &p, const cereal::FrogPilotPlan::Reader &frogpilotPlan, FrogPilotUIScene &frogpilot_scene, SubMaster &sm) {
  if (dmIconPosition == QPoint(0, 0)) {
    return;
  }

  p.save();

  cemStatusPosition.rx() = dmIconPosition.x();
  cemStatusPosition.ry() = dmIconPosition.y() - widget_size / 2;
  cemStatusPosition.rx() += (rightHandDM ? -img_size - widget_size : widget_size);

  QRect cemWidget(cemStatusPosition, QSize(widget_size, widget_size));

  p.setBrush(blackColor(166));
  if (frogpilot_scene.conditional_status == 1) {
    p.setPen(QPen(QColor(bg_colors[STATUS_CONDITIONAL_OVERRIDDEN]), 10));
  } else if (frogpilot_scene.enabled && sm["selfdriveState"].getSelfdriveState().getExperimentalMode()) {
    p.setPen(QPen(QColor(bg_colors[STATUS_EXPERIMENTAL_MODE_ENABLED]), 10));
  } else {
    p.setPen(QPen(blackColor(), 10));
  }
  p.drawRoundedRect(cemWidget, 24, 24);

  QSharedPointer<QMovie> icon = chillModeIcon;
  if (frogpilot_scene.enabled && sm["selfdriveState"].getSelfdriveState().getExperimentalMode()) {
    if (frogpilot_scene.conditional_status == 1) {
      icon = chillModeIcon;
    } else if (frogpilot_scene.conditional_status == 2) {
      icon = experimentalModeIcon;
    } else if (frogpilot_scene.conditional_status == 3 || frogpilot_scene.conditional_status == 4) {
      icon = cemSpeedIcon;
    } else if (frogpilot_scene.conditional_status == 5 || frogpilot_scene.conditional_status == 7) {
      icon = cemTurnIcon;
    } else if (frogpilot_scene.conditional_status == 6 || frogpilot_scene.conditional_status == 11 || frogpilot_scene.conditional_status == 12) {
      icon = cemStopIcon;
    } else if (frogpilot_scene.conditional_status == 8) {
      icon = cemCurveIcon;
    } else if (frogpilot_scene.conditional_status == 9 || frogpilot_scene.conditional_status == 10) {
      icon = cemLeadIcon;
    } else {
      icon = experimentalModeIcon;
    }
  }
  p.drawPixmap(cemWidget, icon->currentPixmap());

  p.restore();
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
}

void FrogPilotAnnotatedCameraWidget::paintCurveSpeedControl(QPainter &p, const cereal::FrogPilotPlan::Reader &frogpilotPlan) {
  p.save();

  QRect curveSpeedRect(QPoint(setSpeedRect.right() + UI_BORDER_SIZE, setSpeedRect.top()), QSize(defaultSize.width() * 1.25, defaultSize.width() * 1.25));

  QPixmap curveSpeedImage = frogpilotPlan.getRoadCurvature() < 0 ? curveSpeedIcon : curveSpeedIcon.transformed(QTransform().scale(-1, 1));
  QSize curveSpeedSize = curveSpeedImage.size();
  QPoint curveSpeedPoint(curveSpeedRect.x() + (curveSpeedRect.width()  - curveSpeedSize.width())  / 2, curveSpeedRect.y() + (curveSpeedRect.height() - curveSpeedSize.height()) / 2);

  p.setOpacity(1.0);

  QRect cscRect(curveSpeedRect.topLeft() + QPoint(0, curveSpeedRect.height() + 10), QSize(curveSpeedRect.width(), 100));

  p.setBrush(blueColor(166));
  p.setFont(InterFont(45, QFont::Bold));
  p.setPen(QPen(blueColor(), 10));

  p.drawRoundedRect(cscRect, 24, 24);
  p.setPen(QPen(whiteColor(), 6));
  p.drawText(cscRect.adjusted(20, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft, QString::number(std::nearbyint(fmin(speed, frogpilotPlan.getCscSpeed() * speedConversion))) + speedUnit);

  p.drawPixmap(curveSpeedPoint, curveSpeedImage);

  p.restore();
}

void FrogPilotAnnotatedCameraWidget::paintRainbowPath(QPainter &p, QLinearGradient &bg, float lin_grad_point, SubMaster &sm) {
  p.save();

  static float hueOffset = 0.0;
  if (sm["carState"].getCarState().getVEgo() > 0) {
    hueOffset += powf(sm["carState"].getCarState().getVEgo(), 0.5f) / sqrtf(145.0f / MS_TO_KPH);
  }

  float alpha = util::map_val(lin_grad_point, 0.0f, 1.0f, 0.5f, 0.1f);
  float pathHue = fmodf((lin_grad_point * 360.0f) + hueOffset, 360.0f);

  bg.setColorAt(lin_grad_point, QColor::fromHslF(pathHue / 360.0f, 1.0f, 0.5f, alpha));
  bg.setSpread(QGradient::RepeatSpread);

  p.restore();
}

void FrogPilotAnnotatedCameraWidget::paintSmartControllerTraining(QPainter &p, const cereal::FrogPilotPlan::Reader &frogpilotPlan) {
  p.save();

  if (!glowTimer.isValid()) {
    glowTimer.start();
  }

  QRect curveSpeedRect(QPoint(setSpeedRect.right() + UI_BORDER_SIZE, setSpeedRect.top()), QSize(defaultSize.width() * 1.25, defaultSize.width() * 1.25));
  QPixmap curveSpeedImage = frogpilotPlan.getRoadCurvature() < 0 ? curveSpeedIcon : curveSpeedIcon.transformed(QTransform().scale(-1, 1));

  qreal phase = (glowTimer.elapsed() % 2000) / 2000.0 * 2 * M_PI;
  qreal alphaFactor = 0.5 + 0.5 * sin(phase);

  QColor glowColor = blueColor();
  glowColor.setAlphaF(0.3 + 0.7 * alphaFactor);

  int glowWidth = 8 + static_cast<int>(2 * alphaFactor);

  p.setOpacity(1.0);

  p.setBrush(blackColor(166));
  p.setPen(QPen(glowColor, glowWidth));
  p.drawRoundedRect(curveSpeedRect, 24, 24);

  QSize curveSpeedSize = curveSpeedImage.size();
  QPoint curveSpeedPoint(curveSpeedRect.x() + (curveSpeedRect.width()  - curveSpeedSize.width())  / 2, curveSpeedRect.y() + (curveSpeedRect.height() - curveSpeedSize.height()) / 2);
  p.drawPixmap(curveSpeedPoint, curveSpeedImage);

  QRect textRect(curveSpeedRect.topLeft() + QPoint(0, curveSpeedRect.height() + 10), QSize(curveSpeedRect.width(), 50));
  p.setBrush(blackColor(166));
  p.setPen(QPen(blackColor(), 10));
  p.drawRoundedRect(textRect, 24, 24);

  p.setFont(InterFont(35, QFont::Bold));
  p.setPen(QPen(whiteColor(), 6));
  p.drawText(textRect.adjusted(20, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft, "Training...");

  p.restore();
}

void FrogPilotAnnotatedCameraWidget::paintTurnSignals(QPainter &p, const cereal::CarState::Reader &carState) {
  p.save();

  bool leftBlinker = carState.getLeftBlinker();
  bool blindspotActive = leftBlinker ? carState.getLeftBlindspot() : carState.getRightBlindspot();

  if (signalStyle == "static") {
    int signalXPosition = leftBlinker ? (rect().center().x() * 0.75) - signalWidth : rect().center().x() * 1.25;
    int signalYPosition = signalHeight / 2;

    if (blindspotActive && !blindspotImages.empty()) {
      p.drawPixmap(signalXPosition, signalYPosition, signalWidth, signalHeight, blindspotImages[0].transformed(QTransform().scale(leftBlinker ? 1 : -1, 1)));
    } else {
      p.drawPixmap(signalXPosition, signalYPosition, signalWidth, signalHeight, signalImages[animationFrameIndex].transformed(QTransform().scale(leftBlinker ? 1 : -1, 1)));
    }
  } else {
    int signalXPosition;
    if (signalStyle == "traditional_gif") {
      signalXPosition = leftBlinker ? width() - (animationFrameIndex * signalMovement) + signalWidth : (animationFrameIndex * signalMovement) - signalWidth;
    } else {
      signalXPosition = leftBlinker ? width() - ((animationFrameIndex + 1) * signalWidth) : animationFrameIndex * signalWidth;
    }

    int signalYPosition = height() - signalHeight - alertHeight;

    if (blindspotActive && !blindspotImages.empty()) {
      p.drawPixmap(leftBlinker ? width() - signalWidth : 0, signalYPosition, signalWidth, signalHeight, blindspotImages[0].transformed(QTransform().scale(leftBlinker ? 1 : -1, 1)));
    } else {
      p.drawPixmap(signalXPosition, signalYPosition, signalWidth, signalHeight, signalImages[animationFrameIndex].transformed(QTransform().scale(leftBlinker ? 1 : -1, 1)));
    }
  }

  p.restore();
}

void FrogPilotAnnotatedCameraWidget::paintWeather(QPainter &p, const cereal::FrogPilotPlan::Reader &frogpilotPlan, FrogPilotUIScene &frogpilot_scene) {
  int weatherId = frogpilotPlan.getWeatherId();
  if (weatherId == 0) {
    return;
  }

  p.save();

  QPoint weatherIconPosition;
  if (compassPosition != QPoint(0, 0)) {
    weatherIconPosition = compassPosition;
    weatherIconPosition.rx() += (rightHandDM ? UI_BORDER_SIZE + widget_size + UI_BORDER_SIZE : -UI_BORDER_SIZE - widget_size - UI_BORDER_SIZE);
  } else {
    weatherIconPosition.rx() = rightHandDM ? UI_BORDER_SIZE + widget_size / 2 : width() - UI_BORDER_SIZE - btn_size;
    if (rightHandDM) {
      weatherIconPosition.rx() += btn_size - UI_BORDER_SIZE;
    } else {
      weatherIconPosition.rx() -= btn_size + UI_BORDER_SIZE;
    }
    weatherIconPosition.ry() = dmIconPosition.y() - widget_size / 2;
  }

  QRect weatherRect(weatherIconPosition, QSize(widget_size, widget_size));

  p.setBrush(blackColor(166));
  p.setPen(QPen(blackColor(), 10));
  p.drawRoundedRect(weatherRect, 24, 24);

  QSharedPointer<QMovie> icon = weather_clear_day;
  if ((weatherId >= 200 && weatherId <= 232) || (weatherId >= 300 && weatherId <= 321) || (weatherId >= 500 && weatherId <= 531)) {
    icon = weather_rain;
  } else if (weatherId >= 600 && weatherId <= 622) {
    icon = weather_snow;
  } else if (weatherId == 800) {
    icon = frogpilotPlan.getWeatherDaytime() ? weather_clear_day : weather_clear_night;
  }

  p.drawPixmap(weatherRect, icon->currentPixmap());

  p.restore();
}
