#include "debug_overlay.h"
// #include "selfdrive/common/swaglog.h"          //For LOGW
#include <cmath>
#include <QPainter>
#include <QFont>

DebugOverlay::DebugOverlay(QWidget *parent, OverlaySide s) : QWidget(parent), side(s) {
// setAttribute(Qt::WA_TransparentForMouseEvents);  // don't block clicks, if df/ls buttons are used
  setAttribute(Qt::WA_TransparentForMouseEvents, false);  //   Enable mouse clicks, for window toggling
  setFixedSize(300, 1080);  // adjust as needed

  overlayBuffer = QPixmap(size());
  overlayBuffer.fill(Qt::transparent);

  updateTimer = new QTimer(this);
  connect(updateTimer, &QTimer::timeout, this, &DebugOverlay::updateValues);
  updateTimer->start(500);  // 2 Hz
}

void DebugOverlay::paintEvent(QPaintEvent *event) {
  if (overlayHidden) return;  // ✅ skip drawing if hidden
  QPainter p(this);
  // p.fillRect(rect(), QColor(255, 0, 0, 100));  // translucent red box
  p.drawPixmap(0, 0, overlayBuffer);
}

void DebugOverlay::updateValues() {
  UIState *s = uiState();

  // 🖌️ Redraw cache
  overlayBuffer.fill(Qt::transparent);
  QPainter p(&overlayBuffer);

  if (side == RIGHT) {
    drawDebugWindowRight(p, *s, 0, 0, width());
  } else {
    drawDebugWindowLeft(p, *s, 0, 0, width());
  }

  update();  // trigger paintEvent
}

// Window clicky stuff
void DebugOverlay::mousePressEvent(QMouseEvent *event) {
  overlayHidden = !overlayHidden;
  // LOGW("Toggled overlay to %s on %s side", overlayHidden ? "HIDDEN" : "VISIBLE", side == RIGHT ? "RIGHT" : "LEFT");
  update();  // triggers paintEvent

}

void DebugOverlay::drawDebugWindowLeft(QPainter &p, const UIState &s, int x, int y, int w) {

  const UIScene &scene = s.scene;
  int rx = x + (w / 2);
  float dpi_scale = devicePixelRatioF();

  // Fonts
  QFont valueFont("Open Sans", int(10 * dpi_scale), QFont::Bold);
  QFont labelFont("Open Sans", int(7 * dpi_scale), QFont::Normal);
  QFont uomFont("Open Sans", int(7 * dpi_scale), QFont::Normal);

  // Colors
  QColor labelColor(255, 255, 255, 200);
  QColor uomColor(255, 255, 255, 200);

  // Padding and layout
  int topPadding = 40;
  int bottomPadding = 40;
  int yOffset = y + topPadding;
  int usedHeight = 0;

  auto draw = [&](QString val, QString uom, QString label, QColor valColor) {
  int valueToLabelGap = 50;
  int labelToNextRowGap = 25;

  // Value + Unit
  p.setFont(valueFont);
  p.setPen(valColor);
  QFontMetrics valFm(valueFont);
  int valBaseline = yOffset + valFm.ascent();
  int valX = rx - valFm.horizontalAdvance(val) / 2;
  p.drawText(valX, valBaseline, val);


  // Label
  p.setFont(labelFont);
  p.setPen(labelColor);
  QFontMetrics labelFm(labelFont);
  int labelY = valBaseline + valueToLabelGap;
  int labelX = rx - labelFm.horizontalAdvance(label) / 2;
  p.drawText(labelX, labelY, label);

  // Update layout height
  yOffset = labelY + labelToNextRowGap;
  usedHeight = labelY;
  };


  // draw(QString::number(scene.angleOffsetAverageDeg, 'f', 2), "Deg", "ANGLE OFFSET", Qt::white);
  draw(QString::number(scene.steeringTorqueEps, 'f', 1), "", "SSC TRQ", Qt::white);
  // draw(QString::number(scene.aEgo, 'f', 1), "m/s²", "ACCEL", Qt::white);

  draw(QString::number(scene.angleSteers, 'f', 1), "°", "SteerAngle", Qt::white);
  draw(QString::number(scene.angleSteersDes, 'f', 1), "°", "SteerAglDes", Qt::white);


  //draw(QString::number(scene.angleSteersDes, 'f', 1), "°", "SteerAngle", Qt::white);
  //draw(QString::number(scene.angleSteers, 'f', 1), "°", "SteerAglDes", Qt::white);

  // Draw border box
  int totalHeight = (usedHeight - y) + topPadding + bottomPadding;
  p.setPen(QPen(QColor(255, 255, 255, 80), 6));
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(QRect(x, y, w, totalHeight), 20, 20);
}

void DebugOverlay::drawDebugWindowRight(QPainter &p, const UIState &s, int x, int y, int w) {
  const UIScene &scene = s.scene;
  int rx = x + (w / 2);
  float dpi_scale = devicePixelRatioF();

  // Fonts
  QFont valueFont("Open Sans", int(10 * dpi_scale), QFont::Bold);
  QFont labelFont("Open Sans", int(7 * dpi_scale), QFont::Normal);
  QFont uomFont("Open Sans", int(7 * dpi_scale), QFont::Normal);

  // Colors
  QColor labelColor(255, 255, 255, 200);
  QColor uomColor(255, 255, 255, 200);

  // Padding and layout
  int topPadding = 40;
  int bottomPadding = 40;
  int yOffset = y + topPadding;
  int usedHeight = 0;

  auto draw = [&](QString val, QString uom, QString label, QColor valColor) {
    int valueToLabelGap = 50;
    int labelToNextRowGap = 25;

    // Value + Unit
    p.setFont(valueFont);
    p.setPen(valColor);
    QFontMetrics valFm(valueFont);
    int valBaseline = yOffset + valFm.ascent();
    int valX = rx - valFm.horizontalAdvance(val) / 2;
    p.drawText(valX, valBaseline, val);

    // Unit next to value
    p.setFont(uomFont);
    p.setPen(uomColor);
    QFontMetrics uomFm(uomFont);
    int uomX = valX + valFm.horizontalAdvance(val) + 8;
    p.drawText(uomX, valBaseline, uom);

    // Label
    p.setFont(labelFont);
    p.setPen(labelColor);
    QFontMetrics labelFm(labelFont);
    int labelY = valBaseline + valueToLabelGap;
    int labelX = rx - labelFm.horizontalAdvance(label) / 2;
    p.drawText(labelX, labelY, label);

    // Update layout height
    yOffset = labelY + labelToNextRowGap;
    usedHeight = labelY;
  };

  // Draw fields
  // draw(QString::number(scene.cpuTemp, 'f', 1), "°C", "CPU", Qt::white);
  // draw(QString("%1°").arg(round(scene.angleDivergence)), QString("%1%").arg(scene.cpuPerc), "SSC HEALTH", Qt::white);
  draw(QString::number(scene.cpuTemp, 'f', 1), "°C", QString("CPU %1%").arg(scene.cpuPerc), Qt::white);
  // draw(QString::number(scene.cpuTemp, 'f', 1), QString("%1%").arg(scene.cpuPerc), "CPU", Qt::white);
  draw(QString("%1°").arg(round(scene.angleDivergence)), "", "SSC HEALTH", Qt::white);

  //if (scene.gpsAccuracyUblox != 0.0) {
  //  QColor gpsColor = Qt::white;
  //  if (scene.gpsAccuracyUblox > 1.3) gpsColor = Qt::red;
  //  else if (scene.gpsAccuracyUblox > 0.85) gpsColor = QColor(255, 188, 3);

  //  QString gpsVal = "None";
  //  if (scene.gpsAccuracyUblox <= 99 && scene.gpsAccuracyUblox != 0) {
  //    gpsVal = scene.gpsAccuracyUblox > 9.99 ?
  //      QString::number(scene.gpsAccuracyUblox, 'f', 1) :
  //      QString::number(scene.gpsAccuracyUblox, 'f', 2);
  //  }

  //  draw(gpsVal, QString::number(scene.satelliteCount), "GPS PREC", gpsColor);
  //}

  draw(QString::number(scene.angleOffsetAverageDeg, 'f', 2), "Deg", "ANGLE OFFSET", Qt::white);
  draw(QString::number(scene.steeringTorque, 'f', 1), "Nm", "EPS IN TRQ", Qt::white);
  draw(QString::number(scene.steeringTorqueOut, 'f', 1), "Nm", "EPS OUT TRQ", Qt::white);
  // draw(QString::number(scene.aEgo, 'f', 1), "m/s²", "ACCEL", Qt::white);

  // draw(QString::number(scene.angleSteers, 'f', 1), "°", "SteerAngle", Qt::white);
  // draw(QString::number(scene.angleSteersDes, 'f', 1), "°", "SteerAglDes", Qt::white);
  draw(QString::number(scene.pFct, 'f', 3), "", "P-value", Qt::white);
  draw(QString::number(scene.fFct, 'f', 3), "", "F-value", Qt::white);

  // Draw border box
  int totalHeight = (usedHeight - y) + topPadding + bottomPadding;
  p.setPen(QPen(QColor(255, 255, 255, 80), 6));
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(QRect(x, y, w, totalHeight), 20, 20);
}
