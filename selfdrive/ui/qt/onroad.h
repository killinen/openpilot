#pragma once

#include <QStackedLayout>
#include <QWidget>
#include <QPushButton>

#include "selfdrive/ui/qt/widgets/cameraview.h"
#include "selfdrive/ui/ui.h"


// ***** onroad widgets *****

class ButtonsWindow : public QWidget {
  Q_OBJECT

public:
  ButtonsWindow(QWidget* parent = 0);

private:
  QPushButton *dfButton;
  QPushButton *lsButton;
  QPushButton *mlButton;

  int dfStatus = -1;  // always initialize style sheet and send msg
  const QStringList dfButtonColors = {"#044389", "#24a8bc", "#fcff4b", "#37b868"};

  int lsStatus = -1;  // always initialize style sheet and send msg
  const QStringList lsButtonColors = {"#ff3737", "#37b868", "#044389"};

  // model long button
  bool mlEnabled = true;  // triggers initialization
  const QStringList mlButtonColors = {"#b83737", "#37b868"};

public slots:
  void updateState(const UIState &s);
};

class OnroadHud : public QWidget {
  Q_OBJECT
  Q_PROPERTY(QString speed MEMBER speed NOTIFY valueChanged);
  Q_PROPERTY(QString speedUnit MEMBER speedUnit NOTIFY valueChanged);
  Q_PROPERTY(QString maxSpeed MEMBER maxSpeed NOTIFY valueChanged);
  Q_PROPERTY(bool is_cruise_set MEMBER is_cruise_set NOTIFY valueChanged);
  Q_PROPERTY(bool engageable MEMBER engageable NOTIFY valueChanged);
  Q_PROPERTY(bool dmActive MEMBER dmActive NOTIFY valueChanged);
  Q_PROPERTY(bool hideDM MEMBER hideDM NOTIFY valueChanged);
  Q_PROPERTY(int status MEMBER status NOTIFY valueChanged);

  Q_PROPERTY(bool showHowAlert MEMBER showHowAlert NOTIFY valueChanged);
  Q_PROPERTY(bool howWarning MEMBER howWarning NOTIFY valueChanged);

  Q_PROPERTY(bool showVTC MEMBER showVTC NOTIFY valueChanged);
  Q_PROPERTY(QString vtcSpeed MEMBER vtcSpeed NOTIFY valueChanged);
  Q_PROPERTY(QColor vtcColor MEMBER vtcColor NOTIFY valueChanged);
  Q_PROPERTY(bool showDebugUI MEMBER showDebugUI NOTIFY valueChanged);

  Q_PROPERTY(QString roadName MEMBER roadName NOTIFY valueChanged);

  Q_PROPERTY(bool showSpeedLimit MEMBER showSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString speedLimit MEMBER speedLimit NOTIFY valueChanged);
  Q_PROPERTY(QString slcSubText MEMBER slcSubText NOTIFY valueChanged);
  Q_PROPERTY(float slcSubTextSize MEMBER slcSubTextSize NOTIFY valueChanged);
  Q_PROPERTY(bool mapSourcedSpeedLimit MEMBER mapSourcedSpeedLimit NOTIFY valueChanged);
  Q_PROPERTY(bool slcActive MEMBER slcActive NOTIFY valueChanged);

public:
  explicit OnroadHud(QWidget *parent);
  void updateState(const UIState &s);

private:
  void drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity);
  void drawText(QPainter &p, int x, int y, const QString &text, int alpha = 255);
  void drawCircle(QPainter &p, int x, int y, int r, QBrush bg);
  void drawSpeedSign(QPainter &p, QRect rc, const QString &speed, const QString &sub_text, int subtext_size, 
                     bool is_map_sourced, bool is_active);
  void paintEvent(QPaintEvent *event) override;

  QPixmap engage_img;
  QPixmap dm_img;
  QPixmap how_img;
  QPixmap map_img;
  const int radius = 192;
  const int img_size = (radius / 2) * 1.5;
  const int subsign_img_size = 35;
  QString speed;
  QString speedUnit;
  QString maxSpeed;
  bool is_cruise_set = false;
  bool engageable = false;
  bool dmActive = false;
  bool hideDM = false;
  int status = STATUS_DISENGAGED;

  bool showHowAlert = false;
  bool howWarning = false;

  bool showVTC = false;
  QString vtcSpeed;
  QColor vtcColor;
  bool showDebugUI = false;
  
  QString roadName;

  bool showSpeedLimit = false;
  QString speedLimit;
  QString slcSubText;
  float slcSubTextSize = 0.0;
  bool mapSourcedSpeedLimit = false;
  bool slcActive = false;

signals:
  void valueChanged();
};

class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0) : QWidget(parent) {};
  void updateAlert(const Alert &a, const QColor &color);

protected:
  void paintEvent(QPaintEvent*) override;

private:
  QColor bg;
  Alert alert = {};
};

// container window for the NVG UI
class NvgWindow : public CameraViewWidget {
  Q_OBJECT

public:
  explicit NvgWindow(VisionStreamType type, QWidget* parent = 0) : CameraViewWidget("camerad", type, true, parent) {}
  int prev_width = -1;  // initializes ButtonsWindow width and holds prev width to update it

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  void updateFrameMat(int w, int h) override;
  void drawLaneLines(QPainter &painter, UIState *s);
  void drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd);
  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  double prev_draw_t = 0;

signals:
  void resizeSignal(int w);
};

// container for all onroad widgets
class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }

private:
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadHud *hud;
  OnroadAlerts *alerts;
  NvgWindow *nvg;
  ButtonsWindow *buttons;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;

private slots:
  void offroadTransition(bool offroad);
  void updateState(const UIState &s);
};
