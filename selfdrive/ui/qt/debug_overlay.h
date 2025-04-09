#pragma once

#include <QWidget>
#include <QPixmap>
#include <QTimer>
#include "selfdrive/ui/ui.h"


class DebugOverlay : public QWidget {
  Q_OBJECT

public:
  enum OverlaySide { LEFT, RIGHT };
  explicit DebugOverlay(QWidget *parent = nullptr, OverlaySide side = RIGHT);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;

private:
  void updateValues();
  void drawDebugWindowLeft(QPainter &p, const UIState &s, int x, int y, int w);
  void drawDebugWindowRight(QPainter &p, const UIState &s, int x, int y, int w);

  QPixmap overlayBuffer;
  QTimer *updateTimer = nullptr;
  OverlaySide side;

  bool overlayHidden = false;  // Update the window visibility by clicks
};
