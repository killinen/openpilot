#include "selfdrive/ui/tests/ui_snapshot.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QImage>
#include <QPainter>
#include <QThread>

#include "selfdrive/ui/qt/home.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/window.h"
#include "selfdrive/ui/ui.h"

void saveWidgetAsImage(QWidget *widget, const QString &fileName) {
  QImage image(widget->size(), QImage::Format_ARGB32);
  QPainter painter(&image);
  widget->render(&painter);
  image.save(fileName);
}

int main(int argc, char *argv[]) {
  initApp(argc, argv);

  QApplication app(argc, argv);

  QCommandLineParser parser;
  parser.setApplicationDescription("Take a snapshot of the UI.");
  parser.addHelpOption();
  parser.addOption(QCommandLineOption(QStringList() << "o"
                                                    << "output",
                                      "Output image file path. The file's suffix is used to "
                                      "determine the format. Supports PNG and JPEG formats. "
                                      "Defaults to \"snapshot.png\".",
                                      "file", "snapshot.png"));
  parser.addOption(QCommandLineOption(QStringList() << "c"
                                                    << "case",
                                      "UI case to render (homescreen, settings_device, onroad, ...).",
                                      "case", ""));
  parser.process(app);

  const QString output = parser.value("output");
  if (output.isEmpty()) {
    qCritical() << "No output file specified";
    return 1;
  }
  const QString case_name = parser.value("case");
  const QString effective_case = case_name.isEmpty() ? "homescreen" : case_name;

  auto current = QDir::current();

  // change working directory to find assets
  if (!QDir::setCurrent(QCoreApplication::applicationDirPath() + QDir::separator() + "..")) {
    qCritical() << "Failed to set current directory";
    return 1;
  }

  MainWindow w;
  w.setFixedSize(2160, 1080);
  w.show();
  app.installEventFilter(&w);

  auto apply_case = [&](const QString &name) {
    if (name == "settings_device") {
      w.showSettingsPanelForTesting(0);
    } else if (name == "settings_network") {
      w.showSettingsPanelForTesting(1);
    } else if (name == "onroad_map") {
      w.setMapVisibleForTesting(true);
      w.setSidebarVisibleForTesting(false);
    } else if (name == "onroad_sidebar") {
      w.setMapVisibleForTesting(false);
      w.setSidebarVisibleForTesting(true);
    } else if (name == "onroad") {
      w.setMapVisibleForTesting(false);
      w.setSidebarVisibleForTesting(false);
    } else {
      w.closeSettingsPanelForTesting();
      w.setMapVisibleForTesting(false);
      w.setSidebarVisibleForTesting(true);
    }
  };
  apply_case(effective_case);

  // restore working directory
  QDir::setCurrent(current.absolutePath());

  const bool needs_onroad = effective_case.startsWith("onroad");
  const int max_wait_ms = needs_onroad ? 12000 : 8000;
  QElapsedTimer timer;
  timer.start();
  qInfo() << "ui_snapshot waiting for case" << effective_case;
  while (timer.elapsed() < max_wait_ms) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    QThread::msleep(50);
    const UIState &s = *uiState();
    if (needs_onroad && !s.scene.started) continue;
    if (!needs_onroad && s.scene.started) continue;
    if (s.sm->frame < 5) continue;
    break;
  }
  qInfo() << "ui_snapshot capturing" << effective_case;
  saveWidgetAsImage(&w, output);
  return 0;
}
