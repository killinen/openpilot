#include "selfdrive/ui/ui.h"

#include "frogpilot/ui/frogpilot_ui.h"

bool FrogPilotConfirmationDialog::toggleReboot(QWidget *parent) {
  ConfirmationDialog d(tr("Reboot required to take effect."), tr("Reboot Now"), tr("Reboot Later"), false, parent);
  return d.exec();
}

bool FrogPilotConfirmationDialog::yesorno(const QString &prompt_text, QWidget *parent) {
  ConfirmationDialog d(prompt_text, tr("Yes"), tr("No"), false, parent);
  return d.exec();
}

void loadGif(const QString &gifPath, QSharedPointer<QMovie> &movie, const QSize &size, QWidget *parent) {
  if (!movie.isNull()) {
    QObject::disconnect(movie.data(), nullptr, parent, nullptr);
    movie->stop();
    movie.clear();
  }

  if (QFileInfo::exists(gifPath)) {
    movie = QSharedPointer<QMovie>::create(gifPath, QByteArray(), parent);
    movie->setCacheMode(QMovie::CacheAll);
    movie->setScaledSize(size);
    QObject::connect(movie.data(), &QMovie::frameChanged, parent,[parent](int) { parent->update(); }, Qt::UniqueConnection);
    movie->start();
  }

  parent->update();
}

void loadImage(const QString &basePath, QPixmap &pixmap, QSharedPointer<QMovie> &movie, const QSize &size, QWidget *parent, Qt::AspectRatioMode aspectRatioMode) {
  QString gifPath = basePath + ".gif";
  if (QFileInfo::exists(gifPath)) {
    loadGif(gifPath, movie, size, parent);
  } else {
    if (!movie.isNull()) {
      QObject::disconnect(movie.data(), nullptr, parent, nullptr);
      movie->stop();
      movie.clear();
    }
    pixmap = QPixmap(basePath + ".png").scaled(size, aspectRatioMode, Qt::SmoothTransformation);
    parent->update();
  }
}

void openDescriptions(bool forceOpenDescriptions, std::map<QString, AbstractControl*> toggles) {
  if (forceOpenDescriptions) {
    for (auto &[key, toggle] : toggles) {
      if (key != "CESpeed") {
        toggle->showDescription();
      }
    }
  }
}
