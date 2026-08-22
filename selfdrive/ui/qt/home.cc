#include "selfdrive/ui/qt/home.h"

#include <cmath>

#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <QProgressBar>
#include <QStackedWidget>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/offroad/experimental_mode.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/prime.h"

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "frogpilot/ui/qt/widgets/drive_stats.h"
#include "frogpilot/ui/qt/widgets/drive_summary.h"
#include "frogpilot/ui/qt/widgets/model_reviewer.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome(this);
  QObject::connect(home, &OffroadHome::openSettings, this, &HomeWindow::openSettings);
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  QObject::connect(onroad, &OnroadWindow::mapPanelRequested, this, [=] { sidebar->hide(); });
  slayout->addWidget(onroad);

  body = new BodyWindow(this);
  slayout->addWidget(body);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &HomeWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::offroadTransition, sidebar, &Sidebar::offroadTransition);

  // FrogPilot variables
  developer_sidebar = new DeveloperSidebar(this);
  main_layout->addWidget(developer_sidebar);
  developer_sidebar->setVisible(false);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::showMapPanel(bool show) {
  onroad->showMapPanel(show);
}

void HomeWindow::updateState(const UIState &s, const FrogPilotUIState &fs) {
  const SubMaster &sm = *(s.sm);

  // switch to the generic robot UI
  if (onroad->isVisible() && !body->isEnabled() && sm["carParams"].getCarParams().getNotCar()) {
    body->setEnabled(true);
    slayout->setCurrentWidget(body);
  }

  // FrogPilot variables
  if (s.scene.started) {
    if (fs.frogpilot_scene.driver_camera_timer >= UI_FREQ / 2) {
      showDriverView(true, true);
    } else {
      if (driver_view->isVisible()) {
        sidebar->setVisible(params.getBool("Sidebar") || frogpilotUIState()->frogpilot_toggles.value("debug_mode").toBool());
        slayout->setCurrentWidget(onroad);
      }

      if (fs.frogpilot_scene.map_open) {
        showSidebar(false);
      }

      developer_sidebar->setVisible(fs.frogpilot_toggles.value("developer_sidebar").toBool());

      frogpilotUIState()->frogpilot_scene.sidebars_open = developer_sidebar->isVisible() && sidebar->isVisible();
    }
  }
}

void HomeWindow::offroadTransition(bool offroad) {
  body->setEnabled(false);
  sidebar->setVisible(offroad || params.getBool("Sidebar") || frogpilotUIState()->frogpilot_toggles.value("debug_mode").toBool());
  if (offroad) {
    developer_sidebar->setVisible(false);

    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
}

void HomeWindow::showDriverView(bool show, bool started) {
  if (show) {
    if (!started) {
      emit closeSettings();
    }
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  developer_sidebar->setVisible(false);
  sidebar->setVisible(show == false);
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  if ((onroad->isVisible() || body->isVisible()) && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
    params.putBool("Sidebar", sidebar->isVisible());
  }
}

void HomeWindow::mouseDoubleClickEvent(QMouseEvent* e) {
  HomeWindow::mousePressEvent(e);
  const SubMaster &sm = *(uiState()->sm);
  if (sm["carParams"].getCarParams().getNotCar()) {
    if (onroad->isVisible()) {
      slayout->setCurrentWidget(body);
    } else if (body->isVisible()) {
      slayout->setCurrentWidget(onroad);
    }
    showSidebar(false);
  }
}

// OffroadHome: the offroad home page

class TrqiUpdateWidget : public QFrame {
  Q_DECLARE_TR_FUNCTIONS(TrqiUpdateWidget)

public:
  explicit TrqiUpdateWidget(QWidget *parent = nullptr) : QFrame(parent) {
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(70, 55, 70, 55);
    layout->setSpacing(24);

    QLabel *title = new QLabel(tr("Updating TRQI Firmware"));
    title->setStyleSheet("font-size: 64px; font-weight: 600;");
    layout->addWidget(title);

    badge = new QLabel;
    badge->setAlignment(Qt::AlignCenter);
    badge->setFixedHeight(64);
    layout->addWidget(badge, 0, Qt::AlignLeft);

    stage = new QLabel;
    stage->setStyleSheet("font-size: 42px; font-weight: 600; color: #A0A0A0;");
    layout->addWidget(stage);

    status = new QLabel;
    status->setWordWrap(true);
    status->setStyleSheet("font-size: 44px;");
    layout->addWidget(status);

    progress = new QProgressBar;
    progress->setFixedHeight(72);
    progress->setTextVisible(true);
    progress->setStyleSheet(R"(
      QProgressBar { border: 2px solid #555; border-radius: 22px; background: #202020;
                     color: white; font-size: 32px; font-weight: 600; text-align: center; }
      QProgressBar::chunk { border-radius: 19px; background-color: #2ECC71; }
    )");
    layout->addWidget(progress);

    details = new QLabel;
    details->setWordWrap(true);
    details->setStyleSheet("font-size: 34px; color: #C8C8C8;");
    layout->addWidget(details);
    layout->addStretch();

    QLabel *warning = new QLabel(tr("Keep ignition on. openpilot will start automatically when the update is safely confirmed."));
    warning->setWordWrap(true);
    warning->setStyleSheet("font-size: 34px; color: #F5C451;");
    layout->addWidget(warning);

    setStyleSheet("TrqiUpdateWidget { background-color: #292929; border-radius: 30px; }");
  }

  bool refresh() {
    if (!params.getBool("TrqiUpdateStartupHold") && !params.getBool("TrqiUpdateInProgress")) {
      return false;
    }

    // The pre-ONROAD safety hold intentionally leaves scene.started false, so
    // the normal ONROAD wakefulness rule does not apply. Keep the display awake
    // while this card is active instead of allowing the OFFROAD timeout to
    // blank firmware progress.
    device()->resetInteractiveTimeout();

    const QJsonObject payload = QJsonDocument::fromJson(
      QByteArray::fromStdString(params.get("TrqiUpdateProgress"))).object();
    const QString trust = payload.value("trust").toString("production");
    const QString stage_name = payload.value("stage").toString("preflight").toUpper();
    const QString message = QString::fromStdString(params.get("TrqiUpdateStatus"));
    const QString active_slot = payload.value("active_slot").toString("?");
    const QString target_slot = payload.value("target_slot").toString("?");
    const QString confirmation = payload.value("confirmation").toString("-");
    const double percent = payload.value("percent").toDouble(-1.0);
    const bool transferring = payload.value("stage").toString() == "transfer" && percent >= 0.0;

    badge->setText(trust == "test" ? tr("TEST KEY — NOT PRODUCTION") : tr("PRODUCTION SIGNATURE"));
    badge->setStyleSheet(QString("font-size: 30px; font-weight: 700; padding: 8px 24px; "
                                 "border-radius: 16px; background-color: %1;")
                           .arg(trust == "test" ? "#C97A16" : "#237A45"));
    stage->setText(tr("STAGE  •  %1").arg(stage_name));
    status->setText(message.isEmpty() ? tr("Preparing the firmware update…") : message);

    if (transferring) {
      progress->setRange(0, 1000);
      progress->setValue(std::lround(percent * 10.0));
      const double durable_kib = payload.value("durable_offset").toDouble() / 1024.0;
      const double total_kib = payload.value("total").toDouble() / 1024.0;
      const double rate = payload.value("throughput_kib_s").toDouble();
      progress->setFormat(QString("%1%  •  %2/%3 KiB  •  %4 KiB/s")
                            .arg(percent, 0, 'f', 1).arg(durable_kib, 0, 'f', 1)
                            .arg(total_kib, 0, 'f', 1).arg(rate, 0, 'f', 1));
    } else {
      progress->setRange(0, 0);
      progress->setFormat(QString());
    }

    const int retries = payload.value("retries").toInt();
    const int timeouts = payload.value("timeouts").toInt();
    details->setText(tr("Active slot %1  →  target slot %2  •  %3\nRetries %4  •  timeouts %5")
                       .arg(active_slot, target_slot, confirmation).arg(retries).arg(timeouts));
    return true;
  }

private:
  Params params;
  QLabel *badge;
  QLabel *stage;
  QLabel *status;
  QLabel *details;
  QProgressBar *progress;
};

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 40);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(16);

  update_notif = new QPushButton(tr("UPDATE"));
  update_notif->setVisible(false);
  update_notif->setStyleSheet("background-color: #364DEF;");
  QObject::connect(update_notif, &QPushButton::clicked, [=]() { center_layout->setCurrentIndex(1); });
  header_layout->addWidget(update_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  alert_notif = new QPushButton();
  alert_notif->setVisible(false);
  alert_notif->setStyleSheet("background-color: #E22C2C;");
  QObject::connect(alert_notif, &QPushButton::clicked, [=] { center_layout->setCurrentIndex(2); });
  header_layout->addWidget(alert_notif, 0, Qt::AlignHCenter | Qt::AlignLeft);

  date = new ElidedLabel();
  header_layout->addWidget(date, 0, Qt::AlignHCenter | Qt::AlignLeft);

  version = new ElidedLabel();
  header_layout->addWidget(version, 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget *home_widget = new QWidget(this);
  {
    QHBoxLayout *home_layout = new QHBoxLayout(home_widget);
    home_layout->setContentsMargins(0, 0, 0, 0);
    home_layout->setSpacing(30);

    // left: MapSettings
    QStackedWidget *left_widget = new QStackedWidget(this);
#ifdef ENABLE_MAPS
    left_widget->addWidget(new MapSettings);
#else
    left_widget->addWidget(new QWidget);
#endif
    left_widget->addWidget(new DriveStats);

    FrogPilotDriveSummary *drive_summary = new FrogPilotDriveSummary(this);
    left_widget->addWidget(drive_summary);

    FrogPilotModelReview *model_review = new FrogPilotModelReview(this);
    left_widget->addWidget(model_review);

    left_widget->setStyleSheet("border-radius: 10px;");
    left_widget->setCurrentIndex(1);

    connect(drive_summary, &FrogPilotDriveSummary::panelClosed, [=]() {
      left_widget->setCurrentIndex(1);
    });
    connect(model_review, &FrogPilotModelReview::driveRated, [=]() {
      left_widget->setCurrentIndex(2);
    });
    connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
      static bool previouslyOnroad = false;
      if (offroad && previouslyOnroad) {
        left_widget->setCurrentIndex(2);
      }
      previouslyOnroad = !offroad;
    });
    connect(frogpilotUIState(), &FrogPilotUIState::reviewModel, [=]() {
      left_widget->setCurrentIndex(3);
    });

    home_layout->addWidget(left_widget, 1);

    // right: ExperimentalModeButton, SetupWidget
    QStackedWidget *right_widget = new QStackedWidget(this);
    right_widget->setFixedWidth(750);

    QWidget *default_right = new QWidget(this);
    QVBoxLayout *default_layout = new QVBoxLayout(default_right);
    default_layout->setContentsMargins(0, 0, 0, 0);
    default_layout->setSpacing(30);

    ExperimentalModeButton *experimental_mode = new ExperimentalModeButton(this);
    QObject::connect(experimental_mode, &ExperimentalModeButton::openSettings, this, &OffroadHome::openSettings);
    default_layout->addWidget(experimental_mode, 1);

    SetupWidget *setup_widget = new SetupWidget;
    QObject::connect(setup_widget, &SetupWidget::openSettings, this, &OffroadHome::openSettings);
    default_layout->addWidget(setup_widget, 1);

    right_widget->addWidget(default_right);

    FrogPilotDriveSummary *random_events_summary = new FrogPilotDriveSummary(this, true);
    right_widget->addWidget(random_events_summary);

    right_widget->setCurrentIndex(0);

    connect(random_events_summary, &FrogPilotDriveSummary::panelClosed, [=]() {
      right_widget->setCurrentIndex(0);
    });
    connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
      static bool previouslyOnroad = false;
      if (offroad && previouslyOnroad && frogpilotUIState()->frogpilot_toggles.value("random_events").toBool()) {
        right_widget->setCurrentIndex(1);
      }
      previouslyOnroad = !offroad;
    });

    home_layout->addWidget(right_widget, 1);
  }
  center_layout->addWidget(home_widget);

  // add update & alerts widgets
  update_widget = new UpdateAlert();
  QObject::connect(update_widget, &UpdateAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(update_widget);
  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::dismiss, [=]() { center_layout->setCurrentIndex(0); });
  center_layout->addWidget(alerts_widget);
  trqi_update_widget = new TrqiUpdateWidget;
  center_layout->addWidget(trqi_update_widget);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);
  trqi_timer = new QTimer(this);
  trqi_timer->callOnTimeout(this, &OffroadHome::refreshTrqi);

  setStyleSheet(R"(
    * {
      color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
  trqi_timer->start(250);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
  trqi_timer->stop();
}

void OffroadHome::refresh() {
  date->setText(QLocale(uiState()->language.mid(5)).toString(QDateTime::currentDateTime(), "dddd, MMMM d"));
  date->setVisible(util::system_time_valid());

  version->setText(getBrand() + " v" + getVersion().left(14).trimmed() + " - " + processModelName(frogpilotUIState()->frogpilot_toggles.value("model_name").toString()));

  bool updateAvailable = update_widget->refresh();
  int alerts = alerts_widget->refresh();

  if (trqi_update_widget->refresh()) {
    center_layout->setCurrentWidget(trqi_update_widget);
    return;
  }

  // pop-up new notification
  int idx = center_layout->currentIndex();
  if (!updateAvailable && !alerts) {
    idx = 0;
  } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
    idx = 1;
  } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
    idx = 2;
  }
  center_layout->setCurrentIndex(idx);

  update_notif->setVisible(updateAvailable);
  alert_notif->setVisible(alerts);
  if (alerts) {
    alert_notif->setText(QString::number(alerts) + (alerts > 1 ? tr(" ALERTS") : tr(" ALERT")));
  }
}

void OffroadHome::refreshTrqi() {
  if (trqi_update_widget->refresh()) {
    center_layout->setCurrentWidget(trqi_update_widget);
  } else if (center_layout->currentWidget() == trqi_update_widget) {
    center_layout->setCurrentIndex(0);
  }
}
