#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <cmath>
#include <string>

#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>

#include "common/params.h"
#include "common/util.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "system/hardware/hw.h"


void SoftwarePanel::checkForUpdates() {
  std::system("pkill -SIGUSR1 -f system.updated.updated");
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : ListWidget(parent) {
  onroadLbl = new QLabel(tr("Updates are only downloaded while the car is off or in park."));
  onroadLbl->setStyleSheet("font-size: 50px; font-weight: 400; text-align: left; padding-top: 30px; padding-bottom: 30px;");
  addItem(onroadLbl);

  // current version
  versionLbl = new LabelControl(tr("Current Version"), "");
  addItem(versionLbl);

  // automatic updates toggle
  ParamControl *automaticUpdatesToggle = new ParamControl("AutomaticUpdates", tr("Automatically Update FrogPilot"),
                                                       tr("FrogPilot will automatically update itself and it's assets when you're offroad and have an active internet connection."), "");
  automaticUpdatesToggle->setVisible(params.getBool("IsReleaseBranch"));
  addItem(automaticUpdatesToggle);

  // download update btn
  downloadBtn = new ButtonControl(tr("Download"), tr("CHECK"));
  connect(downloadBtn, &ButtonControl::clicked, [=]() {
    downloadBtn->setEnabled(false);
    if (downloadBtn->text() == tr("CHECK")) {
      checkForUpdates();
    } else {
      std::system("pkill -SIGHUP -f system.updated.updated");
    }
    frogpilotUIState()->params_memory.putBool("ManualUpdateInitiated", true);
  });
  addItem(downloadBtn);

  auto trqiAutomaticToggle = new ParamControl(
    "TrqiAutoInstall", tr("Automatically Update TRQI"),
    tr("While offroad, download production-signed TRQI releases. Installation runs at the next ignition while openpilot remains pre-ONROAD."), "");
  addItem(trqiAutomaticToggle);

  trqiFirmwareBtn = new ButtonControl(
    tr("TRQI Firmware"), tr("CHECK"),
    tr("Download and authenticate both firmware slots now. No CAN command is sent until the next ignition startup hold."));
  connect(trqiFirmwareBtn, &ButtonControl::clicked, [=]() {
    trqiFirmwareBtn->setEnabled(false);
    params.put("TrqiUpdateStatus", "Checking GitHub for production-signed TRQI firmware…");
    params.putBool("TrqiUpdateDownloadRequest", true);
  });
  addItem(trqiFirmwareBtn);

  trqiStatusLabel = new LabelControl(tr("TRQI Update Status"), "");
  trqiStatusLabel->setVisible(false);
  addItem(trqiStatusLabel);

  trqiProgressBar = new QProgressBar(this);
  trqiProgressBar->setRange(0, 1000);
  trqiProgressBar->setFixedHeight(58);
  trqiProgressBar->setTextVisible(true);
  trqiProgressBar->setStyleSheet(R"(
    QProgressBar { border: 2px solid #555; border-radius: 18px; background: #222; color: white;
                   font-size: 30px; font-weight: 600; text-align: center; }
    QProgressBar::chunk { border-radius: 15px; background-color: #2ECC71; }
  )");
  trqiProgressBar->setVisible(false);
  addItem(trqiProgressBar);

  // install update btn
  installBtn = new ButtonControl(tr("Install Update"), tr("INSTALL"));
  connect(installBtn, &ButtonControl::clicked, [=]() {
    installBtn->setEnabled(false);
    params.putBool("DoReboot", true);
  });
  addItem(installBtn);

  // branch selecting
  targetBranchBtn = new ButtonControl(tr("Target Branch"), tr("SELECT"));
  connect(targetBranchBtn, &ButtonControl::clicked, [=]() {
    auto current = params.get("GitBranch");
    QStringList branches = QString::fromStdString(params.get("UpdaterAvailableBranches")).split(",");
    if (!frogpilotUIState()->frogpilot_scene.frogpilot_toggles.value("frogs_go_moo").toBool()) {
      for (int i = branches.size() - 1; i >= 0; --i) {
        if (branches[i].startsWith("FrogPilot-Development", Qt::CaseInsensitive)) {
          branches.removeAt(i);
        }
      }
    }
    branches.removeAll("FrogPilot-Vetting");
    branches.removeAll("MAKE-PRS-HERE");
    for (QString b : {current.c_str(), "devel-staging", "devel", "nightly", "master-ci", "master"}) {
      auto i = branches.indexOf(b);
      if (i >= 0) {
        branches.removeAt(i);
        branches.insert(0, b);
      }
    }

    QString cur = QString::fromStdString(params.get("UpdaterTargetBranch"));
    QString selection = MultiOptionDialog::getSelection(tr("Select a branch"), branches, cur, this);
    if (!selection.isEmpty()) {
      params.put("UpdaterTargetBranch", selection.toStdString());
      targetBranchBtn->setValue(QString::fromStdString(params.get("UpdaterTargetBranch")));
      checkForUpdates();

      if (selection.toStdString() != current) {
        if (FrogPilotConfirmationDialog::yesorno(tr("This branch must be downloaded before switching. Would you like to download it now?"), this)) {
          std::system("pkill -SIGHUP -f system.updated.updated");

          frogpilotUIState()->params_memory.putBool("ManualUpdateInitiated", true);
        }
      }
    }
  });
  addItem(targetBranchBtn);

  // uninstall button
  auto uninstallBtn = new ButtonControl(tr("Uninstall %1").arg(getBrand()), tr("UNINSTALL"));
  connect(uninstallBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to uninstall?"), tr("Uninstall"), this)) {
      if (FrogPilotConfirmationDialog::yesorno(tr("Do you want to perform a full factory reset? All saved assets and settings will be permanently deleted!"), this)) {
        if (FrogPilotConfirmationDialog::yesorno(tr("This is a complete factory reset and cannot be undone. Are you absolutely sure you want to continue?"), this)) {
          std::system("rm -rf /cache/params/d");
        }
      }
      params.putBool("DoUninstall", true);
    }
  });
  addItem(uninstallBtn);

  // error log button
  auto errorLogBtn = new ButtonControl(tr("Error Log"), tr("VIEW"), tr("View the error log for openpilot crashes."));
  connect(errorLogBtn, &ButtonControl::clicked, [=]() {
    std::string txt = util::read_file("/data/error_logs/error.txt");
    ConfirmationDialog::rich(QString::fromStdString(txt), this);
  });
  addItem(errorLogBtn);

  fs_watch = new ParamWatcher(this);
  QObject::connect(fs_watch, &ParamWatcher::paramChanged, [=](const QString &param_name, const QString &param_value) {
    updateLabels();
  });

  connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    is_onroad = !offroad;
    updateLabels();
  });

  updateLabels();
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  // nice for testing on PC
  installBtn->setEnabled(true);

  updateLabels();

  // FrogPilot variables
  FrogPilotUIState &fs = *frogpilotUIState();
  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;

  if (frogpilot_scene.online && params.get("UpdaterState") == "idle") {
    checkForUpdates();
  }
}

void SoftwarePanel::updateLabels() {
  FrogPilotUIState &fs = *frogpilotUIState();
  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;

  // add these back in case the files got removed
  fs_watch->addParam("LastUpdateTime");
  fs_watch->addParam("UpdateFailedCount");
  fs_watch->addParam("UpdaterState");
  fs_watch->addParam("UpdateAvailable");
  fs_watch->addParam("TrqiUpdateDownloadRequest");
  fs_watch->addParam("TrqiUpdatePending");
  fs_watch->addParam("TrqiUpdateStatus");
  fs_watch->addParam("TrqiUpdateProgress");

  if (!isVisible()) {
    frogpilot_scene.downloading_update = false;
    return;
  }

  // The generic updater may be used while parked ONROAD. TRQI controls remain
  // strictly OFFROAD because the managed TRQI process is stopped ONROAD.
  bool parked = frogpilot_scene.parked || frogpilot_scene.frogpilot_toggles.value("frogs_go_moo").toBool();

  onroadLbl->setVisible(is_onroad && !parked);
  downloadBtn->setVisible(!is_onroad || parked);
  trqiFirmwareBtn->setVisible(!is_onroad);

  // download update
  QString updater_state = QString::fromStdString(params.get("UpdaterState"));
  bool failed = std::atoi(params.get("UpdateFailedCount").c_str()) > 0;
  if (updater_state != "idle") {
    downloadBtn->setEnabled(false);
    downloadBtn->setValue(updater_state);
    frogpilot_scene.downloading_update = true;
  } else {
    frogpilot_scene.downloading_update = false;
    if (failed) {
      downloadBtn->setText(tr("CHECK"));
      downloadBtn->setValue(tr("failed to check for update"));
    } else if (params.getBool("UpdaterFetchAvailable")) {
      downloadBtn->setText(tr("DOWNLOAD"));
      downloadBtn->setValue(tr("update available"));
    } else {
      QString lastUpdate = tr("never");
      auto tm = params.get("LastUpdateTime");
      if (!tm.empty()) {
        lastUpdate = timeAgo(QDateTime::fromString(QString::fromStdString(tm + "Z"), Qt::ISODate));
      }
      downloadBtn->setText(tr("CHECK"));
      downloadBtn->setValue(tr("up to date, last checked %1").arg(lastUpdate));
    }
    downloadBtn->setEnabled(true);
  }
  targetBranchBtn->setValue(QString::fromStdString(params.get("UpdaterTargetBranch")));

  const bool trqi_checking = params.getBool("TrqiUpdateDownloadRequest");
  const bool trqi_pending = params.getBool("TrqiUpdatePending");
  const bool trqi_in_progress = params.getBool("TrqiUpdateInProgress");
  const QString trqi_status = QString::fromStdString(params.get("TrqiUpdateStatus"));
  trqiFirmwareBtn->setEnabled(!trqi_checking && !trqi_pending && !trqi_in_progress);
  if (trqi_checking) {
    trqiFirmwareBtn->setText(tr("CHECKING"));
  } else if (trqi_in_progress) {
    trqiFirmwareBtn->setText(tr("INSTALLING"));
  } else if (trqi_pending) {
    trqiFirmwareBtn->setText(tr("READY"));
  } else {
    trqiFirmwareBtn->setText(tr("CHECK"));
  }
  trqiFirmwareBtn->setValue(trqi_status);

  const QByteArray progress_raw = QByteArray::fromStdString(params.get("TrqiUpdateProgress"));
  const QJsonObject progress = QJsonDocument::fromJson(progress_raw).object();
  const QString stage = progress.value("stage").toString();
  const QString trust = progress.value("trust").toString("production");
  const double percent = progress.value("percent").toDouble(-1.0);
  const bool transferring = stage == "transfer" && percent >= 0.0;
  trqiProgressBar->setVisible(transferring && !is_onroad);
  if (transferring) {
    trqiProgressBar->setValue(std::lround(percent * 10.0));
    const double durable_kib = progress.value("durable_offset").toDouble() / 1024.0;
    const double total_kib = progress.value("total").toDouble() / 1024.0;
    const double rate = progress.value("throughput_kib_s").toDouble();
    const int retries = progress.value("retries").toInt();
    const int timeouts = progress.value("timeouts").toInt();
    trqiProgressBar->setFormat(QString("%1%  •  %2/%3 KiB  •  %4 KiB/s  •  retry %5  timeout %6")
      .arg(percent, 0, 'f', 1).arg(durable_kib, 0, 'f', 1).arg(total_kib, 0, 'f', 1)
      .arg(rate, 0, 'f', 1).arg(retries).arg(timeouts));
  }

  const bool show_trqi_status = !stage.isEmpty() || !trqi_status.isEmpty();
  trqiStatusLabel->setVisible(show_trqi_status && !is_onroad);
  if (show_trqi_status) {
    const QString badge = trust == "test" ? tr("TEST KEY — NOT PRODUCTION") : tr("PRODUCTION SIGNATURE");
    const QString active = progress.value("active_slot").toString("?");
    const QString target = progress.value("target_slot").toString("?");
    const QString confirmation = progress.value("confirmation").toString("-");
    trqiStatusLabel->setText(QString("%1  •  %2").arg(stage.toUpper(), badge));
    trqiStatusLabel->setDescription(QString("%1\nActive slot %2  →  target slot %3  •  %4")
      .arg(trqi_status, active, target, confirmation));
  }

  // current + new versions
  versionLbl->setText(QString::fromStdString(params.get("UpdaterCurrentDescription")));
  versionLbl->setDescription(QString::fromStdString(params.get("UpdaterCurrentReleaseNotes")));

  installBtn->setVisible((!is_onroad || parked) && params.getBool("UpdateAvailable"));
  installBtn->setValue(QString::fromStdString(params.get("UpdaterNewDescription")));
  installBtn->setDescription(QString::fromStdString(params.get("UpdaterNewReleaseNotes")));

  update();
}
