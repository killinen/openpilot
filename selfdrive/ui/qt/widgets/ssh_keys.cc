#include "selfdrive/ui/qt/widgets/ssh_keys.h"

#include <string>

#include "common/params.h"
#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/widgets/input.h"

// GoranConnect keys that launch_chffrplus.sh (ensure_goranconnect_ssh_material)
// adds at boot. Keep them when the user adds or removes GitHub keys so GCS
// access is not lost until the next reboot. Keep in sync with that script.
static const QStringList GORANCONNECT_KEYS = {
  "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICDlC6fGsSjI7ZjAPglJA2QTKzPfieSpVHBgkqEDm5xO",
  "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIL20WPVC09JAplIWBCd61vreHZ5BZTjnZVlBb/KB+whT",
};

static std::string withGoranConnectKeys(const QString &keys) {
  QStringList lines;
  for (const QString &line : keys.split('\n')) {
    if (!line.trimmed().isEmpty()) {
      lines.append(line.trimmed());
    }
  }
  for (const QString &key : GORANCONNECT_KEYS) {
    if (!lines.contains(key)) {
      lines.append(key);
    }
  }
  return (lines.join('\n') + '\n').toStdString();
}

SshControl::SshControl() :
  ButtonControl(tr("SSH Keys"), "", tr("Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username "
                                       "other than your own. A comma employee will NEVER ask you to add their GitHub username.")) {

  QObject::connect(this, &ButtonControl::clicked, [=]() {
    if (text() == tr("ADD")) {
      QString username = InputDialog::getText(tr("Enter your GitHub username"), this);
      if (username.length() > 0) {
        setText(tr("LOADING"));
        setEnabled(false);
        getUserKeys(username);
      }
    } else {
      params.remove("GithubUsername");
      params_cache.remove("GithubUsername");
      params.put("GithubSshKeys", withGoranConnectKeys(""));
      params_cache.remove("GithubSshKeys");
      refresh();
    }
  });

  refresh();
}

void SshControl::refresh() {
  // The key file always holds the GoranConnect keys, so the username tells
  // whether the user has added their own GitHub keys.
  QString username = QString::fromStdString(params.get("GithubUsername"));
  if (username.length()) {
    setValue(username);
    setText(tr("REMOVE"));
  } else {
    setValue("");
    setText(tr("ADD"));
  }
  setEnabled(true);
}

void SshControl::getUserKeys(const QString &username) {
  HttpRequest *request = new HttpRequest(this, false);
  QObject::connect(request, &HttpRequest::requestDone, [=](const QString &resp, bool success) {
    if (success) {
      if (!resp.isEmpty()) {
        params.put("GithubUsername", username.toStdString());
        params.put("GithubSshKeys", withGoranConnectKeys(resp));
      } else {
        ConfirmationDialog::alert(tr("Username '%1' has no keys on GitHub").arg(username), this);
      }
    } else {
      if (request->timeout()) {
        ConfirmationDialog::alert(tr("Request timed out"), this);
      } else {
        ConfirmationDialog::alert(tr("Username '%1' doesn't exist on GitHub").arg(username), this);
      }
    }

    refresh();
    request->deleteLater();
  });

  request->sendRequest("https://github.com/" + username + ".keys");
}
