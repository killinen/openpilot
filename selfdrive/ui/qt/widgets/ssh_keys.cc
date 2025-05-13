#include "selfdrive/ui/qt/widgets/ssh_keys.h"

#include "common/params.h"
#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/widgets/input.h"

SshControl::SshControl() : ButtonControl(tr("SSH Keys"), "", tr("Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username other than your own. A comma employee will NEVER ask you to add their GitHub username.")) {
  username_label.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  username_label.setStyleSheet("color: #aaaaaa");
  hlayout->insertWidget(1, &username_label);

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
      params.remove("GithubSshKeys");
      refresh();
    }
  });

  refresh();
}

void SshControl::refresh() {
  QString param = QString::fromStdString(params.get("GithubSshKeys"));
  if (param.length()) {
    username_label.setText(QString::fromStdString(params.get("GithubUsername")));
    setText(tr("REMOVE"));
  } else {
    username_label.setText("");
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
        params.put("GithubSshKeys", resp.toStdString());
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

GoranConnectPasswordControl::GoranConnectPasswordControl() : ButtonControl(tr("Remote Control Password"), "", tr("Set a password for GoranConnect remote control access.")) {
  password_label.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  password_label.setStyleSheet("color: #aaaaaa");
  hlayout->insertWidget(1, &password_label);

  QObject::connect(this, &ButtonControl::clicked, [=]() {
    if (text() == tr("SET")) {
      QString password = InputDialog::getText(tr("Enter remote control password"), this, "", true);
      if (!password.isEmpty()) {
        params.put("GoranConnectPassword", password.toStdString());
        refresh();
      }
    } else {
      params.remove("GoranConnectPassword");
      refresh();
    }
  });

  refresh();
}

void GoranConnectPasswordControl::refresh() {
  QString pw = QString::fromStdString(params.get("GoranConnectPassword"));
  if (!pw.isEmpty()) {
    password_label.setText("********");
    setText(tr("REMOVE"));
  } else {
    password_label.setText("");
    setText(tr("SET"));
  }
  setEnabled(true);
}
