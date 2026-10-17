from types import SimpleNamespace

from openpilot.tools.teletyped import extensions


def test_disabled_and_missing_extensions_are_isolated(monkeypatch):
  monkeypatch.setenv("TELETYPED_EXTENSIONS", "")
  assert extensions.optional_workers(dict) == []
  errors = []
  monkeypatch.setattr(extensions, "capture_exception", errors.append)
  monkeypatch.setattr(extensions, "log", lambda *args: None)
  monkeypatch.setenv("TELETYPED_EXTENSIONS", "missing.module:step")
  assert extensions.optional_workers(dict) == []
  assert len(errors) == 1


def test_extension_callbacks_keep_their_own_binding_and_auth(monkeypatch):
  calls = []
  module = SimpleNamespace(first=lambda device, headers: calls.append((1, device, headers)), second=lambda device, headers: calls.append((2, device, headers)))
  monkeypatch.setattr(extensions, "import_module", lambda name: module)
  monkeypatch.setenv("TELETYPED_EXTENSIONS", "demo:first,demo:second")
  headers = {}
  workers = extensions.optional_workers(lambda: headers)
  workers[0][0]("device")
  assert calls == []
  headers["X-Device-JWT"] = "test"
  for callback, _, _ in workers:
    callback("device")
  assert [call[0] for call in calls] == [1, 2]
