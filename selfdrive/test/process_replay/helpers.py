import os

class OpenpilotPrefix:
  def __enter__(self):
    self.prev_prefix = os.environ.get("OPENPILOT_PREFIX", None)
    os.environ["OPENPILOT_PREFIX"] = "/tmp"
    # manually create fake params dir structure
    params_path = "/tmp/params"
    if not os.path.exists(params_path):
      os.makedirs(params_path, exist_ok=True)
    # No call to Params().get_param_path()
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    if self.prev_prefix is not None:
      os.environ["OPENPILOT_PREFIX"] = self.prev_prefix
    else:
      del os.environ["OPENPILOT_PREFIX"]

