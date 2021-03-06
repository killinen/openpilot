#!/usr/bin/env python3
import os
import json
import time
from selfdrive.swaglog import cloudlog
travis = False


class KeyInfo:
  default = None
  allowed_types = []
  has_allowed_types = False
  live = False
  has_default = False
  has_description = False
  hidden = False


class opParams:
  def __init__(self):
    """
      To add your own parameter to opParams in your fork, simply add a new dictionary entry with the name of your parameter and its default value to save to new users' op_params.json file.
      The description, allowed_types, and live keys are no longer required but recommended to help users edit their parameters with opEdit correctly.
        - The description value will be shown to users when they use opEdit to change the value of the parameter.
        - The allowed_types key is used to restrict what kinds of values can be entered with opEdit so that users can't reasonably break the fork with unintended behavior.
          Limiting the range of floats or integers is still recommended when `.get`ting the parameter.
          When a None value is allowed, use `type(None)` instead of None, as opEdit checks the type against the values in the key with `isinstance()`.
        - Finally, the live key tells both opParams and opEdit that it's a live parameter that will change. Therefore, you must place the `op_params.get()` call in the update function so that it can update.
      Here's an example of the minimum required dictionary:

      self.default_params = {'camera_offset': {'default': 0.06}}
    """

    self.default_params = {'camera_offset': {'default': 0.06, 'allowed_types': [float, int], 'description': 'Your camera offset to use in lane_planner.py', 'live': True},
                           'steer_ratio': {'default': None, 'allowed_types': [type(None), float, int], 'description': '(Can be: None, or a float) If you enter None, openpilot will use the learned sR.\n'
                                                                                                                      'If you use a float/int, openpilot will use that steer ratio instead', 'live': True},
                           'long_kpV': {'default': 2.4, 'allowed_types': [float, int], 'description': 'The proportional gain to use in the longcontroller PI loop', 'live': True},
                           'long_kiV': {'default': 0.45, 'allowed_types': [float, int], 'description': 'The integral gain to use in the longcontroller PI loop', 'live': True},
                           'lat_kpV': {'default': 0.2, 'allowed_types': [float, int], 'description': 'The proportional gain to use in the latcontroller PI loop', 'live': True},
                           'lat_kiV': {'default': 0.05, 'allowed_types': [float, int], 'description': 'The proportional gain to use in the latcontroller PI loop', 'live': True},

                           'op_edit_live_mode': {'default': False, 'allowed_types': [bool], 'description': 'This parameter controls which mode opEdit starts in. It should be hidden from the user with the hide key', 'hide': True}}

    self.params = {}
    self.params_file = "/data/op_params.json"
    self.last_read_time = time.time()
    self.read_frequency = 1.0  # max frequency to read with self.get(...) (sec)
    self.force_update = False  # replaces values with default params if True, not just add add missing key/value pairs
    self.to_delete = []  # a list of params you want to delete (unused)
    self.run_init()  # restores, reads, and updates params

  def run_init(self):  # does first time initializing of default params
    if travis:
      self.params = self._format_default_params()
      return

    self.params = self._format_default_params()  # in case any file is corrupted

    to_write = False
    if os.path.isfile(self.params_file):
      if self._read():
        to_write = not self._add_default_params()  # if new default data has been added
        if self._delete_old:  # or if old params have been deleted
          to_write = True
      else:  # don't overwrite corrupted params, just print
        cloudlog.error("ERROR: Can't read op_params.json file")
    else:
      to_write = True  # user's first time running a fork with op_params, write default params

    if to_write:
      self._write()

  def get(self, key=None, default=None, force_update=False):  # can specify a default value if key doesn't exist
    self._update_params(key, force_update)
    if key is None:
      return self._get_all()

    if key in self.params:
      key_info = self.key_info(key)
      if key_info.has_allowed_types:
        value = self.params[key]
        if type(value) in key_info.allowed_types:
          return value  # all good, returning user's value

        cloudlog.warning('op_params: User\'s value is not valid!')
        if key_info.has_default:  # invalid value type, try to use default value
          if type(key_info.default) in key_info.allowed_types:  # actually check if the default is valid
            # return default value because user's value of key is not in the allowed_types to avoid crashing openpilot
            return key_info.default

        return self._value_from_types(key_info.allowed_types)  # else use a standard value based on type (last resort to keep openpilot running if user's value is of invalid type)
      else:
        return self.params[key]  # no defined allowed types, returning user's value

    return default  # not in params

  def put(self, key, value):
    self.params.update({key: value})
    self._write()

  def delete(self, key):
    if key in self.params:
      del self.params[key]
      self._write()

  def key_info(self, key):
    key_info = KeyInfo()
    if key is None:
      return key_info
    if key in self.default_params:
      if 'allowed_types' in self.default_params[key]:
        allowed_types = self.default_params[key]['allowed_types']
        if isinstance(allowed_types, list) and len(allowed_types) > 0:
          key_info.has_allowed_types = True
          key_info.allowed_types = allowed_types

      if 'live' in self.default_params[key]:
        key_info.live = self.default_params[key]['live']

      if 'default' in self.default_params[key]:
        key_info.has_default = True
        key_info.default = self.default_params[key]['default']

      key_info.has_description = 'description' in self.default_params[key]

      if 'hide' in self.default_params[key]:
        key_info.hidden = self.default_params[key]['hide']

    return key_info

  def _add_default_params(self):
    prev_params = dict(self.params)
    for key in self.default_params:
      if self.force_update:
        self.params[key] = self.default_params[key]['default']
      elif key not in self.params:
        self.params[key] = self.default_params[key]['default']
    return prev_params == self.params

  def _format_default_params(self):
    return {key: self.default_params[key]['default'] for key in self.default_params}

  @property
  def _delete_old(self):
    deleted = False
    for i in self.to_delete:
      if i in self.params:
        del self.params[i]
        deleted = True
    return deleted

  def _get_all(self):  # returns all non-hidden params
    return {k: v for k, v in self.params.items() if not self.key_info(k).hidden}

  def _value_from_types(self, allowed_types):
    if list in allowed_types:
      return []
    elif float in allowed_types or int in allowed_types:
      return 0
    elif type(None) in allowed_types:
      return None
    elif str in allowed_types:
      return ''
    return None  # unknown type

  def _update_params(self, key, force_update):
    if force_update or self.key_info(key).live:  # if is a live param, we want to get updates while openpilot is running
      if not travis and (time.time() - self.last_read_time >= self.read_frequency or force_update):  # make sure we aren't reading file too often
        if self._read():
          self.last_read_time = time.time()

  def _read(self):
    try:
      with open(self.params_file, "r") as f:
        self.params = json.load(f)
      return True
    except Exception as e:
      cloudlog.error(e)
      self.params = self._format_default_params()
      return False

  def _write(self):
    if not travis:
      with open(self.params_file, "w") as f:
        json.dump(self.params, f, indent=2, sort_keys=True)
      os.chmod(self.params_file, 0o764)
