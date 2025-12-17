import os
from typing import Any

try:
  _requests: Any
  import requests as _requests
except ModuleNotFoundError:
  _requests = None
requests: Any = _requests

use_konik_server: Any
try:
  from openpilot.frogpilot.common.frogpilot_utilities import use_konik_server as _use_konik_server
  use_konik_server = _use_konik_server
except Exception:
  def _use_konik_server_fallback() -> bool:
    return False
  use_konik_server = _use_konik_server_fallback

API_HOST = os.getenv('API_HOST', 'https://api.commadotai.com')
KONIK_API_HOST = os.getenv('API_HOST', 'https://api.konik.ai')

class CommaApi:
  def __init__(self, token=None):
    if requests is None:
      raise ModuleNotFoundError("Missing optional dependency 'requests' (required for Comma API access)")
    self.session = requests.Session()
    self.session.headers['User-agent'] = 'OpenpilotTools'
    if token:
      self.session.headers['Authorization'] = 'JWT ' + token

  def request(self, method, endpoint, **kwargs):
    resp = self.session.request(method, (KONIK_API_HOST if use_konik_server() else API_HOST) + '/' + endpoint, **kwargs)
    resp_json = resp.json()
    if isinstance(resp_json, dict) and resp_json.get('error'):
      if resp.status_code in [401, 403]:
        raise UnauthorizedError('Unauthorized. Authenticate with tools/lib/auth.py')

      e = APIError(str(resp.status_code) + ":" + resp_json.get('description', str(resp_json['error'])))
      e.status_code = resp.status_code
      raise e
    return resp_json

  def get(self, endpoint, **kwargs):
    return self.request('GET', endpoint, **kwargs)

  def post(self, endpoint, **kwargs):
    return self.request('POST', endpoint, **kwargs)

class APIError(Exception):
  pass

class UnauthorizedError(Exception):
  pass
