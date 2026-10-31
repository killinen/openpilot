"""Hardware-independent tests; use --confcutdir=tools/teletyped/tests to skip hardware fixtures."""

import http.client
import importlib.util
import ssl
from pathlib import Path
from urllib.parse import urlsplit


def module():
  spec = importlib.util.spec_from_file_location("archive_lan", Path(__file__).parents[1] / "archive_lan.py")
  result = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(result)
  return result


def test_lan_service_stops_when_device_goes_onroad(tmp_path):
  source = tmp_path / "segment.zip"
  source.write_bytes(b"sensitive file")
  allowed = [True]
  with module().serve_archive(source, "127.0.0.1", lambda: allowed[0]) as (offer, served):
    allowed[0] = False
    connection = http.client.HTTPSConnection("127.0.0.1", urlsplit(offer["url"]).port, timeout=2, context=ssl._create_unverified_context())
    connection.request("GET", "/segment.zip", headers={"Authorization": "Bearer " + offer["token"]})
    response = connection.getresponse()
    assert response.status == 409
    assert b"sensitive file" not in response.read()
    connection.close()
    assert not served.is_set()
  assert source.read_bytes() == b"sensitive file"
  assert not list(tmp_path.glob(".lan-*"))
