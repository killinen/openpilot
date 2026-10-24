"""Short-lived, pinned HTTPS service for one prepared offroad archive.

No access to source directories, general file serving, shell commands or keys.
"""

from __future__ import annotations

import hmac
import ipaddress
import json
import os
import secrets
import socket
import ssl
import subprocess
import tempfile
import threading
from contextlib import contextmanager
from datetime import datetime, timedelta, UTC
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PRIVATE = tuple(ipaddress.ip_network(n) for n in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16"))


def lan_address() -> str | None:
  """Read the default Wi-Fi/Ethernet route without sending a network packet."""
  try:
    result = subprocess.run(
      ["ip", "-j", "route", "get", "1.1.1.1"],
      check=True,
      capture_output=True,
      text=True,
      timeout=3,
    )
    route = json.loads(result.stdout)[0]
    address = ipaddress.IPv4Address(route.get("prefsrc") or route["src"])
    if route.get("dev", "").startswith(("wlan", "wlp", "eth", "en")) and any(address in n for n in PRIVATE):
      return str(address)
  except (
    OSError,
    subprocess.SubprocessError,
    ValueError,
    KeyError,
    IndexError,
    TypeError,
  ):
    pass
  return None


def certificate(directory: Path) -> tuple[Path, Path, str]:
  # Already a device dependency for teletyped's existing Ed25519 SSH keys.
  from cryptography import x509
  from cryptography.hazmat.primitives import hashes, serialization
  from cryptography.hazmat.primitives.asymmetric import ec
  from cryptography.x509.oid import NameOID

  key = ec.generate_private_key(ec.SECP256R1())
  name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "OP-NAS one-job transfer")])
  now = datetime.now(UTC)
  cert = (
    x509.CertificateBuilder()
    .subject_name(name)
    .issuer_name(name)
    .public_key(key.public_key())
    .serial_number(x509.random_serial_number())
    .not_valid_before(now - timedelta(minutes=5))
    .not_valid_after(now + timedelta(days=1))
    .sign(key, hashes.SHA256())
  )
  private, public = directory / "key.pem", directory / "cert.pem"
  for path, data in [
    (
      private,
      key.private_bytes(
        serialization.Encoding.PEM,
        serialization.PrivateFormat.PKCS8,
        serialization.NoEncryption(),
      ),
    ),
    (public, cert.public_bytes(serialization.Encoding.PEM)),
  ]:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as out:
      out.write(data)
  return private, public, cert.fingerprint(hashes.SHA256()).hex()


@contextmanager
def serve_archive(archive: Path, address: str, permitted):
  """Bind only to the selected LAN address and expose just segment.zip."""
  token, served = secrets.token_hex(32), threading.Event()
  with tempfile.TemporaryDirectory(prefix=".lan-", dir=archive.parent) as temporary:
    private, public, pin = certificate(Path(temporary))
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(str(public), str(private))

    class Handler(BaseHTTPRequestHandler):
      def log_message(self, *_args):
        pass  # Do not log request headers, paths or one-time credentials.

      def do_GET(self):
        auth = self.headers.get("Authorization", "")
        if self.path != "/segment.zip" or not hmac.compare_digest(auth, "Bearer " + token):
          self.send_error(404)
          return
        if not permitted():
          self.send_error(409)
          return
        try:
          with archive.open("rb") as source:
            self.send_response(200)
            self.send_header("Content-Type", "application/zip")
            self.send_header("Content-Length", str(os.fstat(source.fileno()).st_size))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            for block in iter(lambda: source.read(256 * 1024), b""):
              if not permitted():
                return
              self.wfile.write(block)
            self.wfile.flush()
            served.set()
        except (OSError, ssl.SSLError):
          pass  # Receiver will discard a partial file and use Wormhole.
        finally:
          self.close_connection = True

    class Server(ThreadingHTTPServer):
      daemon_threads = True
      block_on_close = False

      def __init__(self):
        self.slots = threading.BoundedSemaphore(2)
        self.connections = set()
        self.connections_lock = threading.Lock()
        super().__init__((address, 0), Handler)

      def get_request(self):
        connection, peer = super().get_request()
        connection.settimeout(5)
        try:
          secured = context.wrap_socket(connection, server_side=True)
          secured.settimeout(30)
          return secured, peer
        except Exception:
          connection.close()
          raise

      def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
          request.close()
          return
        with self.connections_lock:
          self.connections.add(request)
        try:
          super().process_request(request, client_address)
        except Exception:
          with self.connections_lock:
            self.connections.discard(request)
          self.slots.release()
          raise

      def process_request_thread(self, request, client_address):
        try:
          super().process_request_thread(request, client_address)
        finally:
          with self.connections_lock:
            self.connections.discard(request)
          self.slots.release()

      def handle_error(self, *_args):
        pass

    server = Server()
    thread = threading.Thread(target=server.serve_forever, name="opnas-lan-source", daemon=True)
    thread.start()
    try:
      yield (
        {
          "url": f"https://{address}:{server.server_port}/segment.zip",
          "certificate_sha256": pin,
          "token": token,
        },
        served,
      )
    finally:
      server.shutdown()
      with server.connections_lock:
        for connection in server.connections:
          try:
            connection.shutdown(socket.SHUT_RDWR)
          except OSError:
            pass
          connection.close()
      server.server_close()
      thread.join(timeout=6)
