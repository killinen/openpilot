from __future__ import annotations

import hashlib
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

from openpilot.tools.hrr_updater.constants import PRODUCT_ID, SLOT_NAMES
from openpilot.tools.hrr_updater.errors import NetworkFailure, ReleaseVerificationError
from openpilot.tools.hrr_updater.manifest import Manifest, Version


def canonical_json(value: Any) -> bytes:
  return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def verify_release_signature(raw: bytes, signature: bytes, public_key: bytes) -> dict[str, Any]:
  if len(public_key) != 32 or not any(public_key):
    raise ReleaseVerificationError("release public key is unavailable or unprovisioned")
  if len(signature) != 64:
    raise ReleaseVerificationError("release signature must be 64 raw bytes")
  try:
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
    Ed25519PublicKey.from_public_bytes(public_key).verify(signature, raw)
  except Exception as exc:
    raise ReleaseVerificationError("release.json signature verification failed") from exc
  try:
    parsed: Any = json.loads(raw)
  except json.JSONDecodeError as exc:
    raise ReleaseVerificationError("release.json is invalid JSON") from exc
  if canonical_json(parsed) != raw:
    raise ReleaseVerificationError("release.json is not in the required canonical encoding")
  if parsed.get("schema_version") != 1 or parsed.get("product_id") != PRODUCT_ID:
    raise ReleaseVerificationError("release metadata schema/product mismatch")
  if parsed.get("product") != "HRR" or parsed.get("hardware_id") != "HRR_G474_V1":
    raise ReleaseVerificationError("release metadata hardware identity mismatch")
  if not isinstance(parsed.get("required_bootloader_version"), int):
    raise ReleaseVerificationError("release metadata lacks required bootloader version")
  if not isinstance(parsed.get("minimum_updater_version"), str):
    raise ReleaseVerificationError("release metadata lacks minimum updater version")
  slots = parsed.get("slots")
  if not isinstance(slots, dict) or set(slots) != {"A", "B"}:
    raise ReleaseVerificationError("release metadata must describe exactly application slots A and B")
  for slot in ("A", "B"):
    entry = slots[slot]
    required = ("image", "manifest", "size", "sha256", "manifest_sha256")
    if not isinstance(entry, dict) or any(name not in entry for name in required):
      raise ReleaseVerificationError(f"release metadata slot {slot} is incomplete")
  return cast(dict[str, Any], parsed)


@dataclass(frozen=True)
class ReleaseCandidate:
  tag: str
  version: Version
  prerelease: bool
  metadata: dict[str, Any]
  asset_urls: dict[str, str]

  def slot_assets(self, slot: int) -> tuple[str, str]:
    try:
      entry = self.metadata["slots"][SLOT_NAMES[slot]]
      return entry["manifest"], entry["image"]
    except (KeyError, TypeError) as exc:
      raise ReleaseVerificationError(f"release has no valid slot {SLOT_NAMES.get(slot, slot)} entry") from exc


class GitHubReleaseClient:
  def __init__(self, repository: str = "killinen/HRR", token: str | None = None, timeout: float = 30.0):
    self.repository = repository
    self.token = token or os.getenv("HRR_GITHUB_TOKEN") or os.getenv("GH_TOKEN")
    self.timeout = timeout

  def _headers(self, *, asset: bool = False) -> dict[str, str]:
    headers = {"Accept": "application/octet-stream" if asset else "application/vnd.github+json",
               "X-GitHub-Api-Version": "2022-11-28"}
    if self.token:
      headers["Authorization"] = f"Bearer {self.token}"
    return headers

  def _get(self, url: str, *, asset: bool = False) -> bytes:
    try:
      import requests
      response = requests.get(url, headers=self._headers(asset=asset), timeout=self.timeout)
      response.raise_for_status()
      return response.content
    except Exception as exc:
      raise NetworkFailure(f"GitHub request failed for {url.rsplit('/', 1)[-1]}") from exc

  def _release_records(self, tag: str | None = None) -> list[dict[str, Any]]:
    suffix = f"/releases/tags/{tag}" if tag else "/releases?per_page=20"
    raw = self._get(f"https://api.github.com/repos/{self.repository}{suffix}")
    try:
      parsed = json.loads(raw)
    except json.JSONDecodeError as exc:
      raise NetworkFailure("GitHub returned invalid release JSON") from exc
    return [parsed] if tag else list(parsed)

  def candidates(self, public_key: bytes, *, tag: str | None = None,
                 allow_prerelease: bool = False) -> list[ReleaseCandidate]:
    candidates: list[ReleaseCandidate] = []
    for record in self._release_records(tag):
      if record.get("draft") or (record.get("prerelease") and not allow_prerelease):
        continue
      assets = {asset["name"]: asset["url"] for asset in record.get("assets", [])}
      if "release.json" not in assets or "release.json.sig" not in assets:
        continue
      metadata_raw = self._get(assets["release.json"], asset=True)
      signature = self._get(assets["release.json.sig"], asset=True)
      metadata = verify_release_signature(metadata_raw, signature, public_key)
      release_tag = record["tag_name"]
      version = Version.parse(release_tag)
      if str(version) != metadata.get("firmware_version"):
        raise ReleaseVerificationError(f"tag {release_tag} disagrees with signed firmware version")
      candidates.append(ReleaseCandidate(release_tag, version, bool(record.get("prerelease")), metadata, assets))
    candidates.sort(key=lambda candidate: (candidate.version.major, candidate.version.minor, candidate.version.patch,
                                           not candidate.version.prerelease, candidate.version.prerelease), reverse=True)
    return candidates

  def download_verified_slot(self, candidate: ReleaseCandidate, slot: int, cache: ArtifactCache,
                             public_key: bytes) -> tuple[Path, Path, Manifest]:
    manifest_name, image_name = candidate.slot_assets(slot)
    if manifest_name not in candidate.asset_urls or image_name not in candidate.asset_urls:
      raise ReleaseVerificationError("signed release references missing GitHub assets")
    manifest_path = cache.store(manifest_name, self._get(candidate.asset_urls[manifest_name], asset=True))
    image_path = cache.store(image_name, self._get(candidate.asset_urls[image_name], asset=True))
    manifest = Manifest.parse(manifest_path.read_bytes())
    manifest.verify(image_path.read_bytes(), public_key)
    slot_meta = candidate.metadata["slots"][SLOT_NAMES[slot]]
    if hashlib.sha256(manifest_path.read_bytes()).hexdigest() != slot_meta["manifest_sha256"]:
      raise ReleaseVerificationError("release.json manifest hash mismatch")
    if hashlib.sha256(image_path.read_bytes()).hexdigest() != slot_meta["sha256"]:
      raise ReleaseVerificationError("release.json image hash disagrees with manifest/image")
    if image_path.stat().st_size != slot_meta["size"]:
      raise ReleaseVerificationError("release.json image size mismatch")
    return manifest_path, image_path, manifest


class ArtifactCache:
  def __init__(self, root: Path, keep_releases: int = 2):
    self.root = root
    self.keep_releases = keep_releases
    root.mkdir(parents=True, exist_ok=True)

  def store(self, name: str, data: bytes) -> Path:
    if Path(name).name != name or not name:
      raise ReleaseVerificationError("unsafe release asset name")
    destination = self.root / name
    fd, temporary_name = tempfile.mkstemp(prefix=f".{name}.", suffix=".part", dir=self.root)
    try:
      with os.fdopen(fd, "wb") as output:
        output.write(data)
        output.flush()
        os.fsync(output.fileno())
      os.replace(temporary_name, destination)
      directory_fd = os.open(self.root, os.O_RDONLY)
      try:
        os.fsync(directory_fd)
      finally:
        os.close(directory_fd)
    finally:
      try:
        os.unlink(temporary_name)
      except FileNotFoundError:
        pass
    self.prune()
    return destination

  def prune(self) -> None:
    files = sorted((path for path in self.root.iterdir() if path.is_file() and not path.name.endswith(".part")),
                   key=lambda path: path.stat().st_mtime_ns, reverse=True)
    # A complete A/B candidate has two manifests and two images. Keep both
    # slots so an ignition-on preflight can select the inactive slot without
    # needing network access.
    for old in files[self.keep_releases * 4:]:
      try:
        old.unlink()
      except FileNotFoundError:
        pass
