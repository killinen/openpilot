from __future__ import annotations

import binascii
import hashlib
import struct
from dataclasses import dataclass
from pathlib import Path

from openpilot.tools.hrr_updater.constants import (BOOT_BASE, BOOT_END, MANIFEST_FORMAT_VERSION, MANIFEST_MAGIC,
                                         PRODUCT_ID, SIGNATURE_ED25519, SLOT_BASES, SLOT_SIZE)
from openpilot.tools.hrr_updater.errors import ImageVerificationError

PREFIX = struct.Struct("<IHHIBBHIIIHHHHII20s32s")
FULL = struct.Struct("<IHHIBBHIIIHHHHII20s32s64sI")


@dataclass(frozen=True, order=True)
class Version:
  major: int
  minor: int
  patch: int
  prerelease: str = ""

  @classmethod
  def parse(cls, value: str) -> Version:
    raw = value.removeprefix("v")
    core, separator, prerelease = raw.partition("-")
    parts = core.split(".")
    if len(parts) != 3 or any(not part.isdigit() for part in parts):
      raise ValueError(f"invalid semantic version: {value}")
    major, minor, patch = (int(part) for part in parts)
    if any(number > 0xFFFF for number in (major, minor, patch)):
      raise ValueError("version components must fit uint16")
    return cls(major, minor, patch, prerelease if separator else "")

  def __str__(self) -> str:
    suffix = f"-{self.prerelease}" if self.prerelease else ""
    return f"{self.major}.{self.minor}.{self.patch}{suffix}"


@dataclass(frozen=True)
class Manifest:
  raw: bytes
  product_id: int
  slot: int
  load_address: int
  entry_address: int
  image_size: int
  version: Version
  flags: int
  security_version: int
  required_bootloader_version: int
  key_id: int
  build_id: bytes
  image_sha256: bytes
  signature: bytes

  @property
  def build_identity(self) -> int:
    return int.from_bytes(self.build_id[:4], "little")

  @property
  def identity(self) -> str:
    return self.image_sha256.hex()

  @classmethod
  def parse(cls, raw: bytes) -> Manifest:
    if len(raw) != FULL.size:
      raise ImageVerificationError(f"manifest must be {FULL.size} bytes, got {len(raw)}")
    values = FULL.unpack(raw)
    (magic, format_version, header_size, product_id, slot, algorithm, key_id,
     load_address, entry_address, image_size, major, minor, patch, flags,
     security_version, required_bootloader_version, build_id, image_hash,
     signature, crc32) = values
    if magic != MANIFEST_MAGIC or format_version != MANIFEST_FORMAT_VERSION or header_size != FULL.size:
      raise ImageVerificationError("unsupported HRR manifest header")
    if algorithm != SIGNATURE_ED25519:
      raise ImageVerificationError("manifest is not signed with Ed25519")
    if (binascii.crc32(raw[:-4]) & 0xFFFFFFFF) != crc32:
      raise ImageVerificationError("manifest CRC32 mismatch")
    if product_id != PRODUCT_ID:
      raise ImageVerificationError(f"wrong product ID 0x{product_id:08x}")
    if slot not in SLOT_BASES:
      raise ImageVerificationError("manifest does not target an application slot")
    if BOOT_BASE <= load_address < BOOT_END:
      raise ImageVerificationError("bootloader artifacts are service-only")
    if load_address != SLOT_BASES[slot] or not 8 <= image_size <= SLOT_SIZE:
      raise ImageVerificationError("manifest slot range is invalid")
    image_end = load_address + image_size
    if image_end > load_address + SLOT_SIZE:
      raise ImageVerificationError("manifest image exceeds slot")
    if not (entry_address & 1) or not load_address <= (entry_address & ~1) < image_end:
      raise ImageVerificationError("manifest entry address is invalid")
    return cls(raw, product_id, slot, load_address, entry_address, image_size,
               Version(major, minor, patch), flags, security_version,
               required_bootloader_version, key_id, build_id, image_hash, signature)

  def verify(self, image: bytes, public_key: bytes) -> None:
    if len(public_key) != 32 or not any(public_key):
      raise ImageVerificationError("firmware public key is unavailable or unprovisioned")
    if len(image) != self.image_size:
      raise ImageVerificationError(f"image size mismatch: expected {self.image_size}, got {len(image)}")
    if hashlib.sha256(image).digest() != self.image_sha256:
      raise ImageVerificationError("image SHA-256 mismatch")
    stack_pointer, reset_handler = struct.unpack_from("<II", image)
    if not (0x20000000 <= stack_pointer <= 0x20020000 and stack_pointer % 8 == 0):
      raise ImageVerificationError("image initial stack pointer is invalid")
    if reset_handler != self.entry_address:
      raise ImageVerificationError("image reset vector does not match manifest")
    try:
      from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
      Ed25519PublicKey.from_public_bytes(public_key).verify(self.signature, self.raw[:PREFIX.size])
    except ImageVerificationError:
      raise
    except Exception as exc:
      raise ImageVerificationError("manifest signature verification failed") from exc


def load_public_key(path: Path) -> bytes:
  try:
    text = path.read_text(encoding="ascii")
    key = bytes.fromhex("".join(text.split()))
  except (OSError, ValueError) as exc:
    raise ImageVerificationError(f"cannot load public key from {path}") from exc
  if len(key) != 32:
    raise ImageVerificationError("public key must contain exactly 32 bytes")
  if not any(key):
    raise ImageVerificationError("firmware public key is unprovisioned")
  return key


def load_verified_artifacts(manifest_path: Path, image_path: Path, public_key_path: Path) -> tuple[Manifest, bytes]:
  manifest = Manifest.parse(manifest_path.read_bytes())
  image = image_path.read_bytes()
  manifest.verify(image, load_public_key(public_key_path))
  return manifest, image
