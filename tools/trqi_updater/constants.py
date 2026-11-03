from dataclasses import dataclass
from pathlib import Path

PRODUCT_ID = 0x54525101
PROTOCOL_VERSION = 1
MANIFEST_MAGIC = 0x4D515254
MANIFEST_FORMAT_VERSION = 1
SIGNATURE_ED25519 = 1

SLOT_NONE = 0
SLOT_A = 1
SLOT_B = 2
SLOT_NAMES = {SLOT_A: "A", SLOT_B: "B"}
SLOT_BASES = {SLOT_A: 0x08010000, SLOT_B: 0x08040000}
SLOT_SIZE = 0x30000
BOOT_BASE = 0x08000000
BOOT_END = 0x08010000

ENTER_BOOT_ID = 0x60A
APP_INFO0_ID = 0x639
APP_INFO1_ID = 0x63A
APP_INFO2_ID = 0x63B
APP_INFO3_ID = 0x63C
BOOT_REQUEST_ID = 0x6A0
BOOT_RESPONSE_ID = 0x6A1
DEFAULT_PANDA_BUS = 1
DEFAULT_FRAME_PACING = 0.005

SVC_GET_INFO = 0x22
SVC_STATUS = 0x31
SVC_START = 0x34
SVC_DATA = 0x36
SVC_FINISH = 0x37
SVC_RESET = 0x11
SVC_ABORT = 0x7F

FLAG_TRIAL = 1 << 0
FLAG_CONFIRMED = 1 << 1
FLAG_UPDATE_SUPPORT = 1 << 2

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_PUBLIC_KEY = PACKAGE_DIR / "keys" / "trqi_release_public_key.hex"
DEFAULT_TEST_PUBLIC_KEY = PACKAGE_DIR / "keys" / "TEST_ONLY_ed25519_public.hex"

# Profiles are explicit at every transport and manifest boundary; this avoids
# sharing response IDs, slot geometry or signed product identities on CAN.
@dataclass(frozen=True)
class TargetProfile:
  name: str
  product_id: int
  slot_bases: dict[int, int]
  slot_size: int
  boot_base: int
  boot_end: int
  ram_end: int
  enter_id: int
  info_ids: tuple[int, int, int, int]
  request_id: int
  response_id: int
  chunk_size: int

TRQI_TARGET = TargetProfile("trqi", PRODUCT_ID, SLOT_BASES, SLOT_SIZE, BOOT_BASE, BOOT_END,
                            0x20020000, ENTER_BOOT_ID,
                            (APP_INFO0_ID, APP_INFO1_ID, APP_INFO2_ID, APP_INFO3_ID),
                            BOOT_REQUEST_ID, BOOT_RESPONSE_ID, 128)
HRR_TARGET = TargetProfile("hrr", 0x48525201, {SLOT_A: 0x8000, SLOT_B: 0x20000}, 0x18000,
                           0, 0x8000, 0x20007f00, 0x60b, (0x63d, 0x63e, 0x63f, 0x640),
                           0x6a2, 0x6a3, 256)
TARGETS = {profile.name: profile for profile in (TRQI_TARGET, HRR_TARGET)}
