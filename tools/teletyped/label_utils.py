import re


# Matches legacy timestamp-style names, with optional segment suffix.
TIMESTAMP_LABEL_RE = re.compile(
  r"^(?P<date>\d{4}-\d{2}-\d{2})--(?P<time>\d{2}-\d{2}-\d{2})(?:--(?P<segment>\d+))?$"
)
# Matches newer counter+nonce names, with optional segment suffix and optional extension.
COUNTER_NONCE_LABEL_RE = re.compile(
  r"^(?P<counter>[0-9a-f]{8})--(?P<nonce>[0-9a-f]{10})(?:--(?P<segment>\d+))?(?:\.[A-Za-z0-9]+)?$",
  re.IGNORECASE,
)


def _split_ext(name: str) -> tuple[str, str | None]:
  if "." in name:
    base, ext = name.rsplit(".", 1)
    return base, ext
  return name, None


def drive_base_name(name: str) -> str:
  """
  Normalize a route/drive label to its base (drop segment + extension).
  """
  normalized = (name or "").strip().replace("\\", "/")
  base = normalized.split("/")[-1]
  base_no_ext, _ = _split_ext(base)

  ts_match = TIMESTAMP_LABEL_RE.match(base_no_ext)
  if ts_match:
    return f"{ts_match.group('date')}--{ts_match.group('time')}"

  cn_match = COUNTER_NONCE_LABEL_RE.match(base_no_ext)
  if cn_match:
    return f"{cn_match.group('counter')}--{cn_match.group('nonce')}"

  return base_no_ext


def is_drive_label(name: str) -> bool:
  """
  Returns True if the label matches either timestamp or counter+nonce format.
  """
  normalized = (name or "").strip().replace("\\", "/")
  base = normalized.split("/")[-1]
  base_no_ext, _ = _split_ext(base)
  return bool(
    TIMESTAMP_LABEL_RE.match(base_no_ext) or COUNTER_NONCE_LABEL_RE.match(base_no_ext)
  )


def strip_boot_prefix(name: str) -> str:
  """
  Remove any boot prefixes (boot_, boot-, boot/) while preserving the remainder.
  """
  normalized = (name or "").replace("\\", "/")
  if normalized.startswith("boot/"):
    return normalized.split("/", 1)[1]
  for prefix in ("boot_", "boot-"):
    if normalized.startswith(prefix):
      return normalized[len(prefix):]
  return normalized
