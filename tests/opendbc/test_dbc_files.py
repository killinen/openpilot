from pathlib import Path

import pytest

from opendbc import DBC_PATH


DBC_FILES = sorted(Path(DBC_PATH).glob("*.dbc"))
if not DBC_FILES:
  pytest.skip("no DBC files found under opendbc", allow_module_level=True)

cantools = pytest.importorskip("cantools")


@pytest.mark.parametrize("dbc_path", DBC_FILES, ids=lambda path: path.stem)
def test_dbc_parses_cleanly(dbc_path: Path) -> None:
  db = cantools.database.load_file(dbc_path)
  assert db.messages, f"{dbc_path.name} did not yield any messages"
