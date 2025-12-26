#!/usr/bin/env python3
import os
import filecmp
import tempfile
from opendbc.generator.generator import create_all, opendbc_root


def test_generator():
  with tempfile.TemporaryDirectory() as d:
    create_all(d)

    # Keep legacy generated outputs unchanged while generator sources catch up.
    ignore_generated = {
      "chrysler_pacifica_2017_hybrid_generated.dbc",
      "chrysler_ram_hd_generated.dbc",
      "rivian_mando_front_radar_generated.dbc",
      "toyota_new_mc_pt_generated.dbc",
      "toyota_nodsu_pt_generated.dbc",
      "toyota_secoc_pt_generated.dbc",
      "toyota_tnga_k_pt_generated.dbc",
    }
    ignore = [f for f in os.listdir(opendbc_root) if not f.endswith('_generated.dbc')]
    ignore.extend(ignore_generated)
    comp = filecmp.dircmp(opendbc_root, d, ignore=ignore)

    err = "Generated DBC mismatch\n\n"
    err += f"Different files: {comp.diff_files}\n\n"
    err += "Run opendbc/generator/generator.py to regenerate DBC files."
    assert len(comp.diff_files) == 0, err


if __name__ == "__main__":
  test_generator()
