from types import SimpleNamespace

import pytest

from openpilot.tools.vehicle_telemetry.collectors.trqi_temperature import TrqiTemperatureCollector, crc8
from openpilot.tools.vehicle_telemetry.route_logs import DriveLogSet
from openpilot.tools.vehicle_telemetry.worker import analyze_telemetry
from openpilot.tools.teletyped.tests.test_drive_stats import can_message, message


def frame(temp=55.5, counter=1, status=9, bus=1, corrupt=False):
  data = round(temp * 10).to_bytes(2, "little", signed=True) + (1234).to_bytes(2, "little") + bytes((status, 3, counter))
  checksum = crc8(b"\x45\x02" + data)
  return bus, 0x245, data + bytes((checksum ^ int(corrupt),))


def temperatures(collector):
  result = collector.result()
  return [] if result is None else [row for row in result["samples"] if row["metric"] == "trqi"]


def test_dbc_scaling_flags_and_crc():
  from opendbc.can.packer import CANPacker

  assert crc8(b"123456789") == 0xF4
  packer = CANPacker("hyundai_i30_2014")
  values = {
    "Mcu_Temperature": -12.3,
    "Mcu_Temp_Raw": 1234,
    "McuTemp_Valid": 1,
    "High_Temp_Error": 1,
    "High_Temp_Disengage_Pending": 1,
    "ADC_Background_Active": 1,
    "MCU_VDDA_Warn": 1,
    "MCU_Reset_Cause": 5,
    "Main_Loop_Max_Interval": 7,
    "McuStatus_Counter": 255,
  }
  packed = packer.make_can_msg("TRQI_McuStatus", 1, values)
  payload = packed[2][:7]
  data = payload + bytes((crc8(b"\x45\x02" + payload),))
  collector = TrqiTemperatureCollector()
  collector.observe(can_message(0, (1, 0x245, data)), 0, 0)
  row = temperatures(collector)[0]
  assert row["value"] == pytest.approx(-12.3)
  assert row["status"] == 0xBF
  assert row["raw"] == 1234
  assert row["loop_max_ms"] == 7
  assert row["crc_valid"] and row["sensor_valid"]


def test_unchanged_values_counter_wrap_duplicates_and_invalid_frames():
  collector = TrqiTemperatureCollector()
  for second, packet in enumerate(
    [
      frame(counter=255),
      frame(counter=0),
      frame(counter=0),
      frame(counter=1, corrupt=True),
      frame(counter=2, status=8),
      frame(counter=3),
    ]
  ):
    collector.observe(can_message(second, packet), second, second // 3)
  rows = temperatures(collector)
  assert [row["value"] for row in rows] == [55.5, 55.5, None, None, 55.5]
  coverage = collector.result()["first_batch"]["coverage"]
  assert sum(row["repeated"] for row in coverage) == 1
  assert sum(row["crc_errors"] for row in coverage) == 1
  assert sum(row["invalid"] for row in coverage) == 2
  assert all(row["frames"] == row["valid"] + row["invalid"] + row["malformed"] + row["repeated"] for row in coverage)


def test_wrong_bus_legacy_car_and_counter_after_a_long_gap():
  collector = TrqiTemperatureCollector()
  collector.observe(can_message(0, frame(bus=129), frame(bus=0)), 0, 0)
  assert collector.result() is None
  collector.observe(can_message(1, frame()), 1, 0)
  collector.observe(can_message(257, frame()), 257, 4)
  assert len(temperatures(collector)) == 2
  assert not collector.matches("toyota", "LEXUS_LS600H")
  assert collector.matches("i30", "HYUNDAI I30 GD 2014")


def test_hvac_candidates_sentinels_and_observed_zero_report_segments():
  collector = TrqiTemperatureCollector()
  collector.observe(message("carState", 0, vEgo=10), 0, 0)
  collector.observe(can_message(0.1, frame(), (0, 0x350, bytes((0, 9, 0, 120, 255, 5, 0, 0)))), 0.1, 0)
  collector.observe(can_message(61, (0, 0x100, bytes(8))), 61, 1)
  result = collector.result()
  rows = {row["metric"]: row for row in result["samples"]}
  assert rows["outside"]["value"] == 20
  assert "outside_sensor" not in rows
  assert rows["blower_on"]["value"] == rows["ac_request"]["value"] == 1
  assert rows["speed"]["value"] == 36
  assert result["first_batch"]["coverage"][1]["frames"] == 0
  assert result["first_batch"]["coverage"][0]["first_ns"] == 100_000_000_000


def test_trqi_envelope_batches_logged_metadata_and_late_carparams(monkeypatch):
  from openpilot.tools.vehicle_telemetry import worker

  monkeypatch.setattr(worker, "TELEMETRY_BATCH_SIZE", 3)
  drive = DriveLogSet("route", ("route--0/rlog",), "a" * 64)
  messages = [
    message("initData", 0, gitBranch="recorded-branch", gitCommit="abc"),
    can_message(0.1, frame()),
    message("carParams", 0.2, carName="i30", carFingerprint="HYUNDAI I30 GD 2014"),
    message("carControl", 0.3, latActive=True, actuators=SimpleNamespace(steer=0.4)),
    can_message(1.1, frame(counter=2)),
    can_message(2.1, frame(counter=3)),
  ]
  batches = analyze_telemetry(drive, "device_123", [TrqiTemperatureCollector()], lambda *a, **kw: messages, lambda: False)
  assert len(batches) == 2
  assert batches[0]["collector"] == "trqi_temperature"
  assert batches[0]["git_branch"] == "recorded-branch"
  assert batches[0]["data"]["coverage"][0]["valid"] == 3
  assert "coverage" not in batches[1]["data"]
  assert any(row["metric"] == "steering_demand" and row["value"] == 0.4 for batch in batches for row in batch["data"]["samples"])


def test_continuous_stuck_frames_never_refresh_temperature():
  collector = TrqiTemperatureCollector()
  for second in range(8):
    collector.observe(can_message(second, frame()), second, 0)
  assert len(temperatures(collector)) == 1
  assert collector.result()["first_batch"]["coverage"][0]["repeated"] == 7
  collector.observe(can_message(20, frame()), 20, 0)
  assert len(temperatures(collector)) == 2
