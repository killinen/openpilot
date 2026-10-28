#!/usr/bin/env python3
"""Replay LS600h shadow angles from saved rlogs; never sends CAN or writes params.

Run in the repository Docker environment. Input files must be in route order.
Publication-time replay is observational, not a closed-loop vehicle simulation.
"""
import argparse
from collections import Counter
import csv
import json

from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.toyota.hrr_angle_shadow import HrrAngleShadow
from openpilot.selfdrive.car.toyota.values import CAR, DBC
from openpilot.tools.lib.logreader import LogReader


def export_recorded(paths, output):
  """Export the actual live observer's decisions, including its OEM rate."""
  modes = Counter()
  samples = 0
  origin = None
  last_t = None
  versions = set()
  would_disable_s = 0.0
  with open(output, 'w', newline='') as f:
    writer = None
    for msg in LogReader(paths):
      if msg.which() != 'frogpilotCarState':
        continue
      diagnostic = msg.frogpilotCarState.ls600hAngleShadow
      if not diagnostic.version:
        continue
      t = msg.logMonoTime * 1e-9
      if origin is None:
        origin = t
      row = {'seconds': t - origin, **diagnostic.to_dict()}
      if writer is None:
        writer = csv.DictWriter(f, fieldnames=list(row))
        writer.writeheader()
      writer.writerow(row)
      dt = min(max(t - last_t, 0), .03) if last_t is not None else 0
      if diagnostic.latActive:
        modes[str(diagnostic.mode)] += dt
        would_disable_s += dt * diagnostic.wouldDisableLateral
      versions.add(diagnostic.version)
      samples += 1
      last_t = t
  return {'recorded_shadow_samples': samples, 'shadow_versions': sorted(versions),
          'active_modes_s': dict(modes), 'would_disable_s': would_disable_s,
          'note': 'Actual recorded ghost diagnostics; live steering remained unchanged.'}


def replay(paths, output):
  dbc = DBC[CAR.LEXUS_LS600h]['pt']
  oem = CANParser(dbc, [('STEER_ANGLE_SENSOR', 0)], 0)
  hrr = CANParser(dbc, [('HRR_TrueAngleStatus', 0)], 2)
  ghost = HrrAngleShadow()
  last_oem_ts = 0
  last_hrr_ts = 0
  hrr_samples = 0
  lat_active = False
  origin = None
  last_t = None
  modes = Counter()
  summary = Counter()
  with open(output, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['seconds', 'lat_active', 'live_angle_deg', 'shadow_angle_deg', 'shadow_rate_deg_s',
                     'usable', 'would_disable', 'mode', 'reason', 'oem_deg', 'hrr_deg', 'innovation_deg',
                     'reference_offset_deg', 'oem_age_s', 'hrr_age_s'])
    for msg in LogReader(paths):
      kind = msg.which()
      t = msg.logMonoTime * 1e-9
      if kind == 'can':
        raw = msg.as_builder().to_bytes()
        oem.update_strings([raw])
        hrr.update_strings([raw])
        hrr_samples += len(hrr.vl_all['HRR_TrueAngleStatus']['True_Angle_Counter'])
      elif kind == 'carControl':
        lat_active = msg.carControl.latActive
      elif kind == 'carState':
        if origin is None:
          origin = t
        if last_t is not None and t - last_t > .1:
          # Missing log segments are not a simulated sensor outage.
          ghost = HrrAngleShadow()
        os = oem.vl['STEER_ANGLE_SENSOR']
        hs = hrr.vl['HRR_TrueAngleStatus']
        ots = oem.ts_nanos['STEER_ANGLE_SENSOR']['STEER_ANGLE']
        hts = hrr.ts_nanos['HRR_TrueAngleStatus']['True_Angle_Counter']
        result = ghost.update(t, os['STEER_ANGLE'], 0.0, ots > last_oem_ts,
                              hs['True_Steering_Angle'], hs['True_Steering_Rate'], hts > last_hrr_ts,
                              int(hs['True_Angle_Counter']),
                              bool(hs['True_Angle_Valid'] and hs['True_Angle_Initialized'] and
                                   hs['True_Angle_Resolver_Valid'] and hs['True_Angle_Calibrated']),
                              bool(hs['True_Angle_Wrap_Ambiguous']), lat_active, hrr_samples)
        # Offline OEM rate is deliberately unspecified; angle decisions do not
        # depend on it. Live shadow logs include the existing OEM rate filter.
        dt = min(max(t - last_t, 0), .03) if last_t is not None else 0
        modes[result.mode] += dt if lat_active else 0
        if lat_active:
          summary['active_s'] += dt
          summary['would_disable_s'] += dt * result.would_disable
          summary['usable_angle_difference_gt3_s'] += dt * (result.usable and abs(result.angle - msg.carState.steeringAngleDeg) > 3)
        writer.writerow([t - origin, lat_active, msg.carState.steeringAngleDeg, result.angle, result.rate,
                         result.usable, result.would_disable, result.mode, result.reason, os['STEER_ANGLE'],
                         hs['True_Steering_Angle'], result.innovation, result.offset, result.oem_age, result.hrr_age])
        last_oem_ts, last_hrr_ts, last_t = ots, hts, t
        hrr_samples = 0
  return {'shadow_version': HrrAngleShadow.VERSION, 'active_modes_s': dict(modes), **summary,
          'note': 'Measurement-only replay; rate in OEM bridge is unspecified; no simulated vehicle response.'}


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('logs', nargs='+')
  parser.add_argument('--output', required=True, help='CSV output path')
  parser.add_argument('--recorded-shadow', action='store_true', help='Export logged ghost decisions instead of recomputing them')
  args = parser.parse_args()
  print(json.dumps((export_recorded if args.recorded_shadow else replay)(args.logs, args.output), indent=2))
