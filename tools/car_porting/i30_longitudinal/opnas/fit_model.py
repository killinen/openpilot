
import os
from pathlib import Path

"""Fit the engine table + vehicle law on training drives; validate on withheld whole drives.

Usage: python3 fit_model.py   (reads analysis/rows_*.npz, writes analysis/model.json, analysis/validation.json)
"""
import hashlib
import json
import sys
import numpy as np
import load
import table

REPO = os.environ.get('I30_REPO', str(Path(__file__).resolve().parents[4]))

# Codex's prototype map, for comparison only (host python here lacks zip(strict=)).
_src = open(f'{REPO}/tools/car_porting/i30_longitudinal/lookup.py').read().replace(', strict=True', '')
_ns: dict = {}
exec(compile(_src, 'lookup.py', 'exec'), _ns)
PedalMap = _ns['PedalMap']

ROOT = load.ROOT
C3X = 'ff528bde139e99de'
CODEX_TRAIN = {f'0000061{c}' for c in 'bcdef'} | {f'0000062{c}' for c in '01234'}
CODEX_VALID = {'00000614', '00000615'}
SUPPORT_MIN = 30  # engine-table weight (10 Hz samples) per cell for "supported"
ENGINE_INPUT = 'dynamic' if '--settled' not in sys.argv else 'settled'


def holdout(route):
  """Deterministic whole-drive split; Codex's two validation drives are always withheld."""
  if route.split('--')[0] in CODEX_VALID:
    return True
  return int(hashlib.sha1(route.encode()).hexdigest(), 16) % 4 == 0


def load_rows():
  rows = []
  # i30 runtime device only (interceptor-fitted C3X). The old devices ran other software/hardware and are not used.
  for f in [ROOT / 'analysis' / f'rows_{C3X}.npz']:
    d = dict(np.load(f, allow_pickle=True))
    d['device'] = np.full(len(d['t']), f.stem[5:])
    rows.append(d)
  return {k: np.concatenate([r[k] for r in rows]) for k in rows[0]}


def fit(d, m):
  # 1. Engine table from settled pedal, clutch engaged, rpm valid. C3X (interceptor fitted) only:
  #    older no-interceptor logs give 8-12 % less torque at the same PV_AV_CAN (see report).
  if ENGINE_INPUT == 'settled':
    e = m & (d['p_set'] > 0) & (d['N'] > 700) & (d['device'] == C3X)
    tab, sup = table.fit_engine(d['pedal'][e], d['N'][e], d['T'][e])
  else:
    # All samples through the fitted pedal->torque dynamics (needed for high pedal, which is rarely held).
    e = m & np.isfinite(d['p_eff']) & (d['N'] > 700) & (d['device'] == C3X)
    tab, sup = table.fit_engine(d['p_eff'][e], d['N'][e], d['T'][e])
  # 2. Vehicle law on ESC specific force (grade-free): f = k_g * T * ratio - c0 - c2 v^2.
  vm = m & (d['esp_ok'] > 0) & (d['v'] > 3) & np.isfinite(d['T_lag']) & np.isfinite(d['f_spec'])
  # C3X only since the full C3X archive is fetched: pooling the old devices biased held-out C3X acceleration by -0.012 m/s^2.
  vm &= d['device'] == C3X
  gears = [g for g in range(1, 7) if np.sum(vm & (d['gear'] == g)) > 500]
  X = np.column_stack([(d['gear'][vm] == g) * d['T_lag'][vm] * d['ratio'][vm] for g in gears] + [-np.ones(vm.sum()), -(d['v'][vm] ** 2)])
  y = d['f_spec'][vm]
  w = np.ones(len(y))
  for _ in range(4):
    c = np.linalg.lstsq(X * np.sqrt(w)[:, None], y * np.sqrt(w), rcond=None)[0]
    w = np.minimum(1, 0.1 / np.maximum(np.abs(y - X @ c), 1e-6))
  k = {g: float(c[i]) for i, g in enumerate(gears)}
  c0, c2 = float(c[-2]), float(c[-1])
  # 3. Controller pitch (carControl.orientationNED) -> ESC grade mapping. C3X only: the old devices' pose
  # runs other software/mounts and maps to grade with gains of 0.37-0.70, which would halve a pooled fit.
  gm = m & (d['device'] == C3X) & (d['pitch_valid'] == 1) & np.isfinite(d['grade_pose']) & np.isfinite(d['grade_esp']) & (d['esp_ok'] > 0) & (d['v'] > 3)
  G, off = np.polyfit(d['grade_pose'][gm], d['grade_esp'][gm], 1)
  return dict(
    pedal_bp=table.PEDAL_BP.tolist(),
    rpm_bp=table.RPM_BP.tolist(),
    torque=tab.tolist(),
    support=sup.tolist(),
    k=k,
    c0=c0,
    c2=c2,
    grade_gain=float(G),
    grade_offset=float(off),
    dynamics=json.load(open(ROOT / 'analysis' / 'dynamics.json'))['best'][1:],
    n_engine=int(e.sum()),
    n_vehicle=int(vm.sum()),
    n_grade=int(gm.sum()),
  )


def predict(model, d, idx, pedal_col='p_eff', grade='pose'):
  tab = np.array(model['torque'])
  T = table.engine_torque(tab, d[pedal_col][idx], d['N'][idx])
  k = np.array([model['k'].get(int(g), model['k'].get(str(int(g)), np.nan)) for g in d['gear'][idx]])
  if grade == 'pose':
    gr = model['grade_gain'] * d['grade_pose'][idx] + model['grade_offset']
  elif grade == 'causal':
    gr = d['grade_causal'][idx]
  else:
    gr = d['grade_esp'][idx]
  return k * T * d['ratio'][idx] - model['c0'] - model['c2'] * d['v'][idx] ** 2 - gr


def supported(model, d, idx):
  sup = np.array(model['support'])
  pi = np.clip(np.searchsorted(table.PEDAL_BP, d['pedal'][idx], side='right') - 1, 0, len(table.PEDAL_BP) - 2)
  ni = np.clip(np.searchsorted(table.RPM_BP, d['N'][idx], side='right') - 1, 0, len(table.RPM_BP) - 2)
  s = np.minimum.reduce([sup[ni, pi], sup[ni + 1, pi], sup[ni, pi + 1], sup[ni + 1, pi + 1]])
  inside = (d['N'][idx] >= table.RPM_BP[0]) & (d['N'][idx] <= table.RPM_BP[-1])
  return (s >= SUPPORT_MIN) & inside


def metrics(err, n_total):
  err = err[np.isfinite(err)]
  if len(err) == 0:
    return dict(n=0, coverage=0.0)
  return dict(
    n=int(len(err)),
    coverage=round(len(err) / max(n_total, 1), 3),
    mae=round(float(np.mean(np.abs(err))), 4),
    bias=round(float(np.mean(err)), 4),
    p95=round(float(np.quantile(np.abs(err), 0.95)), 4),
  )


def codex_predict(d, idx):
  cal = json.load(open(f'{REPO}/tools/car_porting/i30_longitudinal/results/2026-11-12/pedal-map.json'))
  pm = PedalMap(cal)
  out = np.full(len(idx), np.nan)
  pitch = np.arcsin(np.clip(d['grade_pose'][idx] / 9.81, -1, 1))
  # Codex's map uses commanded pedal (0..1) delayed 0.4 s; approximate with PV/100 lagged by the sample 0.4 s earlier when available.
  for j, i in enumerate(idx):
    g = int(d['gear'][i])
    if g in (4, 5, 6) and np.isfinite(pitch[j]):
      r = pm.acceleration(g, float(d['v'][i]), float(pitch[j]), float(d['p_codex'][i]))
      out[j] = np.nan if r is None else r
  return out


def evaluate(model, d, m):
  res = {}
  base = m & np.isfinite(d['a']) & (d['v'] > 3) & (d['gear'] > 0) & (d['pitch_valid'] == 1) & (d['device'] == C3X)
  idx = np.flatnonzero(base)
  a = d['a'][idx]
  sup = supported(model, d, idx)
  pred_dyn = predict(model, d, idx, 'p_eff')
  pred_static = predict(model, d, idx, 'pedal')
  pred_esp = predict(model, d, idx, 'p_eff', grade='esp')
  pred_causal = predict(model, d, idx, 'p_eff', grade='causal')
  pc = codex_predict(d, idx)
  not_codex_train = np.array([str(r).split('--')[0] not in CODEX_TRAIN for r in d['route'][idx]])
  groups = {
    'all': np.ones(len(idx), bool),
    'not_codex_training': not_codex_train,
    'coast_1s': d['coast_s'][idx] >= 1.0,
    'coast_1s_not_codex_training': (d['coast_s'][idx] >= 1.0) & not_codex_train,
    'op_pedal': d['op_pedal'][idx] > 0,
    'driver': d['op_pedal'][idx] == 0,
  }
  for g in range(1, 7):
    groups[f'gear{g}'] = d['gear'][idx] == g
  for lo, hi in ((0, 0.5), (0.5, 10), (10, 25), (25, 40), (40, 101)):
    groups[f'pedal_{lo}-{hi}'] = (d['pedal'][idx] >= lo) & (d['pedal'][idx] < hi)
  for name, gm in groups.items():
    res[name] = dict(
      windows=int(gm.sum()),
      physical_dynamic=metrics(np.where(sup, pred_dyn - a, np.nan)[gm], gm.sum()),
      physical_static=metrics(np.where(sup, pred_static - a, np.nan)[gm], gm.sum()),
      physical_dynamic_esp_grade=metrics(np.where(sup, pred_esp - a, np.nan)[gm], gm.sum()),
      physical_dynamic_causal_esp_grade=metrics(np.where(sup, pred_causal - a, np.nan)[gm], gm.sum()),
      codex_map=metrics((pc - a)[gm], gm.sum()),
    )
  # Per withheld drive, for spread.
  per = {}
  for r in sorted(set(d['route'][idx])):
    rm = d['route'][idx] == r
    per[str(r)] = dict(
      windows=int(rm.sum()), physical_dynamic=metrics(np.where(sup, pred_dyn - a, np.nan)[rm], rm.sum()), codex_map=metrics((pc - a)[rm], rm.sum())
    )
  res['per_route'] = per
  return res


def main():
  d = load_rows()
  for k in ('stable', 'no_brake', 'esp_ok', 'p_set', 'op_pedal', 'driver_gas', 'long_active'):
    d[k] = np.nan_to_num(d[k]).astype(bool)
  hold = np.array([holdout(r) for r in d['route']])
  # ECU pedal is modelled, so driver pedal input is valid data (no ~driver_gas exclusion needed).
  ok = d['stable'] & d['no_brake'] & np.isfinite(d['pedal']) & np.isfinite(d['T'])
  all_drives = '--all' in sys.argv  # plant model for simulation: fit on every drive
  if all_drives:
    hold = np.zeros(len(hold), bool)
  model = fit(d, ok & ~hold)
  # Interceptor command -> ECU pedal (both logged); OP commanding, driver not pressing, settled.
  cm = d['op_pedal'] & ~d['driver_gas'] & d['p_set'] & np.isfinite(d['cmd_pedal']) & (d['cmd_pedal'] > 0.02) & np.isfinite(d['pedal']) & ~hold
  xc, yc, w = d['cmd_pedal'][cm] * 100.0, d['pedal'][cm], np.ones(cm.sum())
  for _ in range(4):  # Huber-style reweighting: a few routes carry multi-% offsets
    sc, of = np.polyfit(xc, yc, 1, w=np.sqrt(w))
    w = np.minimum(1, 1.0 / np.maximum(np.abs(yc - (sc * xc + of)), 1e-6))
  model['pedal_command'] = dict(
    scale=round(float(sc), 4), offset=round(float(of), 3), n=int(cm.sum()), rms=round(float(np.std(d['pedal'][cm] - (sc * d['cmd_pedal'][cm] * 100 + of))), 3)
  )
  # Ratio clusters from data (no labels assumed): median and robust spread per cluster window.
  gr = {}
  for g in range(1, 7):
    gm = d['stable'] & (d['gear'] == g) & (d['v'] > 2)
    if gm.sum() > 200:
      r = d['ratio'][gm]
      med = float(np.median(r))
      mad = float(np.median(np.abs(r - med)))
      gr[str(g)] = dict(median=round(med, 2), mad=round(mad, 2), tol=round(max(4 * 1.4826 * mad, 0.03 * med), 2), n=int(gm.sum()))
  (ROOT / 'analysis' / 'gear_ratios.json').write_text(json.dumps(gr, indent=1))
  model['gear_ratios'] = gr
  model['v_min'], model['v_max'] = 5.0, 40.0
  model['train_routes'] = sorted(set(d['route'][ok & ~hold]))
  model['holdout_routes'] = sorted(set(d['route'][ok & hold]))
  if all_drives:
    (ROOT / 'analysis' / 'model_all.json').write_text(json.dumps(model, indent=1))
    return
  val = dict(holdout=evaluate(model, d, ok & hold), training=evaluate(model, d, ok & ~hold))
  (ROOT / 'analysis' / 'model.json').write_text(json.dumps(model, indent=1))
  (ROOT / 'analysis' / 'validation.json').write_text(json.dumps(val, indent=1))
  print(json.dumps({k: model[k] for k in ('k', 'c0', 'c2', 'grade_gain', 'grade_offset', 'n_engine', 'n_vehicle')}, indent=1))
  for name in ('all', 'coast_1s', 'op_pedal', 'gear3', 'gear4', 'gear5', 'gear6', 'pedal_0-0.5', 'pedal_10-25', 'pedal_25-40', 'pedal_40-101'):
    print(name, json.dumps(val['holdout'][name]))


if __name__ == '__main__':
  main()
