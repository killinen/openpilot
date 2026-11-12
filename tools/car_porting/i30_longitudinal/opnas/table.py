"""Compact runtime-style model: engine torque table + vehicle power balance.

Kept dependency-free (numpy only for fitting) and mirrored by the scalar
runtime code in selfdrive/car/i30/longitudinal_map.py.
"""

import numpy as np

PEDAL_BP = np.array([0.0, 3.0, 6.0, 10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 55.0, 75.0, 100.0])
RPM_BP = np.array([800.0, 1100.0, 1400.0, 1700.0, 2000.0, 2400.0, 2900.0, 3600.0])


def bilinear_basis(p, n):
  """Weights of each table cell for bilinear interpolation (clamped)."""
  pi = np.clip(np.searchsorted(PEDAL_BP, p, side='right') - 1, 0, len(PEDAL_BP) - 2)
  ni = np.clip(np.searchsorted(RPM_BP, n, side='right') - 1, 0, len(RPM_BP) - 2)
  pf = np.clip((p - PEDAL_BP[pi]) / (PEDAL_BP[pi + 1] - PEDAL_BP[pi]), 0, 1)
  nf = np.clip((n - RPM_BP[ni]) / (RPM_BP[ni + 1] - RPM_BP[ni]), 0, 1)
  rows = np.arange(len(p))
  W = np.zeros((len(p), len(RPM_BP) * len(PEDAL_BP)))
  for dn, wn in ((0, 1 - nf), (1, nf)):
    for dp, wp in ((0, 1 - pf), (1, pf)):
      W[rows, (ni + dn) * len(PEDAL_BP) + pi + dp] += wn * wp
  return W


def pav(y, w):
  """Weighted isotonic (nondecreasing) regression."""
  y, w = list(map(float, y)), list(map(float, w))
  blocks = [[y[i], max(w[i], 1e-6), 1] for i in range(len(y))]
  i = 0
  while i < len(blocks) - 1:
    if blocks[i][0] > blocks[i + 1][0]:
      a, b = blocks[i], blocks[i + 1]
      tw = a[1] + b[1]
      blocks[i] = [(a[0] * a[1] + b[0] * b[1]) / tw, tw, a[2] + b[2]]
      del blocks[i + 1]
      i = max(i - 1, 0)
    else:
      i += 1
  out = []
  for v, _, c in blocks:
    out += [v] * c
  return np.array(out)


def fit_engine(p, n, T, iters=4, reg=0.3):
  """Robust bilinear least squares with smoothness regularisation, then monotone in pedal."""
  W = bilinear_basis(p, n).astype(np.float32)
  m = W.shape[1]
  # Second-difference smoothness along pedal and rpm.
  P, R = len(PEDAL_BP), len(RPM_BP)
  D = []
  for r in range(R):
    for c in range(1, P - 1):
      row = np.zeros(m)
      row[r * P + c - 1 : r * P + c + 2] = [1, -2, 1]
      D.append(row)
  for c in range(P):
    for r in range(1, R - 1):
      row = np.zeros(m)
      row[(r - 1) * P + c] = 1
      row[r * P + c] = -2
      row[(r + 1) * P + c] = 1
      D.append(row)
  D = np.array(D) * reg
  w = np.ones(len(T))
  for _ in range(iters):
    AtA = D.T @ D + np.eye(m) * 1e-6
    Atb = np.zeros(m)
    for s in range(0, len(T), 200000):
      Wc = W[s : s + 200000] * w[s : s + 200000, None]
      AtA += Wc.T @ W[s : s + 200000]
      Atb += Wc.T @ T[s : s + 200000]
    x = np.linalg.solve(AtA, Atb)
    res = T - W @ x
    w = np.minimum(1.0, 4.0 / np.maximum(np.abs(res), 1e-6))  # Huber, 4 % torque
  support = W.sum(0)
  table = x.reshape(R, P)
  for r in range(R):
    table[r] = pav(table[r], support.reshape(R, P)[r] + 1e-3)
  return table, support.reshape(R, P)


def engine_torque(table, p, n):
  return bilinear_basis(np.atleast_1d(p), np.atleast_1d(n)) @ table.ravel()
