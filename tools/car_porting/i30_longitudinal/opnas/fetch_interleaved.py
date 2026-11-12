# ruff: noqa: B905  (host analysis runs Python 3.8, which has no zip(strict=))
"""Fetch segments round-robin across routes (C3X first) so partial data covers many drives."""

import json
import random
import sys
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
import fetch

jobs = json.load(open(fetch.ROOT / 'segment-inventory.json'))
order = []
for devices in (['ff528bde139e99de'], ['41c5cb6e', '23a0dabf', '0be758db']):
  by_route = defaultdict(list)
  for dev, route, seg in jobs:
    if dev in devices:
      by_route[(dev, route)].append(seg)
  rng = random.Random(0)
  for segs in by_route.values():
    rng.shuffle(segs)
  keys = sorted(by_route)
  rng.shuffle(keys)
  depth = max(len(v) for v in by_route.values())
  for i in range(depth):
    for k in keys:
      if i < len(by_route[k]):
        order.append((k[0], k[1], by_route[k][i]))
(fetch.ROOT / 'fetch-order.json').write_text(json.dumps(order))
t0 = time.time()
with ThreadPoolExecutor(int(sys.argv[1])) as ex:
  for i, (job, res) in enumerate(zip(order, ex.map(lambda j: fetch.fetch(*j), order))):
    if res != 'cached':
      print(i, round(time.time() - t0), *job, res, flush=True)
