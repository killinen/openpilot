# i30 longitudinal: pedal/coast map, old-API integration and early lift-off

Hyundai i30 GD 2014, manual gearbox, accelerator interceptor only (no automatic braking), old
openpilot longitudinal API. This directory holds the offline study and simulation harness; the
runtime code lives in `selfdrive/car/i30/` and `selfdrive/controls/lib/`.

Nothing here has been deployed to the C3X, committed or road-tested. All results are
offline (held-out logs and simulation). **Road validation is the remaining gate.**

## What changed (live-control path)

| Change | Where | Default |
|---|---|---|
| Old-API trajectory honours the planner's upper accel limit (profile, turn, ramp) | `longitudinal_planner.py` `cap_trajectory_accel()` | on for every old-API car |
| Fitted pedal map: desired accel → interceptor command, with grade feed-forward, explicit legacy fallback, bumpless transfer and bounded/conditional integral | `selfdrive/car/i30/longitudinal_map.py`, `pedal_map.json`, `longcontrol.py`, `controlsd.py`, `carcontroller.py` | `I30_PEDAL_MAP_ENABLED = True` |
| Early lift-off: planner throttle veto from fitted coast capability | `selfdrive/car/i30/coast_lift.py`, `longitudinal_planner.py` | `I30_EARLY_LIFT_ENABLED = True` |
| Pitch passed to the controller only when `liveLocationKalman.calibratedOrientationNED.valid`, otherwise NaN (no grade feed-forward) | `controlsd.py` | always |
| (previous commit) old-API i30 honours `allowThrottle` and freezes the integral | `longcontrol.py` | unchanged |

Both i30 flags are in `selfdrive/car/i30/values.py`; setting either to `False` restores the
previous behaviour for that part exactly. Preserved behaviour (tested):

- Inactive or vetoed: interceptor command is exactly 0 with `ENABLE=0`.
- Driver gas override, brake and clutch still end longitudinal control through the existing
  events. On re-engagement the controller starts from reset, with no integral carried over.
- Lead and stop deceleration requests are never weakened. The trajectory cap removes only
  acceleration above `max(limit, 0)` and only ever lowers speeds. The lift-off veto only removes
  throttle. Neither touches the MPC lead model, the published decelerations or FCW.
- Intentional cruise, speed-limit and curve reductions are unchanged, because the cap is
  positive-only.
- New-API configuration or a missing map pedal: the carcontroller falls back to the legacy
  `accel / 1.6` conversion.

Not integrated: the MPC initial-state gap limiter (`lookup.coast_state_candidate`). It changed
nothing measurable in any simulated scenario (see below).

## Data

**Source.** OP-NAS log-data API (`POST /api/log-data/query` with CAN and log signals per
segment), plus the API's raw-log route `GET …/segments/{n}/log` with SHA-256 checked and parsed
locally in the repository Docker image. The two paths produce identical decoded CAN and fields;
this was checked on one segment fetched both ways. Every sample keeps device/route/segment.

**Inventory (2026-11-12).** The API lists 676 drives. Four devices are this i30: the C3X
`ff528bde139e99de` (186 drives, 2,875 segments, FrogPilot 0.9.7 with interceptor) and three older
devices `41c5cb6e`, `23a0dabf`, `0be758db` (273 drives, 5,288 segments, 0.8.13/0.8.15, **no
interceptor**). There are no duplicate routes. Six C3X drives start at a non-zero segment or have
gaps. Those segments are missing on OP-NAS, not broken. The C3X's own 625-segment retention is
not the dataset.

**Fetched and analysed.** Throughput was limited by OP-NAS (drive uploads in progress drop the
query service to one reader). Segments were taken round-robin across routes, so even partial
fetches spanned every C3X drive.

- **The fit uses all 2,875 C3X segments and nothing else.** 184 of the 186 drives have usable
  rows. That is 35.9 h of moving, clutch-engaged driving, 29.4 h engaged and 11.1 h engaged
  following. Counts are in `results/2026-11-12-v3/`.
- **The old devices are not used.** They run other software and have no interceptor (see below).
- **The first fit used 735 segments** (`results/2026-11-12-v2/`). The full archive confirmed it,
  see Validation.

**Signals.** Everything physical is decoded from CAN with the i30 DBC, so it doesn't depend on the
software revision:

- `EMS_DCT1.PV_AV_CAN`: the ECU accelerator position, whether it comes from the driver or the
  interceptor.
- `N`, plus engine torque `TQI − TQFR` (% of `TQ_STND` = 230 Nm).
- `TCS5` wheel speeds.
- `ESP2.LONG_ACCEL`: the ESC accelerometer, i.e. specific force including grade.
- Brake via `BRAKE_ACT` and `CYL_PRES`.
- `GAS_COMMAND` (sent) and `GAS_SENSOR`.

Pitch is taken exactly as the controller sees it: `carControl.orientationNED`, i.e.
`liveLocationKalman.calibratedOrientationNED`. **livePose pitch differs by a constant +0.011 rad
(≈0.11 m/s² of grade) on the C3X.** Fitting the grade term to livePose would therefore bias
runtime grade compensation.

**Invalid pose.** On some C3X drives `liveLocationKalman` stays `uninitialized` for the whole
drive. `calibratedOrientationNED.valid` is then False and the value is frozen at about 1e-9 rad.
controlsd used to pass that value on as a flat road. Two fixes:

- **Fit.** Pitch only counts where the pose is valid.
- **Runtime.** controlsd now passes NaN, so there is no grade feed-forward and the integral
  trims.

Invalid pose covered 0.6 % of engaged time; those drives were mostly driven manually.

**Facts established from the data:**

- The interceptor command reaches the ECU within one 20 ms frame. Fitted robustly:
  `PV_AV_CAN = 1.061·cmd·100 + 0.37 %` (344k samples). Most drives agree within 0.4 %; a few
  carry offsets of 2–3 %.
- RPM/speed ratio clusters are tight: 461 / 248 / 173.4 / 130.9 / 106.0 / 91.9 rpm per m/s
  (MAD 5.3 / 1.8 / 0.77 / 0.36 / 0.21 / 0.11). Gears 4–6 match `gear.py`. Gears 1–3 are distinct
  clusters in the data; no labels were assumed.
- `LONG_ACCEL = 1.00·dv/dt + offset` with zero lag (correlation 0.87), so
  `LONG_ACCEL − dv/dt` is an independent grade measurement. It agrees with the controller pitch
  (`grade_esp = 0.948·g·sin(pitch) + 0.097`). Per C3X drive the gain is 0.95–1.0, correlation
  about 0.85. Body-pitch confounding is small: 0.05 m/s² of grade error per 1 m/s² of
  acceleration.
- **Why the older devices are excluded:**
  - They give 8–12 % less torque at the same `PV_AV_CAN` than the C3X with interceptor.
  - Their pose maps to ESC grade with gains of only 0.37–0.70. Pooled in, they halved the
    fitted grade gain (0.95 → 0.49).
  - Pooling them into the vehicle law biased held-out C3X acceleration by −0.012 m/s².

  C3X interceptor and C3X driver pedal agree within about 1 %.

## Audit of the previous study (commit a3896569d)

| Item | Finding |
|---|---|
| Data volume | 119 + 74 qlogs from 12 drives, against 2,875 C3X segments on OP-NAS. |
| Input signal | The commanded pedal (`carOutput`) was used and only engaged windows were kept. Driver pedal, the only source of high-pedal data, was discarded, so pedal above 35 % was almost unsupported. |
| Gear | `estimatedGear` only knows 4–6, so gears 1–3 were excluded by construction. |
| Delay | A fixed 0.4 s pure delay was used. Measured: 0.04 s dead time, then a torque rise lag of τ = 0.35 s (fall 0.2 s), then about 0.08 s torque→wheel. Transient torque MAE is 2.7 % with these dynamics vs 5.5 % static. A re-check on 40 other drives gives the optimum at τ_up 0.25 s with MAE 3.13 vs 3.15 for the shipped values, so the choice is insensitive. |
| Grade | It used the same pitch signal as runtime (correct). The gain (0.90) was fitted on coast windows only. The independent ESC grade gives 0.93. |
| Validation | Two disjoint drives, but the metrics only covered 40 % of windows. On my held-out drives, restricted to drives it never trained on, its map covers 47 % of samples. |
| Inverse | It was never evaluated against logged pedal. |
| Simulation | The plant was `coast + 2.5·pedal` with no gears, grade or torque dynamics, and the planner was hand-reimplemented. These results don't transfer. |

## Model (runtime, `longitudinal_map.py`)

```
T [% of 230 Nm] = table(ECU pedal %, rpm)                         12 × 8 values, monotone in pedal
a = k_gear · T · rpm / v − c0 − c2·v² − (0.948·g·sin(pitch) + 0.097)
```

`k·T·rpm/v` is the power balance (wheel force = engine power / speed). The gear enters only
through the **measured** rpm/v ratio, which must fall inside a `gear.py` window. k ≈ 1.46–1.50e-4
in gears 3–6 implies m_eff ≈ 1,450–1,500 kg at η ≈ 0.92, which is physically consistent. Gears 2
and 1 have a lower k (1.40e-4, 1.17e-4) because of their larger rotational inertia.

With the full archive every table cell has data. At 1,100–1,700 rpm torque is flat above
25–30 % pedal: for example 49.7 → 51.4 % torque from 30 to 100 % pedal at 1,400 rpm. These
rows are therefore marked `saturates`, and the map holds the knee there; by design that gives
up at most 2 % torque.

- **Table fit.** The table is fitted on lag-filtered pedal (fitted dynamics) over all samples,
  with smoothness regularisation and a monotone projection. Fitting only on held pedal left
  pedal above 40 % uncovered.
- **Inverse.** Each torque row has a minimum slope (0.25 %T per % pedal). Up to `max_pedal`,
  pedal is found by interpolation.
- **Knee vs coverage edge.** Above `max_pedal`, the map holds the knee only where the data show
  real torque saturation (`saturates`). A pure coverage edge returns `None`.
- **Explicit fallback.** `None` is returned for: no gear window (clutch, shift, gears 1–3), speed
  outside 5–40 m/s, rpm outside the table, unsupported cells, or NaN inputs.
- **Cost.** One call takes about 11 µs on the desktop, and the old-long cycle grew by 39 µs. C3X
  timing has not been measured.

## Validation (held-out whole drives, `results/2026-11-12-v3/`)

There are 56 held-out drives with data (126 for training): a deterministic hash split by route
name, plus Codex's two validation drives. The model never saw them. Adding segments only adds
data inside the same split. Coverage and error are always reported together.

**Forward prediction** (instantaneous 0.48 s acceleration):

| Subset | Windows | Physical (dynamic): coverage / MAE / bias | Codex map: coverage / MAE / bias |
|---|---:|---|---|
| All held-out | 389,788 | 99.8 % / 0.096 / −0.011 | 55.9 % / 0.097 / −0.041 |
| Drives Codex never trained on | 373,982 | 99.8 % / 0.097 / −0.012 | 54.7 % / 0.097 / −0.042 |
| Coast ≥1 s, non-Codex drives | 60,535 | 99.8 % / 0.081 / −0.023 | 62.5 % / 0.080 / −0.042 |
| OP-commanded pedal | 256,288 | 100 % / 0.092 / −0.006 | 59.9 % / 0.098 / −0.047 |
| Driver pedal | 133,500 | 99.5 % / 0.104 / −0.023 | 48.1 % / 0.094 / −0.025 |
| Gear 6 | 169,102 | 100 % / 0.096 / −0.007 | 48.9 % / 0.096 / −0.047 |
| Gear 4 / 5 | 46,245 / 141,755 | 99.5 % / 0.092 · 100 % / 0.085 | 76.7 % / 0.105 · 70.4 % / 0.094 |
| Gear 3 / 2 | 14,764 / 17,131 | 98.4 % / 0.125 · 99.1 % / 0.173 | not covered |
| Pedal 25–40 % | 63,856 | 100 % / 0.113 / −0.006 | 22.1 % / 0.120 / −0.070 |
| Pedal 40–100 % | 28,478 | 98.7 % / 0.125 / −0.063 | 19.5 % / 0.109 / −0.006 |

**Against the 735-segment fit.** Scored on these same held-out samples, the earlier model has
coverage 99.5 %, MAE 0.097 and bias −0.009, with pedal 40–100 % covered 96.7 % (MAE 0.127). Its
v2 report quoted 0.106, but that was on fewer held-out samples. **More data did not make
forward prediction more accurate; it confirmed it.** The gains are:

- coverage (every table cell is now backed by data);
- grade calibration from 864k C3X samples;
- far more transient and lift-off evidence.

Static vs dynamic: the same model without pedal dynamics has 0.116 overall, 0.148 for driver
pedal. Accuracy where both maps apply is about the same. The gain over Codex is about twice the
coverage, gears 2/3/6, high pedal, transients and a smaller bias.

**Coast.** Both maps slightly overestimate coast deceleration, i.e. braking is optimistic:
−0.02 m/s² (physical) and −0.04 (Codex). The lift-off margin covers that (see Planner).

**High pedal** is the largest physical-map error: bias −0.06, under-predicting acceleration.

- **Transient** (5 s open-loop speed from logged pedal, 3,353 windows on 56 drives):

  | Model | RMS end-speed error | Bias |
  |---|---:|---:|
  | Dynamic | 0.36 m/s | −0.05 |
  | Static | 0.38 m/s | |
  | Codex (1,766 windows) | 0.41 m/s | −0.22 |
  | Constant acceleration | 1.28 m/s | |

  Grade error is the main remaining term. Using the non-causal ESC grade instead lowers the
  instantaneous MAE by about a third.
- **Inverse** (runtime class on settled-pedal held-out points, measured acceleration → command
  vs logged command):
  - Coverage 95.3 %. Fallbacks: 10,219 gear window (clutch or between gears), 151 rpm; none for
    pedal coverage.
  - MAE 2.7 pedal-% (bias −1.5, p95 6.6). Per gear: 4th 2.0, 5th 2.1, 6th 3.4.
  - Conditioning: 2.3 pedal-% per 0.1 m/s² (p95 5.6), so no ill-conditioned amplification.
  - For comparison, the legacy static conversion has 9.4 % MAE; it relies entirely on the
    integral for road load.
  - On the same samples the 735-segment map scores 2.4 % (bias −1.2). In raw ECU pedal, on
    samples with pedal above 3 %, it scores 3.9 vs 4.4 % for the new map. **The new map is
    about 0.5 pedal-% worse in steady state.** Both maps under-predict settled pedal by 2–3 %;
    the bounded integral trims that. In closed loop the difference doesn't show (see
    simulation).

**Controller replay on real inputs** (`replay_controller.py`, 608 segments, 4.8 engaged hours,
open loop):

- The map is active ≥99.95 % of engaged time. Fallback no longer happens at coverage edges,
  because every row now has a data-backed knee. Gear-window gaps between shifts disengage anyway.
- Pedal steps: p99 0.0055 per 10 ms (legacy 0.0048). The largest single step is 0.37, against
  0.96 for legacy on the same inputs.
- The mean command is 0.177 against 0.157 logged. Zero-pedal time is 12 % against 18.5 % for
  legacy: the map uses partial pedal where legacy over-coasted.
- The legacy replay reproduces the logged command to MAE 0.009 / r 0.986, so the harness is
  faithful. The 735-segment run had gaps that restarted the controller and gave only 0.042 /
  0.86.
- Because the speed error is the logged legacy one, this replay checks behaviour on real inputs,
  not outcomes.

**Lower gears.** Gears 2 and 3 are calibrated physically from their measured ratio clusters, but
are marked `controllable: false`. Their accuracy is lower (MAE 0.15–0.19), and more importantly
the clutch logic in `carstate.py` treats any ratio outside the 4–6 windows as clutch pressed,
which disengages. Enabling 2nd/3rd gear control would change disengagement behaviour and is a
separate decision. Gear 1 (2 min of data) is exported for reference only.

## Controller integration (`longcontrol.update_old_long`)

- The PID still works in physical acceleration. The map turns its output into pedal, so road
  load and grade come from the map, not the integral.
- **Integral.** In map mode it is bounded to ±0.5 m/s² (it trims model error only, so it may go
  negative). It is frozen while the pedal is saturated in the direction of the error: zero pedal
  with an overspeed error, or the knee with an underspeed error. The existing veto freeze and the
  overspeed cap still apply.
- **Map ↔ legacy switch (bumpless).** The integral is re-solved so the new conversion reproduces
  the last pedal, and that cycle is recomputed. Pedal increases are slew-limited to 0.4/s for
  1 s; decreases never are.
- **(Re)engagement** starts from reset. There's nothing to transfer from the inactive zero pedal;
  a test covers this after a bug where engagement inherited a −0.5 integral.
- `max_desired_acceleration` is applied before mapping. The throttle veto forces exactly 0 after
  mapping, because the map gives positive pedal for mild decelerations above coast.

## Planner

**Trajectory cap.** On the old API the published `speeds`/`accels` drive the PID, while profile,
turn and ramp limits only clipped the scalar `aTarget`. MPC `a_max` is a fixed 2.0. In logged
engaged C3X driving, the first trajectory step exceeded the FrogPilot limit 1.3 % of the time
(by more than 0.3 m/s² 0.5 % of the time; median excess 0.25, max 1.5). The cap uses the same
rate-limited `accel_clip[1]` as `aTarget` and is applied before the MPC's next initial state.

**Early lift-off.** Needed deceleration is the constant ego acceleration that brings closing speed
to zero at `t_follow·v_lead + 6 m`, after 0.5 s of continued closing, assuming any lead
deceleration continues. Throttle is vetoed when the need exceeds 0.7 × coast capability, and
released below 0.28 ×. Coast capability is fitted for the current gear, speed and grade; when it
is unknown, a weak −0.15 m/s² is assumed. The 0.7 threshold comes from a sweep over 0.3–1.0 on
noisy-lead scenarios.

Replayed observationally on logged engaged following (11.7 h of 30.9 engaged hours, full C3X
archive; not a counterfactual outcome):

- The veto would have been active 11.9 % of following time and removed 1,251 s of logged
  throttle.
- 520 of 965 driver brake applications during following (54 %) were preceded by it, median
  6.9 s earlier.
- 1 s before those brake applications the logged OP pedal was still a median 5.5 %.

The 735-segment replay gave the same picture: 105 of 218, median 5.5 s.
- Logged `allowThrottle` was **never** false in the engaged logs, so the previous commit's veto
  fix has not acted on any logged drive.

The fitted coast bias (−0.02 m/s², optimistic) is within the margin the 0.7 threshold leaves:
0.3 × 0.43 ≈ 0.13 m/s².

**State-gap limiter.** It changed no outcome in any scenario, with or without noise (identical
minimum gaps to `map+lift`). It is not integrated.

## Closed-loop simulation (`closed_loop.py`)

The real `LongitudinalPlanner` (production FrogPilot weights) and the real `update_old_long`
drive a plant with the fitted torque dynamics, gear physics, drag and grade. The **plant is fitted
on all drives and perturbed** (torque ×0.85 / 1.0 / 1.15, resistance ±0.1 m/s²); the controller
uses the training-only calibration. Scenarios: steady, cruise drop, speed-limit drop, slowing,
braking and hard-braking lead, cut-in, approach, lead loss/change, uphill, downhill,
downhill-with-lead (insufficient authority), driver gas override, brake tap, clutch/downshift
re-engagement and pitch bias. Each runs with and without vision-like lead noise.

**480 runs** with the full-archive map and plant (240 nominal-lead, 240 noisy-lead, all plants):
0 solver failures, and the interceptor command was exactly 0 in every inactive cycle. Selected
nominal-plant results follow; the full table, including worst case over plants, is in
`results/2026-11-12-v3/closed-loop-table.md`. The 735-segment map's run is in `2026-11-12-v2/`
and gives the same conclusions. The legacy column changed because the plant changed: the C3X-only
plant is harder on the fixed `accel/1.6` conversion.

| Scenario (metric) | legacy | map | map + lift |
|---|---:|---:|---:|
| Steady cruise, RMS speed error m/s | 0.37 | 0.01 | 0.01 |
| Uphill 5 %, RMS / max error m/s | 0.68 / 1.46 | 0.05 / 0.16 | 0.05 / 0.16 |
| Downhill 5 %, RMS / max error m/s | 0.46 / 0.80 | 0.20 / 0.33 | 0.20 / 0.33 |
| Pitch bias 0.57°, RMS error m/s | 0.37 | 0.09 | 0.09 |
| Cruise drop 22→16 m/s: settle down / settle back s | 11.5 / 8.3 | 14.2 / 10.9 | 14.2 / 10.9 |
| Clutch + downshift re-engage: settle s | 26.2 | 4.1 | 4.1 |
| Driver gas release: max underspeed m/s | 0.80 | 0.36 | 0.36 |
| Slowing lead: min gap m, nominal / worst plant | 27.1 / 24.7 | 25.1 / 18.1 | 31.7 / 29.0 |
| Slowing lead + noise: min gap m | 20.1 | 22.6 | 30.8 |
| Approach slower lead: worst-plant min gap m | 22.7 | 15.5 | 28.1 |
| Lead change after loss: min gap m | 9.6 | 14.4 | 15.1 |
| Lead brakes −1 m/s² for 8 s + noise: min gap m | −36.3 | −32.7 | −20.4 |
| Cut-in 25 m ahead, 4 m/s slower: min gap m, nominal / worst plant | 5.4 / 0.2 | 5.8 / 0.9 | 5.8 / 0.9 |
| Lead brakes −2.5 m/s² for 4 s: min gap m | −81.0 | −84.2 | −82.3 |
| Downhill 6 % behind slowing lead | contact | contact | contact |

What the simulation shows:

- **The map fixes steady-state and grade tracking**, because road load comes from physics instead
  of the slowly built integral. It also fixes re-engagement after gear changes: the legacy integral
  has to rebuild road load from zero.
- **The map on its own reduces some following margins** (slowing lead and approach, worst plant).
  Legacy "negative request → zero pedal" coasted harder than planned, which gave accidental early
  lift. **The map must ship with lift-off**, and together they beat legacy in every closing
  scenario.
- **Cruise and speed-limit reductions settle 2.7 s slower with the map**, settling back 2.6–3.6 s
  later. It follows the planned gentle deceleration with partial pedal instead of over-coasting.
  The undershoot is 0.39 m/s, against 0.26–0.35 for legacy and 0.25–0.28 with the
  735-segment map. That is the one closed-loop metric where the full-archive map is worse.
- **No variant can handle a lead braking harder than coast authority** (−1 to −2.5 m/s², or
  downhill). Negative gaps mean the point-mass trajectories crossed; the driver must brake.
- **The state-gap limiter (`map+lift+gap`) is identical to `map+lift`** within 0.02 m in every
  scenario.
- **The lift-off threshold sweep** (0.3–1.0, noisy leads, results in `lift-sweep-*.json`, made
  with the previous calibration) chose 0.7. Lower values mainly lengthen headway on approach
  (2.9–4.0 s vs 2.7 s).
- **A bug the simulation found and fixed:** map/legacy chattering at a coverage boundary while the
  pedal was saturated drove the integral to −0.4. The transfer floor and 0.5 s re-entry delay fix
  it, and a regression test covers it.

Limits: point-mass plant, perfect gear knowledge inside windows, scripted driver, AR(1) lead
noise only (no detection latency, dropouts or false leads), fixed `t_follow`. Simulation results
are not independent evidence that the inverse map is right; the held-out log metrics above are.

## Remaining limitations

- **Braking authority.** Coast capability is median −0.43 m/s² (p5 −0.70, p95 −0.13) while
  following. Any lead braking harder than that for more than a few seconds needs the driver's
  brake. Lift-off reduces the overlap but cannot remove it. Downhill coasting cannot slow the car
  at all. The existing FCW assumes −3.5 m/s² braking and fires far too late for this car. A
  "brake now" alert based on `CoastLiftOff.needed` vs capability is recommended, but it needs a
  new event, so it isn't implemented here.
- **Largest map errors:** the high-pedal forward bias (−0.06 m/s²), the coast bias (−0.02 m/s²,
  optimistic braking) and the 6th-gear inverse (MAE 3.4 pedal-%, bias −2.0). The bounded
  integral absorbs them; grade (LLK pitch) is the largest remaining error source.
- **The C3X archive is exhausted.** All 2,875 segments are used, so more coverage needs new
  driving. Still thin:
  - 5th gear above 30 m/s (2.4 min) and 4th above 25 m/s (1.5 min);
  - gear 1 above walking pace (under 1 min above 5 m/s);
  - grades steeper than 5 % (|grade| > 4 % totals 124 min; the p1/p99 grades are −5.1 / +5.8 %).

  Covered well now: pedal above 60 % in gears 4–5 (8.7 / 10.8 min) and gear 4 at 20–25 m/s
  (2.3 min). The older devices are deliberately not used.
- **Actuator delay setting.** `longitudinalActuatorDelay` stays at 0.2 s. The measured response
  (≈0.1 s dead time + 0.35 s rise) suggests about 0.4 s for the feed-forward horizon, but that
  wasn't changed without road data.
- **Gears 2/3 are not controllable** (clutch logic, see above). There is no C3X timing
  measurement, and nothing has been road-tested.

## Reproduction

Data and analysis tools live outside the repo in `~/OPstuff/i30-long-data` (`I30_LONG_DATA`);
the analysis copies are in `opnas/`. Steps:

1. `opnas/fetch_interleaved.py` (host) and `opnas/rlog_fetch.py` (Docker, `pip install
   zstandard`) fetch into `raw/`. Both are resumable.
2. `convert.py` → `features.py` → `compact.py`. These are numpy-only and run on the host.
3. `dyn.py` (and the `dyn_check.py` robustness check) → `fit_model.py --all` →
   `export.py plant.json model_all.json`, then `fit_model.py` → `export.py` (writes
   `selfdrive/car/i30/pedal_map.json`).
4. `inverse_eval.py [calibration.json output-name.json]` (scores any map on the same samples),
   `transient_eval.py`, `coverage.py`, `lift_replay.py`.

The fit and the dynamics use only the C3X rows (`rows_ff528bde139e99de.npz`). Pitch counts only
where the pose was valid.

Simulation and tests run in the Docker workflow from `AGENTS.md`, after
`scons msgq_repo/msgq/ipc_pyx.so selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so opendbc_python`:

```bash
python -m tools.car_porting.i30_longitudinal.closed_loop --calibration map.json --plant plant.json --output /tmp/cl [--lead-noise]
python -m tools.car_porting.i30_longitudinal.closed_loop_metrics --input /tmp/cl --output /tmp/cl/metrics.json
python -m tools.car_porting.i30_longitudinal.closed_loop_table --clean /tmp/cl --noise /tmp/cl_noise
python -m tools.car_porting.i30_longitudinal.replay_controller --raw $I30_LONG_DATA/raw/ff528bde139e99de --output replay.json
python -m pytest -c /dev/null --confcutdir=selfdrive/car/i30/tests -p no:cacheprovider \
  selfdrive/controls/lib/tests/test_longcontrol.py selfdrive/controls/lib/tests/test_trajectory_accel_cap.py \
  selfdrive/car/i30/tests/test_longitudinal_map.py selfdrive/car/i30/tests/test_pedal_command.py \
  tools/car_porting/i30_longitudinal/test_lookup.py tools/car_porting/i30_longitudinal/test_fit.py tools/car_porting/i30_longitudinal/test_coast.py
```

Don't refresh the container copy of the checkout while a simulation is running. `LongControl`
reloads the repo's `pedal_map.json` on every run even when `--calibration` is given. A refresh
mid-run has crashed a run twice; both were rerun from scratch.

Result sets:

- `results/2026-11-12-v3/`: the full C3X archive. This is what `pedal_map.json` holds now.
- `results/2026-11-12-v2/`: the 735-segment fit, kept unchanged for comparison.
- `results/2026-11-12/`: Codex's study, kept for reference. Its `simulate.py` results are
  superseded by `closed_loop.py`.
