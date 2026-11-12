# i30 longitudinal calibration and coast-state experiment

This directory contains workstation analysis and **inactive candidate code**. Nothing here is imported by the running planner/controller, and no C3X settings or software were changed. It implements the offline calibration stage and tests a bounded planner-state experiment. Production integration of these candidates and road validation remain outstanding. The accompanying change in `selfdrive/controls/lib/longcontrol.py` makes the i30 old API honor the existing throttle veto and freezes integral growth while vetoed; it does not load this experimental map. The broader scalar acceleration-profile/turn-limit trajectory issue remains separate.

The runtime candidate uses three tables of 3 speeds × 8 pedal positions: **72 acceleration values**, a pitch gain, and coverage flags. A lookup needs scalar interpolation, one sine and at most eight pedal knots. It adds no optimizer, neural model, background training or per-frame file I/O. The measured desktop inverse-lookup cost was approximately 6–7 µs; C3X timing has not been measured. Extraction, regression and simulation run only on the workstation.

## Negative acceleration gap: what it can and cannot do

The existing gas-interceptor speed PID already prevents its integral from becoming negative. Its negative output is immediately converted to zero pedal. Clipping a final −2 m/s² output to −0.5 m/s² therefore does not improve throttle recovery by itself. Clipping the negative feedforward before adding the existing positive integral can even reintroduce positive pedal.

There is another state: the planner's integrated desired speed and acceleration. This state can fall below what the actual car achieves while coasting. `coast_state_candidate()` investigates bounding that discrepancy **at the MPC initial state**, before solving, rather than flooring the lead's deceleration request or the published trajectory:

- Require valid inputs, speed ≥8 m/s and at least 0.5 s at zero pedal.
- Candidate acceleration floor: `min(coast_accel, 0) − 0.15 m/s²`.
- Candidate speed floor: `vEgo − 0.5 m/s` (1.8 km/h).
- Preserve forced deceleration and intentional reductions in the effective FrogPilot cruise target, including speed/curve caps. Checking only the dashboard set speed is insufficient.
- The caller must reject stale data, driver gas/brake/clutch, unknown gear and gear transitions. The evaluator demonstrates those gates; the scalar helper alone does not inspect vehicle messages.

The gaps are **experiment values**, not established tuning recommendations. Changing the MPC starting state can alter future throttle decisions even though the lead observations are unchanged, so offline gap counts alone do not prove better following. Do not connect this helper directly to published `speeds`, `accels` or final pedal commands.

## Results from 2026-11-12

The training selection comprises 119 qlogs from route counters `0000061b`–`00000624`, 114.42 logged minutes and 5,111 usable one-second calibration windows. Validation uses two separately downloaded drives, `00000614--b9716077be` and `00000615--dc696323e9`: 74 qlogs, 73.48 logged minutes and 1,278 usable windows. No validation routes were fitted. Their older code revision has unchanged pedal, gear-estimation and longitudinal PID code; the carState difference concerns steering torque sign only.

The fit separates coast-down from positive pedal response. Coast-down uses a gear-specific intercept/speed term and a shared pitch coefficient. Positive pedal response has constrained positive piecewise slopes, interpolated with speed. The exported table is monotone in pedal. Delay is fixed at 0.4 s from the earlier exploratory work, not selected on these validation routes. The fitted pitch gain is 0.902 times the nominal gravity term.

| Independent validation | Result |
|---|---:|
| Forward prediction coverage | 510 / 1,278 windows (39.9%) |
| Mean absolute acceleration error, covered points | 0.066 m/s² |
| 95th percentile absolute error, covered points | 0.171 m/s² |
| Signed error (prediction − measurement) | −0.059 m/s² |
| Coast prediction coverage | 135 / 321 coast windows |
| Coast mean absolute error, covered points | 0.050 m/s² |
| Coast signed error | −0.040 m/s² |
| Inverse lookup available at measured acceleration | 493 / 1,278 windows |

These errors describe only supported inputs. Coverage requires at least ten training windows from two routes near both surrounding speed nodes and within the pedal interval. Speed is restricted to the training 5th–95th percentile per gear, and pitch to the training 99th-percentile absolute range. The inverse additionally rejects nearly flat response intervals to avoid amplifying small acceleration errors into large pedal changes. An inverse query returning a value is not evidence that its resulting pedal is correct in closed-loop operation.

The limited coverage, systematic prediction bias, inferred gear labels and potentially acceleration-dependent body pitch prevent treating this as a deployable full-range map. In particular, the negative coast bias slightly **overestimates braking capability**. More unsupported operating regions need data, and grade/coast uncertainty must be accounted for before planning earlier lift-off. Gears 1–3 remain uncalibrated.

On the original logs, after applying the effective cruise caps and forced-deceleration gates, the state-gap candidate changed speed initialization for **73.8 seconds** and acceleration initialization for **75.0 seconds**, out of 382.3 eligible seconds. These sets overlap. This is an observation of logged initial states, not a replay of subsequent MPC decisions or vehicle motion. A diagnostic feedforward-only clamp also produced five 10 Hz samples with a positive reconstructed output despite an originally nonpositive request; asynchronous message timing makes that illustrative rather than proof of an on-road event.

The first 20-run MPC experiment used the solver constructor's unit cost weights. It reduced internal state drift but did not improve minimum lead gap or consistently improve throttle return. Those results are retained as `unit-weight-exploration.json` and `unit-weight-comparison.json`, and must not be interpreted as validation of the production FrogPilot weighting. The follow-up harness explicitly uses `A_CHANGE_COST`, `DANGER_ZONE_COST` and `J_EGO_COST`, matching the nominal production weight scales.

The follow-up compares baseline, the state-gap limiter, and a separate early-lift candidate on six scenarios and two coast capabilities. `coast.py` adds closing-distance allowance from relative speed, one second of predicted lead deceleration and known coast capability; its only output is a throttle veto, with release hysteresis. It remains offline. This does not provide braking authority, guarantee stopping distance or handle vision uncertainty.

The corrected comparison completed **36 runs / 43,200 MPC steps with zero solver failures**. Minimum simulated lead gaps:

| Scenario; coast capability | Baseline | State-gap limiter | Early lift-off |
|---|---:|---:|---:|
| Slowing lead; −0.35 m/s² | 47.57 m | 47.57 m | 48.82 m |
| Braking lead; −0.35 m/s² | 3.72 m | 3.72 m | 5.63 m |
| Cut-in; −0.35 m/s² | 5.76 m | 5.76 m | 5.76 m |
| Approach to a slower lead; −0.15 m/s² | 36.26 m | 36.26 m | 42.69 m |

The state limiter reduced the braking-lead scenario's maximum speed-state deficit from 3.28 to 0.52 m/s without improving minimum gap. Early lift-off improved some gaps; it did not help the sudden cut-in where pedal was already removed immediately. Steady cruise and intentional cruise-speed reductions remained unchanged by early lift-off. Weak coast-down (−0.15 m/s²) still produced simulated contact in the harder braking/cut-in cases for all variants. Negative gaps in the JSON mean trajectories crossed; the point-mass model continues through that crossing and subsequent behavior is not physically meaningful. Zero solver failures is a numerical result, not a safety claim.

**Current decision:** retain the map, gap limiter and early-lift rule as offline candidates. The map has insufficient independent coverage and a braking-capability bias; the early-lift comparison assumes perfect lead observations and known coast acceleration. The only active-path source edit is the separately tested old-API throttle-veto fix, not deployed to C3X. Before map integration, coordinate steady-state feedforward with the old PID integral to avoid double-counting road-load compensation. Before early-lift integration, validate coast uncertainty, lead loss/changes and driver/clutch behavior against recorded data and a broader scenario set.

Checked-in results are under `results/2026-11-12/`. Raw logs, aligned CSVs, full-rate CAN spot checks and build logs remain under `/tmp/i30-long-audit`; retain that temporary directory elsewhere before cleanup if needed.

## Tools and reproduction

- `extract.py`: qlogs to aligned CSV, including validity, driver inputs, inferred gear, commanded pedal, pitch, planner states and effective cruise target. Backward alignment accepts ages ≤150 ms for car/controls/gear and ≤350 ms for plan/radar. Errors are recorded and cause a nonzero exit.
- `fit.py`: constrained coast/response fit; requires distinct training and validation routes; exports candidate JSON and validation metrics.
- `lookup.py`: scalar forward/inverse lookup and inactive state-gap experiment. Unsupported inputs return `None`, requiring an explicit caller fallback. Exact zero while inactive must remain enforced by the eventual controller integration.
- `evaluate_gap.py`: observation of candidate state changes on existing logs, without claiming counterfactual vehicle behavior.
- `coast.py`: inactive early-lift candidate; scalar arithmetic and a latch, without additional optimization.
- `simulate.py`: actual repository longitudinal MPC and old PID coupled to an idealized pedal-only plant. The MPC is built specifically, without a full build or process replay. It uses perfect lead observations, known coast acceleration, fixed linear pedal response and 0.35 s actuator delay. It does not validate vision, grade estimation or the new map's inverse.

Use the Docker workflow in the repository `AGENTS.md`: mount the checkout read-only and copy tracked files into `/tmp/openpilot`. Mount `/tmp/i30-long-audit` as `/audit`. Inside the container copy:

```bash
PYTHONPATH=/tmp/openpilot python -m tools.car_porting.i30_longitudinal.extract --logs /audit/logs --output /audit/train-v3
PYTHONPATH=/tmp/openpilot python -m tools.car_porting.i30_longitudinal.extract --logs /audit/holdout/logs --output /audit/holdout
PYTHONPATH=/tmp/openpilot python -m tools.car_porting.i30_longitudinal.fit --train /audit/train-v3/samples.csv --validation /audit/holdout/samples.csv --output /audit/candidate
PYTHONPATH=/tmp/openpilot python -m tools.car_porting.i30_longitudinal.evaluate_gap --samples /audit/train-v3/samples.csv --calibration /audit/candidate/pedal-map.json --output /audit/candidate
PYTHONPATH=/tmp/openpilot python -m pytest -c /dev/null --confcutdir=tools/car_porting/i30_longitudinal tools/car_porting/i30_longitudinal/test_lookup.py tools/car_porting/i30_longitudinal/test_fit.py tools/car_porting/i30_longitudinal/test_coast.py -q
PYTHONPATH=/tmp/openpilot scons -j4 selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so
PYTHONPATH=/tmp/openpilot python -m tools.car_porting.i30_longitudinal.simulate --output /audit/candidate
```

The focused suites have **26 passing tests**: 9 old-controller tests and 17 calibration/lookup/coast tests. They cover prolonged throttle inhibition without integral growth, release, starting/inactive states, preserved negative demand, scope to the i30 interceptor, a known synthetic plant, monotonicity/inversion, sparse/invalid-input rejection, flat-response rejection, cruise-cap/forced-deceleration handling, state-gap behavior and early-lift hysteresis. Ruff formatting and lint pass. Tests isolate this directory because the repository-wide conftest imports unrelated compiled parameter modules; the actual MPC simulation separately builds its required native dependencies.
