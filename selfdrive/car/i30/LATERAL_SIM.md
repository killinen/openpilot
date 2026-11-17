# i30 TRQI lateral: offline closed-loop simulator

An offline closed-loop simulator for the TRQI steering path lives **outside this repo** at
`~/OPstuff/i30_lat_sim/`. Its `README.md` covers setup, parameters, metrics and validation.

It replays logged i30 drives through a model of:

- `latcontrol_torque`: friction and its threshold, setpoint low-pass, FF scale, Kp/Ki;
- the host slew and limit logic in `carcontroller.py`, including the catch-up slew and the 12 Nm output hold;
- the TRQI firmware torque map;
- a fitted EPS and rack plant.

The model's steering request is replayed from the log.

## When to use it

Run it before driving a change to any of these:

- `TrqiSteerLimitParams` slew or catch-up values (`values.py`);
- the TRQI rate limiter or output-torque guard (`carcontroller.py`);
- i30 torque-controller tuning: SteerFriction and its threshold, setpoint filter, FF scale, Kp scale, Ki;
- the TRQI firmware torque map shape.

Compare each candidate against the current settings on the same drives. Look at twitch metrics
(big snaps, wiggles, straight twitch) and at tracking and ramp-lag metrics together. A change that
helps one usually costs the other.

## When not to use it

- **Absolute event counts.** Use it only to rank candidates against each other.
- **Model, planner or curvature changes.** The model's request is replayed open loop, so it never
  reacts to where the simulated car goes.
- **Driver takeover, passthrough or driver-gate behaviour.** Use unit tests for these.
- **Speeds outside about 40–70 km/h.**
- **After an EPS, car or TRQI map change.** Refit the plant first (`refit_plant.py`).

## Decisions it informed

- **2026-11-05:** SteerFriction 0.14 with threshold 0.6, plus the 50 ms setpoint filter (1bc81708a).
- **2026-11-17:** catch-up slew of 15 Ncm/frame when |request − applied| > 300 Ncm, gated off on
  driver input. In replay it cut ramp-lag episodes by 80–90% and p99.9 tracking error by 40%, for
  +1–3% twitch.

After any such change ships, confirm it on real drives by comparing the same roads before and after.
Split by light (sun elevation) and rain: dark and rain alone make tracking worse by 9–40%.
