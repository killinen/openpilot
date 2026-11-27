# LS600h HRR lateral simulator

The closed-loop simulator for LS600h steering lives outside the repo at `~/OPstuff/ls600h_lat_sim/`.
Its `README.md` has the full method, file list and commands. This file records what it models and
what it has decided.

## What it models

- **Controller:** the `latcontrol_torque` replica, with the setpoint replayed from the logs.
- **HRR carcontroller:** low-pass α 0.3, rate limits 8.2 / 12.2 Ncm per frame, ±800 Ncm.
- **HRR firmware v0.1.1:** linear torque-sensor emulation in 0.1° steps, no filter.
- **OEM EPS assist map:** fitted from `steeringTorque` and `steeringTorqueEps`.
- **Rack:** Coulomb friction, damping and speed-dependent aligning stiffness, with a disturbance
  bias for each window.
- **Measurement:** the openpilot `VehicleModel`.

The plant was fitted on 21 drives (routes `000001c5`–`00000221`) and refit on 2026-11-27 on 28
drives, which add 7 drives on the deadband map. On held-out routes it explains 92–95% of
steering-angle variance. The two fits agree, so the car's steering has not changed.

**Plant facts:**
- The EPS gives almost no assist below about 30 column-torque units, about 0.1 HRR command.
- Rack friction is about 40–48 EPS units on top of that.
- So from rest the wheel does not move until the command reaches about 0.2.
- Assist falls with speed.

## Validation

Closed loop, sim vs log on 542 windows of 20 s:
- Torque rate matches within 2–4%.
- Small-correction gain matches within about 0.05, except at 30–50 km/h.
- RMS tracking error is 5–18% lower in the sim.
- Curve lag is under-predicted.

Out of sample (2026-11-27): the 2026-11-22 plant had never seen the map drives (877 windows) and
still matched them. RMS error was 0–10% low, torque rate within 2–4%, and the 50–70 km/h
small-correction gain was 0.70 in the sim vs 0.77 on the car.

## Decisions

| Date | Change | Sim result, median of 9 plant fits, vs the previous setting | Commit |
|---|---|---|---|
| 2026-11-22 | Deadband map d0·tanh(out/0.3), d0 0 → 0.10 → 0 at 35/50/70/90 km/h | 50–70 km/h: slow error −15%, small-correction gain +0.09, torque rate +12%. Motorway unchanged | f4b77e807 |
| 2026-11-27 | Ki 0.45 → 0.6 | Tracking error −2 to −4% at every speed, torque rate +1–4%. The map is confirmed: removing it gives +15% error at 50–70 km/h | this commit |

Also tested on 2026-11-22 and not shipped:
- **Removing the learned latAccelOffset:** worse at every speed.
- **Friction 0.10:** small gain.

Also tested on 2026-11-27 and not shipped:
- **Map 0.075:** slightly worse than 0.10 everywhere.
- **Ki 0.75:** −4 to −7% error for +3–9% torque rate. Try it after clean drives on 0.6.
- **Kp 1.0:** −3 to −4% for +3–5%, a worse trade than Ki.
- **Map 0.10 held to 90 and faded by 110:** −11 to −14% at 70–110 km/h for +10–17% torque rate.
  That is a lower bound, in the band that had the 1.3 Hz limit cycle, and the motorway offset there
  is mostly the model's choice of line.

## Limits

- **Jitter at high torque:** the sim does not show the 1.3 Hz motorway limit cycle. An earlier
  LS600h sim under-predicted the real torque-jitter rise from a friction change by 3–4 times, so
  treat a predicted jitter rise as a lower bound.
- **Lane position:** the setpoint is open loop, so the sim can't predict lane position. Slow
  tracking error is the drift proxy.
- **When to refit:** refit the plant after any HRR firmware, calibration or EPS change.
