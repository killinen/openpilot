# TRQI steering authority: 12 Nm window

Drive `00000602--b4a8333768` showed the host's 10 Nm torque ceiling, rather
than a TRQI DAC-voltage clamp. At controller elapsed 126.94–129.48 s and
approximately 48 km/h, the command held near -9.97 Nm, the EPS reported
approximately -10.7 Nm (peak magnitude 11.2), and further windup was blocked.
The controller saturation indicator was active for about 1.95 s. A second
example at 858–860 s hit the measured-output guard with only about -8.5 Nm
commanded and an EPS estimate reaching -10.6 Nm. Neither example had a
TRQI-owned limiter flag. Some other events include driver takeover and must
not be interpreted as unassisted ECU torque measurements.

The configured torque authority is increased 20%, from +/-10 to +/-12 Nm:

- `TrqiSteerLimitParams.MAX_STEERING_TQ` is 1200 Ncm.
- `STEER_MAX` stays at 1000 Ncm per nominal steering unit. Thus 0.3 still
  commands 3 Nm; 1.2 units provides the additional 12 Nm headroom.
- `CarInterface.get_steer_max` allows 1.2 units only for the TRQI backend.
  Other interfaces and the legacy SSC backend retain 1.0. The lateral PID
  uses this headroom without rescaling its ordinary-range P/I/feedforward.
- The measured EPS-output guard blocks further windup above 12 Nm while
  permitting unwind.
- Panda accepts at most 1201 Ncm, preserving its existing 1 Ncm boundary
  tolerance; firmware 0.2.3 clips requested output to 1200 Ncm.

The nominal host slew stays at 6 Nm/s. Kp/Ki, learned torque factors, driver
passthrough, CRC/counter/complement checks, firmware torque-step/sign-flip
checks, watchdogs and DAC-voltage bounds retain their previous values.

Deploy host and Panda code together, with TRQI 0.2.3. Merely changing the host
threshold would leave the controller, Panda or board at the earlier cap.
Openpilot's normal Panda signature check must activate the rebuilt Panda
firmware. Stage the matching A/B TRQI bundle for its ignition-startup update.
Until all components are updated, the lower remaining limit still applies.

This is commanded torque headroom and an EPS-estimate guard, not an increase
in the EPS hardware rating or a promise of exactly 20% more wheel torque.
The 3 m/s² curvature acceleration limit is separate and is not changed here.

Validation exercises both signs, unchanged ordinary-range conversion and
slew, measured-output windup/unwind, lateral PID headroom and the Panda
transmit boundary with controls/CRC/counter/complement enforcement. Firmware
host tests cover the +/-1200 Ncm clamp, out-of-range flags, monotonic mapping
and the existing DAC-voltage window across the extended request domain.

The focused torque-command Panda tests pass for lateral and longitudinal
modes, including the extended boundary and the existing authentication and
controls checks. The full legacy Hyundai-community module also contains two
0x231 delta-command tests that fail identically on the original source because
that legacy command is not allowed by the transmit list. This change does not
re-enable it.

## Catch-up slew (2026-11-17)

The nominal host slew is still 6 Ncm/frame. When the applied command lags the
request by more than `CATCHUP_GAP` (300 Ncm), windup and unwind may both move
`CATCHUP_DELTA` (15 Ncm/frame). This targets curve entry/exit, where the
request swings about 24 Nm in about 1.2 s while the nominal slew needs about 4 s.
Near-straight driving never opens a 3 Nm gap.

Catch-up is blocked:

- during the first `CATCHUP_ENGAGE_FRAMES` (1 s) of lateral control, so the engage ramp is unchanged;
- while the driver touches the wheel, and for `CATCHUP_HOLDOFF_FRAMES` (1 s) afterwards.
  SNR passthrough active (`steeringPressed`), requested or pending blocks both directions
  at any speed. The firmware-identical `|SNR1-SNR2+95 mV|` at or above 170 mV (about
  0.5 Nm, read from `TRQI_VoltageStatus`) blocks windup. On its own, with no flag in the
  hold-off, it still allows unwind toward zero below 60 km/h
  (`CATCHUP_ANALOG_UNWIND_MAX_SPEED`, added 2026-11-20). That estimate rises about
  0.04 Nm per Nm of EPS output. On tight low-speed turn exits it tripped while openpilot
  held 8-12 Nm, blocked the unwind, and the car kept turning (00000640 at 128 s:
  0.040 1/m against 0.014 requested). Whether the driver's hands were on the wheel then is
  not known from the logs;
- during lane changes (added 2026-11-20). This covers a physical blinker, or the model lane
  change: controlsd sets `CC.leftBlinker` / `CC.rightBlinker` while `laneChangeState` is not
  off, which can outlast the blinker. It also covers `CATCHUP_LANE_CHANGE_HOLDOFF_FRAMES`
  (1 s) afterwards. The gate applies once speed has exceeded 50 km/h during the episode
  (`CATCHUP_LANE_CHANGE_MIN_SPEED`, latched until the hold-off ends). So a blinker at a
  slowly approached junction keeps catch-up, and crossing 50 km/h never switches policy
  mid-manoeuvre.

Windup catch-up also fades with speed (2026-11-20). It is 15 Ncm/frame up to 70 km/h,
then falls linearly to the nominal 6 at 90 km/h (`CATCHUP_WINDUP_FADE_BP`). Unwind keeps 15.

Why: on the 575 km highway drive of 2026-11-20 (0000063e-00000640), catch-up fired in 45
of 46 lane changes. The 90th-percentile lane-change lateral jerk was 5.4 m/s^3 (wheel) and
5.9 (yaw), against 2.9 / 3.2 before catch-up. In a replay of the logged commands, the
physical blinker plus 1 s left fast steps in 4 of the 46; the model state removes those.
The windup fade is a precaution against fast highway torque build-up: it changed 1 of 132
ordinary highway curve entries, by at most 0.18 Nm. The sharp "curve entry" cases first
blamed on catch-up were lane-change tails.

The measured-output guard still blocks windup above 12 Nm. Panda only bounds
0x232 magnitude, and TRQI firmware allows 100 + 40 Ncm per frame, so neither
limit changes. In a closed-loop replay of 1.7 h of drives after 1bc81708a, the
catch-up slew cut ramp-lag episodes by 80-90% and p99.9 tracking error by 40%,
for +1-3% wheel twitch.
Rerun the offline simulator before changing these values; see `LATERAL_SIM.md`.
