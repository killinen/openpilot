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
