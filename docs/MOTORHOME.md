# Motorhome cruise engagement and optional SSC

The motorhome controls steering only. Factory cruise controls vehicle speed;
openpilot sends no throttle, brake, cruise-button or Mobileye/Jake Brake commands.

## Test engagement before installing the SSC

Select the Motorhome platform in Vehicle Settings. Leave **Motorhome SSC Steering
Output** OFF (the default), then restart the device. The interface reads the
startup setting once and publishes the same choice to the controller and Panda.
Changing the stored setting does not enable or disable a running actuator.

For independent steering, enable **Always On Lateral** and **Enable With Cruise
Control** in Lateral Settings. The usual calibration, speed, driver monitoring
and fault checks still apply. The brake-pause speed setting also still applies.

| Driver action | Steering permission with independent steering enabled | Factory cruise |
|---|---|---|
| Cruise ON | Available, subject to normal checks | Not yet engaged |
| SET | Remains available | Engages |
| CANCEL or brake-induced cruise dropout | Remains available | Disengages |
| RESUME | Remains available | May re-engage according to factory rules |
| Cruise OFF | Disabled | Factory rules apply |

With SSC output OFF:

- There is no SSC feedback requirement or missing-SSC steering fault.
- The controller sends **no actuator CAN messages**, including OFF messages.
- Panda independently rejects every steering command, even if stock cruise is
  engaged or an unexpected sender tries to transmit one.
- `carControl.latActive`/the independent steering display describe the requested
  engagement state, not actual motor operation. Reported actuator output is zero.

This allows testing the cruise state transitions, UI and logs without the SSC
connected. Other required vehicle CAN messages must still be present. Physical
CANCEL-button behavior should be checked explicitly: the recorded road tests
show brake-induced cruise dropouts, rather than an isolated CANCEL press.

## Signals and shutdown handling

All coach signals below are received on bus 0:

- `0x18FEF117`, SPN 596: cruise master switch.
- `0x18FEF100`, SPN 595: actual cruise engagement; the same source supplies speed
  and cruise set speed.
- `0x18FEF117` and `0x18FEF121`, SPN 597: brake switches.
- `0x18FEF117`, SPNs 599–602: SET, COAST, RESUME and ACCEL switch states.
- `0x18F00503`, SPNs 524/523: selected and current transmission gear. Steering
  requires both to be forward gears; selecting Reverse or Neutral blocks output
  before a shift completes. Park, invalid gear and missing messages also block it.

The J1939 DBC uses byte 5 (DBC bit 32) for the four two-bit switches and byte 6
(bit 40) for set speed. The old interpretation of `0x55` as an 85 km/h set speed
was incorrect. End-of-session master frames with all four switches asserted are
rejected as steering permission, as are error/unavailable switch states.

Panda keeps `controls_allowed` tied to stock cruise for the existing heartbeat
and controls-mismatch checks. Independent steering is a separate permission and
requires the Always On Lateral alternative-experience flag and a valid master.
Master OFF, invalid input, stale CAN, steering faults and driver-monitoring
disable events stop steering permission. The motorhome never bypasses those
faults using the optional developer override.

## Start commissioning the SSC

**Motorhome SSC Steering Output** is visible for the Motorhome selection in
Vehicle Settings, can be changed only offroad, and requires a device restart.
Enable it only after the actuator and its feedback have been checked on the bench.
Panda must run firmware containing the updated motorhome safety mode; its safety
parameter bit 0 enables SSC output and monitoring. Leaving that bit clear is the
engagement-only configuration.

The initial interface uses the existing SSC/i30 wire protocol on isolated bus 1:

- `0x22E`, 5 bytes: torque-mode steering commands at the 100 Hz control rate.
- `0x22F`, 8 bytes: status, monitored at a minimum expected rate of 20 Hz.
- Commands use the existing SSC helper's counter/checksum convention. Status
  uses a four-bit counter advancing modulo 16. Its checksum covers bytes 1–6
  plus `0x22F`, excluding checksum byte 0 and final `DEBUG_STATES` byte.
- Status `CONTROL_STATUS` bit 1 must report feedback enabled; bit 2 (soft-off)
  blocks steering. No undocumented debug bits are interpreted as fault codes.

The firmware must report calibrated **steering-wheel** angle in degrees, rate in
revolutions/second and torque in the protocol's Nm units. The motorhome does not
apply the i30's pulley conversion. Verify the angle/torque signs, scaling, center
and mechanical coupling for this installation before applying steering torque.

The commissioning controller limits torque to 4 Nm, ramps magnitude up at
5 Nm/s and down at 10 Nm/s, and reaches zero before reversing direction. It uses
`latActive`, so cruise cancellation does not stop otherwise permitted steering.
Deactivation, faults or invalid requests reset the ramp and send mode OFF with
zero angle and torque. Relative-angle and soft-off command modes are not used.

Panda checks the command address, bus, length, checksum, torque and rate limits.
Active commands require fresh engine/master/brake/transmission messages within 500 ms and
valid SSC feedback within 250 ms. A valid zero-output OFF command remains allowed
during faults. Missing or invalid SSC feedback prevents active output.

These are initial commissioning limits, not a validated motorhome steering tune.
The interface still lacks decoded parking-brake information and a separate driver
steering-override signal. Its driver-torque field remains a placeholder;
mechanical override, actuator watchdog/fault behavior, steering
feedback and vehicle dynamics require installation testing. Automatic Mobileye/
Jake Brake interventions have not been identified separately in the recordings.

Transmission field positions follow the [Eaton J1939 broadcast table](https://www.eaton.com/content/dam/eaton/products/transmissions/vehicle-transmissions/installation-guides/eaton-autoshift-ultrashift-transmission-installation-guide-trig0930-en-us.pdf)
and were checked against the coach's recorded forward, reverse and neutral frames.

## Focused verification

Use the repository Docker workflow from `AGENTS.md`; build only these targets in
the disposable checkout as needed:

```sh
scons -j4 opendbc_python common/params_pyx.so msgq_repo/msgq/ipc_pyx.so common/transformations/transformations.so
(cd panda && scons -j4 tests/libpanda/libpanda.so)
PYTHONPATH="$PWD" python -m pytest -q -c /dev/null --confcutdir=selfdrive/car/motorhome/tests selfdrive/car/motorhome/tests
PYTHONPATH="$PWD" python -m pytest -q -c /dev/null --confcutdir=panda/tests/safety panda/tests/safety/test_motorhome.py
```

The checks include recorded SET/RESUME byte fixtures, default-off output
isolation, ON/SET/CANCEL/RESUME/OFF transitions, manual pedals, driver-monitoring
faults, missing CAN, bad SSC checksums/counters and actual generated torque ramps
through the native Panda safety hooks. Source recordings and the host checkout
are mounted read-only; compilation happens only in the disposable copy.
