# debug scripts

## [can_printer.py](can_printer.py)

```
usage: can_printer.py [-h] [--bus BUS] [--max_msg MAX_MSG] [--addr ADDR]

simple CAN data viewer

optional arguments:
  -h, --help         show this help message and exit
  --bus BUS          CAN bus to print out (default: 0)
  --max_msg MAX_MSG  max addr (default: None)
  --addr ADDR
```

## [hrr_can_test.py](hrr_can_test.py)

Interactive Panda test tool for the STM32G474 HRR CAN controller. It sends the brake (`0x2C6`)
and torque (`0x160`) frames continuously at `100 Hz` by default. Run it with the vehicle safely
secured and no one near the steering mechanism. The script uses `SAFETY_ALLOUTPUT` while running
and restores `SAFETY_SILENT` during shutdown.

Connect a Panda, choose the CAN bus with `--bus`, and start the tool from the openpilot checkout:

```bash
python3 selfdrive/debug/hrr_can_test.py --bus 2
```

If `--bus` is omitted, the script prompts for bus `0`, `1`, or `2`. The initial state is
disengaged with zero torque and the brake released. At the `hrr>` prompt:

- `e` engages by enabling both `REL` and `RELE`
- `x` disengages, sets torque to zero, and disables both relays
- `<Ncm>` sets torque directly in the range `-1000..1000` Ncm
- `d <samples>` sets and persists SVEC DLY in the range `0..127`; subsequent `s` output shows
  that value for the current session
- `a <tenths-deg>` sets and persists SVEC `ANGLE_OFFSET` in tenths of a degree; for example,
  `a -45` sets `-4.5 deg`
- `b 0` or `b 1` sets `BRAKE_PRESSED` to released or pressed
- `r 1` forces the Panda harness relay and disables firmware forwarding; `r 0` restores both
- `s` shows the current state
- `h` shows command help
- `q` safely shuts down and exits

The `RX` line above the prompt monitors HRR status frames on the selected bus. `device=ONLINE`
means the script has received an HRR status frame within the last `0.5 s`; `WAITING` changes to
`OFFLINE` if none arrive. It also displays the reported relay states and angle/torque feedback:

```text
RX device=ONLINE age=0.012s bus=2 REL=ON RELE=ON
   SVEC_Delta=+1.0deg Emulated_Torque=+250Ncm
   OU_Angle=92.4deg IN_Angle=91.4deg
```

`REL` and `RELE` come from `CANCTR_IOStatus` (`0x631`). `SVEC_Delta`, `Emulated_Torque`,
`OU_Angle`, and `IN_Angle` come from `HRR_AngleStatus` (`0x632`). Values are shown as `---`
until their first valid-length status frame is received. The `TX` line separately labels the
requested states as `REL_Cmd` and `RELE_Cmd`. The HRR does not report its persisted DLY or
`ANGLE_OFFSET` values in any current status frame, so the `TX` line shows each as `unknown (not
reported)` until this tool sends the corresponding command; afterward it shows the value sent
during the current session.

For example:

```text
hrr> e
hrr> 250
hrr> b 1
hrr> x
hrr> q
```

Use `--dry-run` to exercise the prompt without opening a Panda or transmitting CAN frames, and
`--self-test` to verify the known frame encodings and exit:

```bash
python3 selfdrive/debug/hrr_can_test.py --bus 2 --dry-run
python3 selfdrive/debug/hrr_can_test.py --self-test
```

The streaming rate can be changed with `--rate-hz`:

```bash
python3 selfdrive/debug/hrr_can_test.py --bus 2 --rate-hz 50
```

Use `--force-harness-relay` to start with the Panda harness relay forced into intercept mode. Like
the UI's **Force Harness Relay On** option, this keeps CAN0 and CAN2 physically separated and
disables Panda firmware forwarding while active:

```bash
python3 selfdrive/debug/hrr_can_test.py --bus 2 --force-harness-relay
```

Pressing Ctrl-C, sending EOF, or entering `q` performs a safe shutdown: zero torque, relays off,
`BRAKE_PRESSED=1`, Panda safety set to `SAFETY_SILENT`, and any harness-relay force applied by the
script cleared with firmware forwarding restored.

## [hrr_angle_calibrate.py](hrr_angle_calibrate.py)

Guided electrical and steering-reference calibration for the HRR `IN_Angle` and `OU_Angle`
resolver estimators. The vehicle must be stationary and secured, with the brake held, HRR relays
open, and torque output interlocked. The tool uses the validated coarse `STEER_ANGLE` from
LS600h `0x25` (1.5-degree resolution), pairs that reference with the HRR's per-window signed RMS/covariance resolver
vectors on `0x637`, and asks the operator to sweep slowly center -> left lock -> right lock ->
center. Keep the sweep below approximately 30 degrees/second so consecutive 10 Hz projective
resolver vectors cannot cross the ambiguous 90-degree modulo-180 half-period.

```bash
python3 selfdrive/debug/hrr_angle_calibrate.py --bus 2
```

`--bus 2` is the HRR command/status bus. Panda reports received frames with their original source
bus, so the script reads `0x025` from vehicle buses 0 and 1 by default rather than expecting it
to appear as a receive frame on bus 2. Override that selection when needed, for example:

```bash
python3 selfdrive/debug/hrr_angle_calibrate.py --bus 2 --reference-bus 0
```

Press Enter once the display reports `READY`. The script robustly fits the resolver/steering ratio
and independent 2x2 IN/OU gain, skew, and phase-correction matrices before `atan2()`. Readiness
requires both sweep directions, at least 100 accepted samples, broad steering and resolver-phase
coverage, and no more than 1.5 degrees RMS or 5.0 degrees maximum steering-equivalent residual.
The LS600h resolver completes one 360-degree electrical revolution per 22.5 degrees of shaft
rotation, so the fitted signed phase-per-steer magnitude must be near 16. The RMS/covariance
vectors are projective modulo 180 degrees; continuous unwrapping retains the 16:1 relationship.
The firmware stages the fit under a calibration session and commits the complete coefficient set
to an A/B flash snapshot only after validating it. A failed or interrupted recalibration leaves
the previous committed calibration intact. With no valid enabled calibration, the original raw
modulo-180 estimator remains active.

The script reads the steering reference directly from its vehicle-side receive bus and does **not**
bridge CAN0 and CAN2. To satisfy the current HRR firmware's local safety protocol, after the
operator confirms the prompt it transmits the fixed pressed-brake `0x2C6` frame directly on HRR
bus 2 at 100 Hz for the duration of the calibration session. This is a synthetic HRR-local
interlock input, not a measurement of the physical pedal; the operator must still secure the
vehicle and hold the brake. Transmit stops on exit and the firmware's brake freshness timeout
then returns the HRR to its safe state.

The stored mode can be selected explicitly without repeating the sweep:

```bash
python3 selfdrive/debug/hrr_angle_calibrate.py --bus 2 --legacy
python3 selfdrive/debug/hrr_angle_calibrate.py --bus 2 --calibrated
```

`--calibrated` is rejected when no valid snapshot exists, and both mode commands wait for firmware
confirmation. Ctrl-C aborts the temporary session, preserves the previous calibration, restores
Panda to `SAFETY_SILENT`, and exits nonzero. Protocol-only checks do not require a Panda:

```bash
python3 selfdrive/debug/hrr_angle_calibrate.py --self-test
python3 selfdrive/debug/hrr_angle_calibrate.py --bus 2 --dry-run --yes
```

## [hrr_can_test_logged.py](hrr_can_test_logged.py)

Logged version of `hrr_can_test.py` with the same HRR controls and safe-shutdown behavior. In
addition to running the interactive test, it creates a standard openpilot route containing:

- `fcamera.hevc` for the road-camera video
- `rlog` with full-rate Panda RX frames published as `can` and script TX frames published as
  `sendcan`
- `qlog` with the normal service decimation, including only a subset of the CAN traffic
- `qcamera.ts` and any other camera files enabled by the existing logger parameters

Run it on a comma device from the openpilot checkout:

```bash
python3 selfdrive/debug/hrr_can_test_logged.py --bus 2
```

The tool starts `loggerd`, `encoderd`, and `camerad` when they are not already running. It reuses
existing instances and stops only the processes it started. Before showing the interactive prompt,
it waits for `loggerd` to subscribe to `can` and `sendcan`, creates or identifies the current
route, and verifies that the active segment's `fcamera.hevc` is growing. The route name and segment
path are printed at startup and again during shutdown.

Like `hrr_can_test.py`, this script opens Panda directly. Stop `pandad` and any other process that
owns Panda before running it. The usual options and prompt commands are supported, including:

```bash
python3 selfdrive/debug/hrr_can_test_logged.py --bus 2 --force-harness-relay
python3 selfdrive/debug/hrr_can_test_logged.py --bus 2 --rate-hz 50
python3 selfdrive/debug/hrr_can_test_logged.py --self-test
```

`--dry-run` still starts camera and route logging, but it does not open Panda or publish CAN
traffic.

The brake frame `0x2C6` is generated by the script, not copied from an incoming bus. Each streaming
cycle builds either the released payload `99 20 84` or pressed payload `9B 20 86` and transmits it
on the selected bus immediately before torque frame `0x160`. Panda is placed in
`SAFETY_ALLOUTPUT` with parameter `0`, so its CAN0-to-CAN2 software-forwarding hook is inactive.
Using `--force-harness-relay` or entering `r 1` also explicitly disables firmware forwarding and
keeps CAN0 and CAN2 physically separated.

The same transmitted frame can appear in the route twice without having been forwarded:

- `sendcan` with `src=<bus>` is the script's transmit request
- `can` with `src=<bus>+128` is Panda's returned transmit confirmation

Use the `rlog` for synchronized, full-rate CAN and `fcamera.hevc` analysis. The `qlog` is useful for
quick inspection but intentionally contains decimated `can` and `sendcan` messages. Files may
initially be named `rlog` and `qlog`; the uploader can later compress them to `rlog.bz2` and
`qlog.bz2`.

## [send_canctr_delta.py](send_canctr_delta.py)

Sends guarded CANCTR delta-control command `0x231` directly through Panda on bus 1. The script does
not use `sendcan` or `pandad`; it opens `Panda()`, forces `SAFETY_ALLOUTPUT`, sends the frame with
`panda.can_send(...)`, and restores `SAFETY_SILENT` on exit. By default it keeps resending the last
command every `100 ms` so the STM32 CANCTR watchdog stays fed; the script increments the 4-bit `Delta_Counter` on every transmit. Use `--once` for a single frame.

The command format is:
- CAN ID `0x231`
- payload `<hhBB>`
- bytes `0..1`: `DAC_Delta`
- bytes `2..3`: `DAC_Delta_Redundant`
- byte `4`: flags, bit0=`REL`, bit1=`RELE`, bits4..7=`Delta_Counter`
- byte `5`: checksum = `low8(id_lo + id_hi + bytes0..4)`

The current script takes a command value on `--voltage`.
In the default mode it converts that value as:
- `applied_voltage = -input/1000`
- then converts that voltage into DAC delta counts
- duplicates the delta into the redundant field
- computes the checksum byte

With `--TQ`, the same `--voltage` field and interactive values are interpreted as torque instead:
- `100 torque = 165` legacy input units = `0.165 V`
- the script first maps torque into the legacy input scale, then applies the usual `applied_voltage = -input/1000`

Typical use:

```bash
python3 send_canctr_delta.py --bus 1 --rel 1 --rele 1 --voltage 0
```

That sends zero delta and turns both relay bits on, then keeps resending that same command every `100 ms` until you stop the script.

```bash
python3 send_canctr_delta.py --bus 1 --rel 1 --rele 1 --voltage 1000
```

That applies `-1.0 V` relative command, because the script uses `applied_voltage = -input/1000`.

```bash
python3 send_canctr_delta.py --bus 1 --rel 0 --rele 0 --voltage -500
```

That applies `+0.5 V` relative command with both relay bits off.

Interactive mode:

The current command is sent immediately on startup and then resent every `100 ms` in the background.
`rel`, `rele`, `flags`, and value updates all transmit immediately and then become the new keepalive state.
Use `err` or `errors` at the interactive prompt to print the latest decoded `0x241 CANCTR_IOStatus`, including `DELTA_DLC_ERR`, `DELTA_MISMATCH`, `DELTA_CHECKSUM_ERR`, `DELTA_COUNTER_ERR`, `WATCHDOG_TIMEOUT`, `SNR_PAIR_ERR`, `EPS_PAIR_ERR`, and `OUTPUT_RANGE_ERR` when present.

```bash
python3 send_canctr_delta.py --interactive --bus 1 --rel 1 --rele 1 --voltage 0
```

Torque-input mode:

```bash
python3 send_canctr_delta.py --TQ --bus 1 --rel 1 --rele 1 --voltage 606.06
```

That sends about the same command as legacy input `1000`, so it applies about `-1.0 V` relative command.
With the new scaling, `100` torque corresponds to `165` legacy input, or `-0.165 V`.

```bash
python3 send_canctr_delta.py --interactive --TQ --bus 1 --rel 1 --rele 1 --voltage 0
```

Then at the `input>` prompt:
- press Enter to resend the current command
- type `1000` to send `-1.0 V`
- type `-500` to send `+0.5 V`

In `--TQ` mode the prompt changes to `tq>`:
- type `606.06` to send about `-1.0 V`
- type `303.03` to send about `-0.5 V`
- type `100` to send `-0.165 V`
- type `rel 0` or `rele 0` to change the stored relay bits
- type `flags 1 0` to set both relay bits together
- type `show` to print the current state
- type `q` to exit

```
usage: send_canctr_delta.py [-h] [--bus BUS] [--rel REL] [--rele RELE] [--voltage VOLTAGE] [--TQ] [--interactive] [--no-interactive] [--period-ms PERIOD_MS] [--once] [--monitor] [--no-monitor]
```

## [dump.py](dump.py)

```
usage: dump.py [-h] [--pipe] [--raw] [--json] [--dump-json] [--no-print] [--addr ADDR] [--values VALUES] [socket [socket ...]]

Dump communication sockets. See cereal/services.py for a complete list of available sockets.

positional arguments:
  socket           socket names to dump. defaults to all services defined in cereal

optional arguments:
  -h, --help       show this help message and exit
  --pipe
  --raw
  --json
  --dump-json
  --no-print
  --addr ADDR
  --values VALUES  values to monitor (instead of entire event)
```

## [vw_mqb_config.py](vw_mqb_config.py)

```
usage: vw_mqb_config.py [-h] [--debug] {enable,show,disable}

Shows Volkswagen EPS software and coding info, and enables or disables Heading Control
Assist (Lane Assist). Useful for enabling HCA on cars without factory Lane Assist that want
to use openpilot integrated at the CAN gateway (J533).

positional arguments:
  {enable,show,disable}
                        show or modify current EPS HCA config

optional arguments:
  -h, --help            show this help message and exit
  --debug               enable ISO-TP/UDS stack debugging output

This tool is meant to run directly on a vehicle-installed comma three, with
the openpilot/tmux processes stopped. It should also work on a separate PC with a USB-
attached comma panda. Vehicle ignition must be on. Recommend engine not be running when
making changes. Must turn ignition off and on again for any changes to take effect.
```
