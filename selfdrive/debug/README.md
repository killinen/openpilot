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
