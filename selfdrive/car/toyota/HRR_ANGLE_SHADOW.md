# LS600h angle shadow (ghost), version 1

This is an experimental observer for the existing HRR firmware. It runs beside
the current steering-angle selection and publishes diagnostics in
`frogpilotCarState.ls600hAngleShadow` at the normal CarState publication rate.

**It does not change steering feedback, torque, faults, engagement, controller
trim, learned parameters or firmware. The existing steering defects therefore
remain present in live control.** Observer exceptions disable only the observer
and mark `observerError`; they do not interrupt the live CarState path.

The version identifies the candidate policy. Version 1 hypothesizes:

- five fresh, consistent HRR samples before qualification;
- 50 ms freshness, including progressing HRR counter (normal counter wrap allowed);
- a 3-degree HRR/OEM differential consistency gate;
- up to 30 ms of holding during fresh wrap ambiguity;
- an OEM-delta bridge in the last trusted HRR reference for up to 250 ms;
- hypothetical unavailability if no qualified estimate remains;
- a hypothetical fault latch during lateral activity, cleared only after inactive,
  qualified recovery.

These thresholds are for measurement and comparison, not approved control limits.
The prototype cannot establish an absolutely correct angle zero from two sources
that are both wrong. Its startup qualification tests relative consistency.
Changes to the policy require a version change so results remain interpretable.

## What to measure

Log `liveAngleDeg` versus `angleDeg`, with `usable`, `mode`, `reason`, ages,
raw OEM/HRR values, reference difference, innovation, counter, flags and the
actual lateral-active input. `wouldDisableLateral` is a ghost decision only.
An unavailable candidate angle must not be interpreted as valid feedback.

Review every source transition and rejection, not just averages. Use calibrated
yaw × speed as a separate motion check, and video for the actual lane position.
Check false rejection during genuine rapid steering, counter restarts, freshness
gaps and recovery. The LS600h driver-steering flag is forced false, so it cannot
establish whether the driver intervened.

No controller subscribes to this diagnostic. In particular it does not freeze the
real integrator or parameter learner when the ghost would reject feedback.
Adding those protections belongs to a separately reviewed active implementation.

## Recorded-log replay

Run in the repository Docker environment after building `opendbc_python`:

```sh
PYTHONPATH=/tmp/openpilot python tools/tuning/replay_hrr_angle_shadow.py \
  /logs/route--2/rlog.bz2 /logs/route--3/rlog.bz2 --output /tmp/angle-shadow.csv
```

Use files from one route, in order. The tool never sends CAN or writes device
parameters. A gap in supplied segments resets its observer. It replays at logged
CarState publication times using the available CAN and carControl publications;
one-cycle ordering can differ from the live observer. Its OEM bridge rate is
unspecified (zero), so use it to examine angle/quality decisions, not rate fidelity.
Live shadow records use the current OEM rate filter.

For a drive recorded with this observer, add `--recorded-shadow` to export its
actual diagnostic publications instead of recomputing the hypothesis offline.
Old logs contain no shadow publications and yield a zero-sample summary in that
mode. Counter qualification accounts for multiple HRR frames in a normal CAN
batch; a skipped/reset sequence requires requalification.

Neither offline replay nor live shadow measures a counterfactual vehicle path.
The actual vehicle still responds to the old controller. Corrected feedback would
change future torque, motion, model inputs and subsequent requests; predicting
that requires additional closed-loop validation.

## Checks before any activation

The focused tests cover reference-preserving fallback, valid sector jumps,
timeout/latching, qualification, repeated/reset/wrapped counters, nonfinite or
missing inputs, diagnostic serialization and isolation from live CarState even
when the observer fails. Exercise the same observer on known recorded failures.
Future activation needs its own tests of actuation, warning timing, takeover,
learning gates and re-engagement. There is deliberately no activation switch in
this change.
