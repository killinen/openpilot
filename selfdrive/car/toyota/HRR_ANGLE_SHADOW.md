# LS600h angle correction, version 2

**This is a fix for HRR firmware v0.1 and must be revised on HRR v0.2.** The
offset, sector and margins describe v0.1 behavior and its current calibration.

This observer runs on the existing HRR firmware and publishes diagnostics in
`frogpilotCarState.ls600hAngleShadow` at the normal CarState publication rate.

**CarState uses its angle and rate for `steeringAngleDeg` and `steeringRateDeg`
whenever it is usable.** When it is not usable, or after an observer exception,
CarState uses the previous HRR/OEM selection, logged as `liveAngleDeg`. It does
not change torque limits, faults, engagement, firmware or alerts. Being unusable
never disables lateral, and `wouldDisableLateral` remains a diagnostic only.

Learned parameters now see the corrected angle. The sector correction and bridge
have limited closed-loop evidence, so review the first engaged drives.

## The failure it targets

HRR firmware (`TrueAngle_Update`) anchors on raw OEM `0x25` and cannot currently
be updated or recalibrated. The resolver repeats every 22.5 degrees of shaft
rotation, and on every fresh reference the firmware picks the sector nearest raw
OEM. It invalidates, setting `Wrap_Ambiguous`, when OEM and HRR differ by more
than 10 degrees. Measured HRR minus OEM is about -7.2 degrees near center, so a
correct HRR has only about 2.8 degrees of margin on that side.

Logs match this. Invalidation clusters at a 10-degree OEM/HRR difference. HRR
then either returns to the right sector, or once the difference passes 12.5
degrees it re-anchors one sector high and reports valid. The wrong sector holds
until OEM returns, because continuity tracking keeps it inside the same window.

Live control therefore sees two defects: raw OEM during the invalid gap, a
10-degree step each way, and a valid-flagged angle about 22 degrees wrong.

## Version 2 policy

The version identifies the candidate policy. Version 2 hypothesizes:

- a sector chosen only after firmware invalidation, a counter gap, an unexplained
  jump or startup, from a fixed -7.2 degree offset rather than a learned anchor;
- `sectorCorrectedHrr`: HRR minus the chosen sector, which keeps resolver
  resolution during OEM excursions that would mislead any OEM-based estimate;
- no OEM consistency gate once a sector is chosen, since firmware already bounds
  the difference and OEM is the sensor that misbehaves in these events;
- `alignedOem` across gaps, using OEM plus the last good local offset, for up to
  2.5 seconds (the longest engaged gap in the v1 logs was 1.91 seconds);
- five fresh samples before returning to HRR;
- hypothetical unavailability only while no estimate exists, with no latch.

Version 1 is not usable. Its anchor followed HRR per sample, so fast low-speed
steering could move it far enough to reject healthy HRR for the rest of a drive,
and any gap over 250 ms latched until disengagement.

These thresholds are for measurement and comparison, not approved control limits.
Changes to the policy require a version change so results remain interpretable.

## Limits

- **Envelope:** the -7.2 offset is only supported within about 60 degrees of
  center. HRR minus OEM varies by many degrees at larger angles, so outside it no
  sector is applied and `reason` is `outsideEnvelope`. HRR may be wrong there.
- **Tuning data:** the thresholds were chosen from the same logs used to score
  them. Drives recorded with version 2 are the first independent test.
- **Calibration:** the offset is a property of the current HRR calibration and
  must be remeasured if that changes.
- **OEM excursions:** why OEM reads up to about 21 degrees from corrected HRR is
  unexplained. Yaw agrees with corrected HRR in the events scored so far.

## What to measure

Log `liveAngleDeg` versus `angleDeg`, with `usable`, `mode`, `reason`, ages,
raw OEM/HRR values, `sectorOffsetDeg`, reference offset, innovation, counter,
flags and the actual lateral-active input. `wouldDisableLateral` is a ghost
decision only. An unavailable candidate angle must not be interpreted as valid
feedback.

Review every source transition and rejection, not just averages. Use calibrated
yaw x speed as a separate motion check, and video for the actual lane position.
Check that every nonzero sector agrees with yaw, because a wrong sector choice is
the new failure this policy could introduce. The LS600h driver-steering flag is
forced false, so it cannot establish whether the driver intervened.

Only CarState reads this result. It does not freeze the real integrator or
parameter learner while the angle is unusable. Adding that protection needs its
own review.

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

Offline replay does not measure a closed-loop vehicle path. Logs recorded before
this change show the vehicle responding to the old angle. In logs recorded after
it, `carState.steeringAngleDeg` is the corrected angle, so compare against
`liveAngleDeg` from `--recorded-shadow`, not the replay's `live_angle_deg` column.

## Checks

The focused tests cover reference-preserving fallback, sector correction and
reselection, OEM excursions, offset walks, the envelope, bridge expiry without a
latch, qualification, repeated/reset/wrapped counters, nonfinite or missing
inputs, diagnostic serialization, CarState source selection and the fallback to
the previous path when the observer is unusable or fails.

Disabling lateral when the angle is unusable would need its own tests of warning
timing, takeover and re-engagement.
