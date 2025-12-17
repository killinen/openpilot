#!/usr/bin/env python3
import argparse
import os
import sys
from collections import Counter

from panda.tests.libpanda import libpanda_py
from panda.tests.safety_replay.helpers import package_can_msg, init_segment

# replay a drive to check for safety violations
def replay_drive(lr, safety_mode, param, alternative_experience, segment=False):
  safety = libpanda_py.libpanda

  err = safety.set_safety_hooks(safety_mode, param)
  assert err == 0, "invalid safety mode: %d" % safety_mode
  safety.set_alternative_experience(alternative_experience)

  if segment:
    init_segment(safety, lr, safety_mode, param)
    lr.reset()

  rx_tot, rx_invalid, tx_tot, tx_blocked, tx_controls, tx_controls_blocked = 0, 0, 0, 0, 0, 0
  safety_tick_rx_invalid = False
  blocked_addrs = Counter()
  invalid_addrs = set()

  can_msgs = [m for m in lr if m.which() in ('can', 'sendcan')]
  # Keep timestamps monotonic to avoid false lagging in safety_tick on out-of-order logs.
  can_msgs.sort(key=lambda m: m.logMonoTime)
  start_t = can_msgs[0].logMonoTime
  end_t = can_msgs[-1].logMonoTime
  for msg in can_msgs:
    safety.set_timer((msg.logMonoTime // 1000) % 0xFFFFFFFF)

    # skip start and end of route, warm up/down period
    if msg.logMonoTime - start_t > 1e9 and end_t - msg.logMonoTime > 1e9:
      safety.safety_tick_current_safety_config()
      safety_tick_rx_invalid |= not safety.safety_config_valid() or safety_tick_rx_invalid

    if msg.which() == 'sendcan':
      for canmsg in msg.sendcan:
        to_send = package_can_msg(canmsg)
        sent = safety.safety_tx_hook(to_send)
        if not sent:
          tx_blocked += 1
          tx_controls_blocked += safety.get_controls_allowed()
          blocked_addrs[canmsg.address] += 1

          if "DEBUG" in os.environ:
            print("blocked bus %d msg %d at %f" % (canmsg.src, canmsg.address, (msg.logMonoTime - start_t) / 1e9))
        tx_controls += safety.get_controls_allowed()
        tx_tot += 1
    elif msg.which() == 'can':
      # ignore msgs we sent
      for canmsg in filter(lambda m: m.src < 128, msg.can):
        to_push = package_can_msg(canmsg)
        recv = safety.safety_rx_hook(to_push)
        if not recv:
          rx_invalid += 1
          invalid_addrs.add(canmsg.address)
        rx_tot += 1

  print("\nRX")
  print("total rx msgs:", rx_tot)
  print("invalid rx msgs:", rx_invalid)
  print("safety tick rx invalid:", safety_tick_rx_invalid)
  print("invalid addrs:", invalid_addrs)
  print("\nTX")
  print("total openpilot msgs:", tx_tot)
  print("total msgs with controls allowed:", tx_controls)
  print("blocked msgs:", tx_blocked)
  print("blocked with controls allowed:", tx_controls_blocked)
  print("blocked addrs:", blocked_addrs)

  return tx_controls_blocked == 0 and rx_invalid == 0 and not safety_tick_rx_invalid

if __name__ == "__main__":
  from openpilot.tools.lib.logreader import LogReader

  parser = argparse.ArgumentParser(description="Replay CAN messages from a route or segment through a safety mode",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("route_or_segment_name", nargs='+')
  parser.add_argument("--mode", type=int, help="Override the safety mode from the log")
  parser.add_argument("--param", type=int, help="Override the safety param from the log")
  parser.add_argument("--alternative-experience", type=int, help="Override the alternative experience from the log")
  args = parser.parse_args()

  override_mode = args.mode
  override_param = args.param
  override_alternative_experience = args.alternative_experience

  all_ok = True
  for identifier in args.route_or_segment_name:
    lr = LogReader(identifier)
    safety_mode = override_mode
    safety_param = override_param
    alternative_experience = override_alternative_experience

    if None in (safety_mode, safety_param, alternative_experience):
      for msg in lr:
        if msg.which() == 'carParams':
          safety_cfg = None
          try:
            safety_cfgs = msg.carParams.safetyConfigs
            n = len(safety_cfgs)
            if n > 0:
              safety_cfg = safety_cfgs[n - 1]
          except Exception:
            safety_cfg = None

          if safety_cfg is not None:
            if safety_mode is None:
              safety_mode = safety_cfg.safetyModel.raw
            if safety_param is None:
              safety_param = safety_cfg.safetyParam
          else:
            if safety_mode is None:
              safety_mode = msg.carParams.safetyModel.raw
            if safety_param is None:
              safety_param = msg.carParams.safetyParam
          if alternative_experience is None:
            alternative_experience = msg.carParams.alternativeExperience
          break
      else:
        raise Exception(f"carParams not found in log {identifier}. Set safety mode and param manually.")

      lr.reset()

    assert safety_mode is not None
    assert safety_param is not None
    assert alternative_experience is not None

    print(f"replaying {identifier} with safety mode {safety_mode}, param {safety_param}, alternative experience {alternative_experience}")
    ok = replay_drive(lr, safety_mode, safety_param, alternative_experience, segment=len(lr.logreader_identifiers) == 1)
    all_ok &= ok

  sys.exit(0 if all_ok else 1)
