#!/usr/bin/env python3
"""Exercise pandad's actual state-publication function with fake Params/Panda.

Run in the AGENTS.md Docker copy after building pandad.o (generated C++ headers).
The source function is extracted unchanged so the check needs no device or IPC.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'selfdrive/pandad/pandad.cc').read_text()
function = source.split('std::optional<bool> send_panda_states(', 1)[1].split('\nvoid send_peripheral_state(', 1)[0]
enum = source.split('enum class IgnitionOverride', 1)[1].split('};', 1)[0]
preamble = r'''
#include <array>
#include <bitset>
#include <cassert>
#include <cstdio>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include "cereal/messaging/messaging.h"
#include "panda/board/health.h"
#include "panda/board/can_definitions.h"

struct Params {
  std::map<std::string, bool> values;
  std::vector<std::string> reads, writes;
  int fail_writes = 0;
  bool getBool(const std::string &key) { reads.push_back(key); return values[key]; }
  int putBool(const std::string &key, bool value) {
    writes.push_back(key);
    if (fail_writes > 0) { --fail_writes; return -1; }
    values[key] = value; return 0;
  }
};
struct Panda {
  cereal::PandaState::PandaType hw_type = cereal::PandaState::PandaType::TRES;
  health_t health{};
  bool fail_health = false;
  int fail_can_health = -1;
  Panda() { health.safety_mode_pkt = (uint8_t)cereal::CarParams::SafetyModel::NO_OUTPUT; }
  std::optional<health_t> get_state() { return fail_health ? std::nullopt : std::make_optional(health); }
  std::optional<can_health_t> get_can_state(int bus) {
    return bus == fail_can_health ? std::nullopt : std::make_optional(can_health_t{});
  }
  void set_safety_model(cereal::CarParams::SafetyModel mode) { health.safety_mode_pkt = (uint8_t)mode; }
  void set_power_saving(bool enabled) { health.power_save_enabled_pkt = enabled; }
  bool comms_healthy() { return true; }
};
struct FakePubMaster {
  int messages = 0;
  void send(const char *, MessageBuilder &) { ++messages; }
};
#define PubMaster FakePubMaster
'''
checks = r'''
#undef PubMaster
struct Fixture {
  Params params;
  Panda panda;
  FakePubMaster pm;
  bool onroad = false;
  bool ready_onroad_cleared = false;
  std::optional<bool> tick() {
    params.reads.clear(); params.writes.clear();
    return send_panda_states(&pm, {&panda}, IgnitionOverride::AUTO, params, onroad, ready_onroad_cleared);
  }
  void poll_twice() { assert(tick()); assert(tick()); }
};
int main() {
  using Mode = cereal::CarParams::SafetyModel;
  // Even persisted/requested leases cannot be granted while ONROAD.
  for (bool engaged : {false, true}) {
    Fixture f;
    f.panda.health.ignition_line_pkt = 1;
    f.panda.health.safety_mode_pkt = (uint8_t)Mode::TRQI_UPDATER;
    f.params.values = {{"IsOnroad", true}, {"IsEngaged", engaged},
      {"TrqiUpdaterRequested", true}, {"TrqiUpdateInProgress", true},
      {"TrqiUpdateStartupHold", true}, {"TrqiUpdaterReady", true}};
    assert(f.tick() == true);
    assert(f.onroad);
    assert(!f.params.values["TrqiUpdaterReady"]);
    assert(f.panda.health.safety_mode_pkt == (uint8_t)Mode::NO_OUTPUT);
    assert((f.params.writes == std::vector<std::string>{"TrqiUpdaterReady"}));
    for (int i = 0; i < 100; ++i) {
      assert(f.tick() == true);
      assert((f.params.reads == std::vector<std::string>{"IsOnroad"}));
      assert(f.params.writes.empty());
    }
    // Returning OFFROAD must resume full lease processing immediately.
    f.params.values["IsOnroad"] = false;
    f.params.values["IsEngaged"] = false;
    f.poll_twice();
    assert(f.params.values["TrqiUpdaterReady"]);
    assert(f.panda.health.safety_mode_pkt == (uint8_t)Mode::TRQI_UPDATER);
    // Entering ONROAD revokes a live lease on the first successful poll.
    f.params.values["IsOnroad"] = true;
    assert(f.tick());
    assert(!f.params.values["TrqiUpdaterReady"]);
    assert(f.panda.health.safety_mode_pkt == (uint8_t)Mode::NO_OUTPUT);
  }
  // Exhaust the existing OFFROAD ignition/hold/request/recovery/engagement gate.
  for (bool ignition : {false, true}) for (bool hold : {false, true})
  for (bool requested : {false, true}) for (bool recovery : {false, true})
  for (bool engaged : {false, true}) {
    Fixture f;
    f.panda.health.ignition_line_pkt = ignition;
    f.params.values = {{"IsEngaged", engaged}, {"TrqiUpdaterRequested", requested},
      {"TrqiUpdateInProgress", recovery}, {"TrqiUpdateStartupHold", hold}};
    f.poll_twice();
    bool expected = !engaged && (recovery || (requested && (!ignition || hold)));
    assert(f.params.values["TrqiUpdaterReady"] == expected);
    assert((f.panda.health.safety_mode_pkt == (uint8_t)Mode::TRQI_UPDATER) == expected);
    if (expected) {
      // Updater release/manager clear must be noticed even if readiness was
      // cleared externally; then regrant while the request remains active.
      f.params.values["TrqiUpdaterReady"] = false;
      assert(f.tick());
      assert(f.params.values["TrqiUpdaterReady"]);
      f.params.values["TrqiUpdaterRequested"] = false;
      f.params.values["TrqiUpdateInProgress"] = false;
      assert(f.tick());
      assert(!f.params.values["TrqiUpdaterReady"]);
      assert(f.panda.health.safety_mode_pkt == (uint8_t)Mode::NO_OUTPUT);
    }
  }
  // A failed readiness clear must retry before taking the fast ONROAD path.
  {
    Fixture f;
    f.params.values = {{"IsOnroad", true}, {"TrqiUpdaterReady", true}};
    f.params.fail_writes = 1;
    assert(f.tick());
    assert(f.params.values["TrqiUpdaterReady"] && !f.ready_onroad_cleared);
    assert(f.tick());
    assert(!f.params.values["TrqiUpdaterReady"] && f.ready_onroad_cleared);
    assert(f.tick());
    assert((f.params.reads == std::vector<std::string>{"IsOnroad"}));
  }
  // Failed health polls must not consume the first-ONROAD cleanup transition.
  for (int failure = -1; failure < 3; ++failure) {
    Fixture f;
    f.params.values = {{"IsOnroad", true}, {"TrqiUpdaterReady", true}};
    f.panda.fail_health = failure == -1;
    f.panda.fail_can_health = failure;
    assert(!f.tick());
    assert(!f.onroad && f.params.reads.empty() && f.params.writes.empty());
    f.panda.fail_health = false;
    f.panda.fail_can_health = -1;
    assert(f.tick());
    assert(!f.params.values["TrqiUpdaterReady"]);
  }
  puts("PASS: 32 OFFROAD gate combinations, ONROAD restart/lease revocation, 200 steady polls, release/regrant, write retry, health failures");
}
'''
program = preamble + '\nenum class IgnitionOverride' + enum + '};\n'
program += 'std::optional<bool> send_panda_states(' + function + checks
with tempfile.TemporaryDirectory() as temp:
  path = Path(temp)
  (path / 'check.cc').write_text(program)
  subprocess.run(['clang++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I.', '-Icereal', '-Imsgq',
                  str(path / 'check.cc'), '-lcapnp', '-lkj', '-o', str(path / 'check')], cwd=ROOT, check=True)
  subprocess.run([str(path / 'check')], check=True)
