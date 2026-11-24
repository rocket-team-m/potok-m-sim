<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Firmware-in-the-loop bridge (`potok-firmware-bridge`)

This crate is a **real** firmware bridge. It does not port or transcribe guard
decisions: it links the published firmware crate and runs the actual
`potok_m_firmware::step()` once per simulator frame. `docs/FIRMWARE_BRIDGE.md`
(on the `main` lane) describes the earlier host port and why that was *not*
firmware-in-the-loop. This document supersedes that status for this workspace:
the bridge crate compiles the firmware and executes its decision path.

## Pin

| item | value |
|---|---|
| firmware crate | `potok-m-firmware` |
| source | `https://github.com/rocket-team-m/potok-m-firmware.git` |
| revision | `dfba1305c8dc2c17c3e9ca5fa83c25cafed3b4c4` |
| firmware's protocol pin | `potok-protocol` @ `fdc485d14e8e10f500c2b81fce4ca9e5b1e64a55` |
| published in | `firmware-bridge/src/lib.rs` as `FIRMWARE_REV` |
| published reply version | `REPLY_VERSION = 1` (from `uart.yaml`; the firmware crate does not re-export it) |

Cargo resolves both protocol sources (the simulator uses its own `potok-protocol`
from `main`). This is deliberate: the bridge never passes a simulator wire type
to the firmware. It passes only frame-derived scalars into the firmware's own
`MockBoard`, and the firmware builds and decodes its own `ReplyFrame`.

## What actually runs

Each `FirmwareLinkControl::decide(frame, proposed)` call executes, from the
pinned firmware source:

- `potok_m_firmware::step()` — the production control tick, not `step_link`;
- `LinkState` — seq issue/check, miss latch, `on_reply`, shield tally;
- `PhaseFsm` (from `phase.rs`) — forward-only phase gating;
- `Interlocks::evaluate_shield` — phase/health/fault gates, plausibility,
  slew (`max_delta = 10` raw) and throttle (`min_interval_us = 1000`), plus the
  Mach/pitot shield;
- `WatchdogPolicy::on_control_cycle` — deadline monitor, AI latch, feed
  decision;
- `MockBoard` — the host HAL: `apply_command` records what the controller
  actually applied, and the reply codec encodes/decodes the queued frame.

The returned simulator command is read from `MockBoard.applied.last()`, not from
the proposed demand. That distinction is what the rate-limit test pins.

## Frame → board mapping

| simulator (`potok_sim::sensor::Frame`) | firmware (`MockBoard` / `LinkState`) |
|---|---|
| `time_us` | `board.now_us` |
| `flags` | `board.sensor_flags` |
| `flags & FLAG_MOTOR_ARMED` | `board.motor_armed` (also sets the wire `MOTOR_ARMED` bit in `step`) |
| `cycle_complete` | always `true` |
| `phase` | one `PhaseEvent` when the phase rank moves forward |
| `pitot_dp` | `ShieldSample.pitot_dp` (real Pa reading) |
| — | `ShieldSample.mach_est = 0.0` (declared placeholder, see below) |

Phase rank is `Pad|Boost = 0`, `Coast = 1`, `Apogee = 2`, `Descent = 3`,
`Landed = 4`. An event is emitted only when the rank strictly increases, so a
phase held for many frames emits once and a backwards sim phase emits nothing.
`Pad` and `Boost` stay in firmware `BURN`, the safe phase where the AI output is
gated off. The events are exactly `Burnout` (→ COAST), `Apogee` (→ APOGEE),
`Descent` (→ DESCENT) and `Touchdown` (→ LANDED).

### Sequence

The bridge keeps a local `u16` starting at `0`, advanced once per frame. It is
the echo value for the reply. Firmware `SeqGate::issue()` issues `0` on the
first tick and wraps at `u16::MAX`, so the reply matches the request that the
same `step()` call issues. `reset_link()` returns both counters to `0` /
`None`. This is a host-side echo, not a shared object with the firmware; the
gate still checks it on the real `LinkState`.

## Command scale — proportional, NOT a physical calibration

```
raw   = proposed / MAX_COMMAND_MPS2 * MAX_CORRECTION_RAW
mps2  = raw / MAX_CORRECTION_RAW * MAX_COMMAND_MPS2
```

`MAX_COMMAND_MPS2 = 10` (sim full scale, m/s²) and `MAX_CORRECTION_RAW = 100`
(firmware full scale, raw i16). Axis convention is first-to-first: sim `[ay, ax]`
maps to firmware `[g_pitch, g_yaw]` and back. Out-of-box demands are refused at
the boundary via `Command::checked()` before encoding.

**This is not a physical calibration.** The firmware raw unit is 0.001 rad of
fin correction; the simulator command is m/s² of desired lateral acceleration.
No published rad→m/s² actuator calibration exists, and none is invented here.
The mapping only puts two bounded scales in correspondence so the firmware's
own gates see a proportional demand. It makes no claim about what deflection
produces what acceleration, and no result from this bridge may be quoted as an
actuator or aerodynamic measurement.

## Real vs uncalibrated

| real (firmware code executes) | not calibrated / not claimed |
|---|---|
| link sequencing, miss and AI latches, reset | rad→m/s² actuator conversion (full-scale proportionality only) |
| phase FSM transitions and BURN gate | Mach estimate: `mach_est = 0.0` is a declared placeholder, not a reading |
| interlock plausibility, slew, throttle, BOOST/MOTOR_ARMED/FAULT gates | the Mach-limit veto cannot fire from this bridge (pitot-validity gate does) |
| watchdog policy and feed decision | hardware IWDG, control-loop timing, CPU load |
| reply encode/decode, CRC, version and seq checks | UART baud, DMA, electrical levels, board I/O |
| command actually applied to `MockBoard` | flight hardware, HIL latency, any flight-log equivalence |

The sim `Frame` carries pitot dynamic pressure but no static pressure or speed
of sound, so a Mach estimate cannot be derived from it without inventing an
atmospheric model. `mach_est = 0.0` keeps the firmware's Mach gate exercised
without fabricating air data; the pitot-validity gate uses the frame's real
`pitot_dp`. Add a real airdata source before quoting any Mach-related veto.

`reset_link()` resets the firmware `LinkState`, `PhaseFsm`, `Interlocks`,
`WatchdogPolicy` (AI latch and deadman), `MockBoard`, the local seq and the
phase-rank tracker. Nothing from the previous run survives.

## CLI runner

`cargo run -p potok-cli -- --episodes 1 --firmware-guard` runs the same bridge
through `HostAdapter<FirmwareLinkControl, MockAdapter>` and prints the firmware
interlock counters. This is a real firmware decision path with a deterministic
stand-in producer, not the trained AI and not HIL.

## Observed default-run integration finding (2026-11-25)

On the shipped default trajectory, one 30 s episode reports:

```
firmware first FAULT flag t=7800000us pitot_dp=30058Pa phase=Boost
episode 0: ... fw_accepted=0 fw_blocked=1500 fw_trips=1 fw_latched=Some(Fault)
```

The simulator sets the wire `FAULT` bit when the pitot reading saturates at
`pitot_sat = 30000 Pa` (and above the i16 ceiling). The real firmware's
`Interlocks` latch `Fault` permanently, so no AI command can be accepted for the
rest of that flight. This is not hidden by the bridge and is not fixed here: it
needs a protocol/hardware decision (a wider or scaled pitot field, a larger
sensor range, or a separate saturation bit that maps to a transient
`PitotInvalid` rather than a latching `Fault`). It is a real consequence of the
new firmware being in the loop, not a bridge bug.

## Gates

`firmware-bridge/tests/bridge_tests.rs` proves the firmware code ran, not just
that a function returned:

- `boost_command_is_zeroed_by_the_firmware_gates` — firmware interlock's own
  `shield_reasons.boost` counter moves to 1.
- `coast_command_is_accepted_and_the_reply_seq_echoes_the_firmware_request` —
  link `counters.accepted` moves and `last_request_seq()` matches the echoed
  reply.
- `firmware_rate_limit_slews_a_full_scale_demand_over_frames` — a full-scale
  demand climbs 10 raw per tick (1.0 → 2.0 → 3.0 m/s²) because the firmware's
  `max_delta = 10`; `board.applied.last().pitch == 30`.
- `unhealthy_reply_latches_and_stays_latched_until_reset` — `ai_healthy = 0`
  latches the link and policy, and a later healthy reply does not clear it.
- `reset_recovers_all_firmware_objects` — all five firmware objects, the seq
  and the phase tracker return to their initial state.
- `phase_events_are_emitted_once_and_only_forward` — repeated and backwards
  phases never move the FSM again.
- `full_scale_mapping_is_proportional_and_not_a_physical_calibration` — pins
  the documented formula and the finite guard.
