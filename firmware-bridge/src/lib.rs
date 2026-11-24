// SPDX-License-Identifier: GPL-3.0-or-later
//! Real firmware-in-the-loop bridge for the Potok-M simulator.
//!
//! Unlike a host port of the guard decisions, this crate links the published
//! firmware crate and executes its actual `potok_m_firmware::step()` once per
//! simulator frame. The link, phase FSM, interlocks, watchdog policy, reply
//! codec and board command sink are the firmware's own types; the simulator
//! only supplies frame-derived inputs and a proposed AI command.
//!
//! Pinned source: `potok-m-firmware` @ [`FIRMWARE_REV`].
//!
//! **What is not claimed:** no flight hardware, no electrical or timing
//! fidelity, no UART baud rate, no measured HIL latency. This runs the
//! firmware's host-testable decision code in-process against simulated frames.
//! The command scale between the two systems is a full-scale proportionality,
//! not a physical actuator calibration (see [`mps2_to_raw`]).

use potok_m_firmware::{
    control::ShieldSample, interlocks::Interlocks, mock::MockBoard, step, watchdog::WatchdogPolicy,
    Command as FwCommand, LinkState, PhaseEvent, PhaseFsm, ReplyFrame, MAX_CORRECTION_RAW,
};
use potok_sim::{
    control::MAX_COMMAND_MPS2,
    dynamics::Phase,
    firmware_adapter::{Command, HostControl},
    sensor::{Frame, FLAG_MOTOR_ARMED},
};

/// The exact firmware revision compiled into this bridge. A dependency bump
/// must be accompanied by a re-read of the mapped gate behaviour in
/// `docs/FIRMWARE_IN_THE_LOOP.md`.
pub const FIRMWARE_REV: &str = "dfba1305c8dc2c17c3e9ca5fa83c25cafed3b4c4";

/// Wire reply version from `uart.yaml`, as decoded by the firmware's pinned
/// `potok-protocol` (`fdc485d14e8e10f500c2b81fce4ca9e5b1e64a55`). The firmware
/// crate does not re-export this constant, so it is restated here and pinned by
/// the reply round-trip test.
pub const REPLY_VERSION: u16 = 1;

/// Map a bounded simulator lateral-acceleration demand onto the firmware's
/// bounded raw command: `raw = proposed / MAX_COMMAND_MPS2 * MAX_CORRECTION_RAW`.
///
/// **This is not a physical calibration.** The firmware raw unit is 0.001 rad
/// of fin correction while the simulator command is m/s² of desired lateral
/// acceleration, and no published rad→m/s² actuator calibration exists. This
/// only maps one bounded full scale onto the other so the real firmware gates
/// (plausibility, slew, throttle, phase, health) see a proportional demand.
/// It makes no claim about what deflection produces what acceleration.
///
/// A non-finite demand is mapped to zero at this trust boundary; the caller is
/// expected to use [`Command::checked`] for out-of-range demands.
#[inline]
pub fn mps2_to_raw(mps2: f32) -> i16 {
    if !mps2.is_finite() {
        return 0;
    }
    let raw = (f64::from(mps2) / MAX_COMMAND_MPS2 * f64::from(MAX_CORRECTION_RAW)).round();
    raw.clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16
}

/// Inverse of [`mps2_to_raw`], applied to the command the firmware actually put
/// on the board (`MockBoard.applied.last()`). Same caveat: full-scale
/// proportionality, not a rad→m/s² actuator model.
#[inline]
pub fn raw_to_mps2(raw: i16) -> f32 {
    (f64::from(raw) / f64::from(MAX_CORRECTION_RAW) * MAX_COMMAND_MPS2) as f32
}

/// Flight-order rank used to emit each phase event at most once. Pad and Boost
/// both stay in the firmware's safe `BURN` phase (rank 0).
fn phase_rank(phase: Phase) -> u8 {
    match phase {
        Phase::Pad | Phase::Boost => 0,
        Phase::Coast => 1,
        Phase::Apogee => 2,
        Phase::Descent => 3,
        Phase::Landed => 4,
    }
}

/// Owns the real firmware objects and implements the simulator's host-control
/// seam. One `decide` call == one firmware `step` tick.
#[derive(Debug)]
pub struct FirmwareLinkControl {
    board: MockBoard,
    fsm: PhaseFsm,
    link: LinkState,
    interlock: Interlocks,
    policy: WatchdogPolicy,
    /// Local echo counter, starts at 0 and advances once per frame, matching
    /// `SeqGate::issue()` inside the firmware's `step`. The reply must echo the
    /// request seq the firmware issues on that same tick.
    next_seq: u16,
    /// Highest simulator phase rank already reported to the firmware.
    phase_rank: u8,
}

impl Default for FirmwareLinkControl {
    fn default() -> Self {
        Self::new()
    }
}

impl FirmwareLinkControl {
    pub fn new() -> Self {
        Self {
            board: MockBoard::new(),
            fsm: PhaseFsm::new(),
            link: LinkState::new(),
            interlock: Interlocks::default(),
            policy: WatchdogPolicy::new(),
            next_seq: 0,
            phase_rank: 0,
        }
    }

    /// The firmware FSM's own read-only view, for tests and telemetry.
    pub fn fsm(&self) -> &PhaseFsm {
        &self.fsm
    }

    pub fn link(&self) -> &LinkState {
        &self.link
    }

    pub fn interlock(&self) -> &Interlocks {
        &self.interlock
    }

    pub fn policy(&self) -> &WatchdogPolicy {
        &self.policy
    }

    pub fn board(&self) -> &MockBoard {
        &self.board
    }

    pub fn next_seq(&self) -> u16 {
        self.next_seq
    }

    /// Emit the firmware phase event for a newly observed simulator phase, at
    /// most once per phase and never backwards. Pad/Boost produce no event.
    fn phase_event(&mut self, phase: Phase) -> Option<PhaseEvent> {
        let rank = phase_rank(phase);
        if rank <= self.phase_rank {
            return None;
        }
        self.phase_rank = rank;
        Some(match phase {
            Phase::Coast => PhaseEvent::Burnout,
            Phase::Apogee => PhaseEvent::Apogee,
            Phase::Descent => PhaseEvent::Descent,
            Phase::Landed => PhaseEvent::Touchdown,
            // rank 0 never exceeds the initial rank and can never be reached
            // again while moving forward.
            Phase::Pad | Phase::Boost => return None,
        })
    }
}

impl HostControl for FirmwareLinkControl {
    fn decide(&mut self, frame: &Frame, proposed: Command) -> Command {
        // A demand outside the simulator's command box is refused before it
        // can be encoded, mirroring `Command::checked`'s boundary rule.
        let proposed = proposed.checked().unwrap_or(Command {
            ay: 0.0,
            ax: 0.0,
            ai_healthy: proposed.ai_healthy,
        });

        // Simulator frame -> firmware board inputs. The board's phase event is
        // consumed by `step` before the command gates run.
        self.board.now_us = frame.time_us;
        self.board.sensor_flags = frame.flags;
        self.board.motor_armed = frame.flags().has(FLAG_MOTOR_ARMED);
        self.board.cycle_complete = true;
        self.board.phase_event = self.phase_event(frame.phase);

        // The firmware shield takes pitot dp Pa and Mach. The sim frame has a
        // real pitot reading but no static pressure or speed of sound, so a
        // Mach estimate cannot be derived from it. 0.0 is a declared
        // placeholder, NOT a measurement: the pitot-validity gate is real, the
        // Mach-limit gate cannot veto from this bridge. See the doc.
        self.link.set_shield_sample(ShieldSample {
            pitot_dp: frame.pitot_dp,
            mach_est: 0.0,
        });

        // Reply echoes the seq the firmware will issue on this tick. Axis
        // convention is first-to-first: sim [ay, ax] -> raw [pitch, yaw].
        let reply = ReplyFrame {
            seq: self.next_seq,
            g_pitch: mps2_to_raw(proposed.ay),
            g_yaw: mps2_to_raw(proposed.ax),
            confidence: u8::MAX,
            ai_healthy: u8::from(proposed.ai_healthy),
            version: REPLY_VERSION,
        };
        self.next_seq = self.next_seq.wrapping_add(1);
        self.board.queue_reply(&reply);

        // The actual firmware decision path, including interlocks and the
        // watchdog policy, runs on every frame.
        let _result = step(
            &mut self.board,
            &mut self.fsm,
            &mut self.link,
            &mut self.interlock,
            &mut self.policy,
            true,
        );

        let applied = self
            .board
            .applied
            .last()
            .copied()
            .unwrap_or(FwCommand::ZERO);
        Command {
            ay: raw_to_mps2(applied.pitch),
            ax: raw_to_mps2(applied.yaw),
            ai_healthy: proposed.ai_healthy,
        }
    }

    fn reset_link(&mut self) {
        self.link.reset();
        self.fsm = PhaseFsm::new();
        self.interlock.reset();
        self.policy.reset_ai_latch();
        self.policy.reset_deadman();
        self.board = MockBoard::new();
        self.next_seq = 0;
        self.phase_rank = 0;
    }
}
