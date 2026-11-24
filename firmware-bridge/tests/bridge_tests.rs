// SPDX-License-Identifier: GPL-3.0-or-later
//! Gates that can only pass if the real firmware crate ran.

use potok_firmware_bridge::{mps2_to_raw, raw_to_mps2, FirmwareLinkControl};
use potok_m_firmware::{interlocks::Refusal, protocol::Phase as FwPhase};
use potok_sim::{
    dynamics::Phase,
    firmware_adapter::{Command, HostControl},
    sensor::{wire_flags, Frame},
};

fn frame(phase: Phase, time_us: u32, pitot_dp: i16) -> Frame {
    Frame {
        seq: 0,
        time_us,
        phase,
        gyro: [0.0; 3],
        accel: [0.0; 3],
        mag: [0.0; 3],
        height: 100.0,
        pitot_dp,
        flags: wire_flags(phase, true, true, false),
        crc: 0,
        gust: false,
        shear: false,
        saturated: false,
        spike: false,
        pitot_valid: true,
        physics: [0.0; 6],
    }
}

fn demand(ay: f32, ax: f32, ai_healthy: bool) -> Command {
    Command { ay, ax, ai_healthy }
}

fn close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 1e-5,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn boost_command_is_zeroed_by_the_firmware_gates() {
    let mut ctl = FirmwareLinkControl::new();
    let out = ctl.decide(&frame(Phase::Boost, 0, 1000), demand(8.0, -6.0, true));
    assert_eq!(out.ay, 0.0);
    assert_eq!(out.ax, 0.0);
    assert_eq!(ctl.fsm().phase(), FwPhase::BURN);
    // The interlock's own per-reason counter: this refusal came from the
    // firmware's BOOST gate, not from a host-side stand-in.
    assert_eq!(ctl.link().shield_reasons.boost, 1);
}

#[test]
fn coast_command_is_accepted_and_the_reply_seq_echoes_the_firmware_request() {
    let mut ctl = FirmwareLinkControl::new();
    // Boost first: the phase event must move the firmware FSM to COAST before
    // the command gate runs on the next tick.
    ctl.decide(&frame(Phase::Boost, 0, 1000), demand(0.0, 0.0, true));
    let out = ctl.decide(&frame(Phase::Coast, 2000, 1000), demand(4.0, 0.0, true));

    assert_eq!(ctl.fsm().phase(), FwPhase::COAST);
    assert!(out.ay > 0.0, "coast command was refused: {out:?}");
    assert_eq!(ctl.link().last_request_seq(), Some(1));
    assert_eq!(ctl.next_seq(), 2);
    // Link-level acceptance counts both decoded replies; the Boost one is
    // zeroed downstream by the interlock, not rejected by the link.
    assert_eq!(ctl.link().counters.accepted, 2);
    // Firmware slew from zero: raw 10 -> 1.0 m/s^2, not the proposed 4.0.
    close(out.ay, 1.0);
}

#[test]
fn firmware_rate_limit_slews_a_full_scale_demand_over_frames() {
    let mut ctl = FirmwareLinkControl::new();
    let mut applied = Vec::new();
    for i in 0..3 {
        let out = ctl.decide(
            &frame(Phase::Coast, i * 2000, 1000),
            demand(10.0, 0.0, true),
        );
        applied.push(out.ay);
    }
    // potok-m-firmware interlocks: max_delta = 10 raw, min_interval = 1000 us.
    // 100 raw full scale can only climb 10 raw per accepted change.
    close(applied[0], 1.0);
    close(applied[1], 2.0);
    close(applied[2], 3.0);
    assert_eq!(ctl.interlock().counters.slewed, 3);
    assert_eq!(
        ctl.board().applied.last().unwrap().pitch,
        30,
        "the command reported must be what the board actually applied"
    );
}

#[test]
fn unhealthy_reply_latches_and_stays_latched_until_reset() {
    let mut ctl = FirmwareLinkControl::new();
    let healthy = ctl.decide(&frame(Phase::Coast, 0, 1000), demand(6.0, 0.0, true));
    assert!(healthy.ay > 0.0);

    let unhealthy = ctl.decide(&frame(Phase::Coast, 2000, 1000), demand(6.0, 0.0, false));
    assert_eq!(unhealthy.ay, 0.0);
    assert_eq!(unhealthy.ax, 0.0);
    assert!(ctl.link().is_disabled());
    assert!(ctl.policy().ai_latched());

    // A later healthy reply cannot clear an AI latch; only reset can.
    let late = ctl.decide(&frame(Phase::Coast, 4000, 1000), demand(6.0, 0.0, true));
    assert_eq!(late.ay, 0.0);
    assert!(ctl.link().is_disabled());

    ctl.reset_link();
    let recovered = ctl.decide(&frame(Phase::Coast, 0, 1000), demand(6.0, 0.0, true));
    assert!(recovered.ay > 0.0, "reset did not restore command flow");
}

#[test]
fn reset_recovers_all_firmware_objects() {
    let mut ctl = FirmwareLinkControl::new();
    ctl.decide(&frame(Phase::Coast, 0, 1000), demand(6.0, 0.0, true));
    ctl.decide(&frame(Phase::Coast, 2000, 1000), demand(6.0, 0.0, false));
    assert!(ctl.link().is_disabled());
    assert!(ctl.interlock().latched().is_some());
    assert!(ctl.policy().ai_latched());
    assert_eq!(ctl.fsm().phase(), FwPhase::COAST);
    assert_eq!(ctl.next_seq(), 2);

    ctl.reset_link();
    assert!(!ctl.link().is_disabled());
    assert_eq!(ctl.link().misses(), 0);
    assert_eq!(ctl.link().last_request_seq(), None);
    assert!(ctl.interlock().latched().is_none());
    assert!(!ctl.policy().ai_latched());
    assert_eq!(ctl.fsm().phase(), FwPhase::BURN);
    assert_eq!(ctl.next_seq(), 0);
    assert!(ctl.board().applied.is_empty());

    // Forward-only phase mapping restarts from BURN after reset.
    let zeroed = ctl.decide(&frame(Phase::Boost, 0, 1000), demand(8.0, 0.0, true));
    assert_eq!(zeroed.ay, 0.0);
}

#[test]
fn phase_events_are_emitted_once_and_only_forward() {
    let mut ctl = FirmwareLinkControl::new();
    let order = [
        (Phase::Boost, FwPhase::BURN),
        (Phase::Pad, FwPhase::BURN),
        (Phase::Coast, FwPhase::COAST),
        (Phase::Coast, FwPhase::COAST),
        (Phase::Apogee, FwPhase::APOGEE),
        (Phase::Descent, FwPhase::DESCENT),
        (Phase::Landed, FwPhase::LANDED),
        // A backwards sim phase must not rewind the firmware FSM or re-emit.
        (Phase::Coast, FwPhase::LANDED),
        (Phase::Boost, FwPhase::LANDED),
    ];
    for (i, (phase, expected)) in order.into_iter().enumerate() {
        ctl.decide(
            &frame(phase, (i * 2000) as u32, 1000),
            demand(0.0, 0.0, true),
        );
        assert_eq!(ctl.fsm().phase(), expected, "step {i} ({phase:?})");
    }
}

#[test]
fn fault_flag_latches_the_firmware_interlock() {
    let mut ctl = FirmwareLinkControl::new();
    let mut bad = frame(Phase::Coast, 0, 1000);
    // A saturated sensor is represented by the wire FAULT bit. The firmware's
    // independent interlock must latch, not merely zero one tick.
    bad.flags = wire_flags(Phase::Coast, true, true, true);
    let out = ctl.decide(&bad, demand(6.0, 0.0, true));
    assert_eq!(out.ay, 0.0);
    assert_eq!(ctl.interlock().latched(), Some(Refusal::Fault));
    assert!(ctl.link().is_disabled() || ctl.interlock().latched().is_some());
}

#[test]
fn full_scale_mapping_is_proportional_and_not_a_physical_calibration() {
    assert_eq!(mps2_to_raw(0.0), 0);
    assert_eq!(mps2_to_raw(1.0), 10);
    assert_eq!(mps2_to_raw(10.0), 100);
    assert_eq!(mps2_to_raw(-10.0), -100);
    assert_eq!(mps2_to_raw(f32::NAN), 0);
    close(raw_to_mps2(0), 0.0);
    close(raw_to_mps2(10), 1.0);
    close(raw_to_mps2(100), 10.0);
    close(raw_to_mps2(-100), -10.0);
}
