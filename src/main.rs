//! Artemis Vehicle Telemetry & Failsafe Link Daemon
//!
//! Demonstrates fault-tolerant telemetry framing, lossy datalink monitoring,
//! HMAC-SHA256 authenticated command dispatch, and deterministic failsafe state transitions.

pub mod auth;
pub mod crc;
pub mod frame;
pub mod link_monitor;
pub mod messages;
pub mod simulator;
pub mod state_machine;

use auth::CommandAuthenticator;
use frame::{Frame, MsgType};
use link_monitor::LinkMonitor;
use messages::{Command, Heartbeat, SignedCommand, Telemetry, VehicleAction};
use simulator::{ChannelProfile, LossyChannel};
use state_machine::{VehicleState, VehicleStateMachine};
use std::time::{Duration, Instant};

const PRE_SHARED_DEFENSE_KEY: &[u8] = b"TERRA_ARTEMIS_SYSTEMS_PSK_KEY_2026";

#[tokio::main]
async fn main() {
    println!("========================================================================");
    println!("   ARTEMIS OS // VEHICLE SYSTEMS & AUTONOMY TELEMETRY DAEMON            ");
    println!("   Authenticated Datalink, RF Monitor & Deterministic Failsafe FSM      ");
    println!("========================================================================\n");

    let start_time = Instant::now();
    let authenticator = CommandAuthenticator::new(PRE_SHARED_DEFENSE_KEY);
    let mut vehicle_fsm = VehicleStateMachine::new(95, 0);
    let mut uplink_monitor = LinkMonitor::new();
    let mut downlink_monitor = LinkMonitor::new();

    // Configure a simulated contested RF link (15% drop, 2% bit flip)
    let uplink_channel = LossyChannel::new(ChannelProfile {
        drop_probability: 0.15,
        corruption_probability: 0.02,
    });
    let downlink_channel = LossyChannel::new(ChannelProfile {
        drop_probability: 0.10,
        corruption_probability: 0.01,
    });

    println!("[INIT] Vehicle state: {}", vehicle_fsm.state());
    println!("[INIT] Initial battery: {}%", vehicle_fsm.battery_pct());
    println!("[INIT] Security: HMAC-SHA256 message signing (128-bit truncated MAC, anti-replay)");
    println!("[INIT] RF Profile: 15% uplink loss, 10% downlink loss, 3000ms watchdog timeout\n");

    let session_id: u32 = 1;
    let mut gcs_seq: u32 = 1;
    let mut vehicle_seq: u32 = 1;

    // Helper closure to dispatch signed commands over the uplink channel
    let dispatch_command = |cmd: Command, seq: &mut u32, fsm: &mut VehicleStateMachine, monitor: &mut LinkMonitor| {
        let now_ms = start_time.elapsed().as_millis() as u64;
        let cmd_bytes = cmd.to_bytes().expect("Serialization failed");
        let sig = authenticator.sign_command(session_id, now_ms, cmd.command_id, &cmd_bytes);

        let signed_cmd = SignedCommand {
            session_id,
            timestamp_ms: now_ms,
            command: cmd.clone(),
            signature: sig,
        };
        let payload = signed_cmd.to_bytes().expect("Serialization failed");
        let frame = Frame::new(MsgType::Command, *seq, now_ms, payload);
        *seq += 1;

        // Transmit over simulated uplink channel
        if let Some(wire_bytes) = uplink_channel.send(&frame) {
            match Frame::decode(&wire_bytes) {
                Ok((decoded, _)) => {
                    monitor.record_valid_packet(decoded.sequence_id);
                    if let Ok(signed_rx) = SignedCommand::from_bytes(&decoded.payload) {
                        let inner_bytes = signed_rx.command.to_bytes().unwrap();
                        let verify_res = authenticator.verify_command(
                            signed_rx.session_id,
                            signed_rx.timestamp_ms,
                            signed_rx.command.command_id,
                            &inner_bytes,
                            &signed_rx.signature,
                            now_ms,
                        );

                        match verify_res {
                            Ok(()) => {
                                let ack = fsm.process_command(signed_rx.command);
                                println!("    [CMD {:>3}: {:<7}] Authenticated & Executed -> Result: {:?}", cmd.command_id, format!("{:?}", cmd.action), ack.status);
                            }
                            Err(auth_err) => {
                                println!("    [SECURITY ALERT] Command {:>3} signature verification rejected: {}", cmd.command_id, auth_err);
                            }
                        }
                    }
                }
                Err(frame::FrameError::CorruptedChecksum { expected, found }) => {
                    monitor.record_corrupted_crc();
                    println!("    [UPLINK CORRUPTED] Bit-flip detected! Expected CRC 0x{:04X}, found 0x{:04X} -> Dropped", expected, found);
                }
                Err(err) => println!("    [UPLINK ERROR] {}", err),
            }
        } else {
            println!("    [UPLINK LOSS] Command {:>3} packet dropped in flight by RF channel", cmd.command_id);
        }
    };

    // Phase 1: Uplink Authenticated Commands
    println!(">>> PHASE 1: Authenticated Command Uplink (GCS -> Vehicle)");
    dispatch_command(Command { command_id: 101, action: VehicleAction::Arm }, &mut gcs_seq, &mut vehicle_fsm, &mut uplink_monitor);
    dispatch_command(Command { command_id: 102, action: VehicleAction::Takeoff { target_alt_m: 50.0 } }, &mut gcs_seq, &mut vehicle_fsm, &mut uplink_monitor);
    println!("    [CURRENT VEHICLE STATE] {}\n", vehicle_fsm.state());

    // Phase 2: Live Mission Telemetry & Heartbeat Exchanges
    println!(">>> PHASE 2: Live Flight Telemetry & Uplink/Downlink Exchanges");
    let mut altitude = 10.0;
    let mut battery = 95;

    for tick in 1..=8 {
        let elapsed_ms = start_time.elapsed().as_millis() as u64;

        // Ground station sends periodic heartbeat
        let hb = Heartbeat {
            vehicle_id: 1,
            uptime_secs: tick,
            armed: true,
        };
        let hb_payload = hb.to_bytes().unwrap();
        let hb_frame = Frame::new(MsgType::Heartbeat, gcs_seq, elapsed_ms, hb_payload);
        gcs_seq += 1;

        if let Some(wire_bytes) = uplink_channel.send(&hb_frame) {
            match Frame::decode(&wire_bytes) {
                Ok((decoded, _)) => {
                    uplink_monitor.record_valid_packet(decoded.sequence_id);
                    vehicle_fsm.record_heartbeat(elapsed_ms);
                }
                Err(frame::FrameError::CorruptedChecksum { .. }) => {
                    uplink_monitor.record_corrupted_crc();
                }
                _ => {}
            }
        }

        // Vehicle simulates flight dynamics and downlinks telemetry
        altitude += 5.0;
        if tick % 2 == 0 {
            battery -= 1;
            let _ = vehicle_fsm.update_battery(battery);
        }

        let telem = Telemetry {
            vehicle_id: 1,
            roll_deg: 0.5,
            pitch_deg: 1.2,
            yaw_deg: 125.0,
            altitude_m: altitude,
            ground_speed_mps: 14.5,
            battery_mv: (battery as u16) * 260,
            battery_pct: battery,
            rssi_dbm: -72,
        };

        let telem_payload = telem.to_bytes().unwrap();
        let telem_frame = Frame::new(MsgType::Telemetry, vehicle_seq, elapsed_ms, telem_payload);
        vehicle_seq += 1;

        if let Some(downlink_bytes) = downlink_channel.send(&telem_frame) {
            match Frame::decode(&downlink_bytes) {
                Ok((decoded, _)) => {
                    downlink_monitor.record_valid_packet(decoded.sequence_id);
                }
                Err(frame::FrameError::CorruptedChecksum { .. }) => {
                    downlink_monitor.record_corrupted_crc();
                }
                _ => {}
            }
        }

        let up_stats = uplink_monitor.statistics();
        let down_stats = downlink_monitor.statistics();

        println!(
            "    [t={:04}ms] State: {:<18} Alt: {:>4.1}m | Bat: {:>2}% | Up-PDR: {:>5.1}% | Down-PDR: {:>5.1}%",
            elapsed_ms,
            vehicle_fsm.state().to_string(),
            telem.altitude_m,
            telem.battery_pct,
            up_stats.packet_delivery_ratio_pct,
            down_stats.packet_delivery_ratio_pct,
        );

        tokio::time::sleep(Duration::from_millis(500)).await;
    }

    // Phase 3: Simulated Radio Blackout / RF Jamming
    println!("\n>>> PHASE 3: Complete Radio Blackout (Jamming / Severe Link Severance)");
    println!("    [ALERT] Ground Station transmission ceased. Heartbeat watchdog monitoring...\n");

    for _ in 1..=10 {
        tokio::time::sleep(Duration::from_millis(1000)).await;
        let elapsed_ms = start_time.elapsed().as_millis() as u64;

        let state_before = vehicle_fsm.state();
        if let Some(new_state) = vehicle_fsm.evaluate_watchdog(elapsed_ms) {
            println!(
                "    [WATCHDOG TRIGGER] State Transition: {} -> {}",
                state_before, new_state
            );
        } else {
            println!(
                "    [t={:04}ms] State: {:<18} (Watchdog counting down...)",
                elapsed_ms,
                vehicle_fsm.state().to_string()
            );
        }

        if vehicle_fsm.state() == VehicleState::FailsafeRtl {
            println!("\n    [AUTONOMOUS FAILSAFE ENGAGED] Vehicle computing autonomous Return-to-Launch vector.");
            break;
        }
    }

    // Phase 4: Final Link Diagnostics
    println!("\n========================================================================");
    println!("   DATALINK DIAGNOSTICS & SYSTEM METRICS REPORT                        ");
    println!("========================================================================");
    let up_stats = uplink_monitor.statistics();
    let down_stats = downlink_monitor.statistics();
    println!("   Uplink Frames Received   : {}", up_stats.packets_received);
    println!("   Uplink Frames Dropped    : {}", up_stats.packets_dropped_estimate);
    println!("   Uplink CRC Corruptions   : {}", up_stats.corrupted_crc_count);
    println!("   Uplink Packet Delivery   : {:.2}%", up_stats.packet_delivery_ratio_pct);
    println!("   Downlink Packet Delivery : {:.2}%", down_stats.packet_delivery_ratio_pct);
    println!("   Final Vehicle Safety State: {}", vehicle_fsm.state());
    println!("========================================================================\n");
}
