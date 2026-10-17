//! Artemis Vehicle Telemetry & Failsafe Link Daemon
//!
//! Demonstrates fault-tolerant telemetry framing, lossy datalink monitoring,
//! and deterministic failsafe state transitions for unmanned autonomous platforms.

pub mod crc;
pub mod frame;
pub mod link_monitor;
pub mod messages;
pub mod simulator;
pub mod state_machine;

use frame::{Frame, MsgType};
use link_monitor::LinkMonitor;
use messages::{Command, Heartbeat, Telemetry, VehicleAction};
use simulator::{ChannelProfile, LossyChannel};
use state_machine::{VehicleState, VehicleStateMachine};
use std::time::{Duration, Instant};

#[tokio::main]
async fn main() {
    println!("========================================================================");
    println!("   ARTEMIS OS // VEHICLE SYSTEMS & AUTONOMY TELEMETRY DAEMON            ");
    println!("   Deterministic Failsafe State Machine & Lossy RF Link Monitor         ");
    println!("========================================================================\n");

    let start_time = Instant::now();
    let mut vehicle_fsm = VehicleStateMachine::new(95, 0);
    let mut link_monitor = LinkMonitor::new();

    // Configure a simulated contested RF link (20% packet drop, 3% bit flip corruption)
    let channel = LossyChannel::new(ChannelProfile {
        drop_probability: 0.15,
        corruption_probability: 0.02,
    });

    println!("[INIT] Vehicle state: {}", vehicle_fsm.state());
    println!("[INIT] Initial battery: {}%", vehicle_fsm.battery_pct());
    println!("[INIT] RF Profile: 15% packet loss, 2% bit corruption, 3000ms heartbeat timeout\n");

    // Phase 1: Command Arm & Takeoff
    println!(">>> PHASE 1: Command Uplink & Mission Initiation");
    let arm_cmd = Command {
        command_id: 101,
        action: VehicleAction::Arm,
    };
    let ack = vehicle_fsm.process_command(arm_cmd);
    println!("    [CMD 101: ARM] Ack Result: {:?}", ack.status);
    println!("    [STATE] Vehicle is now: {}", vehicle_fsm.state());

    let takeoff_cmd = Command {
        command_id: 102,
        action: VehicleAction::Takeoff { target_alt_m: 50.0 },
    };
    let ack2 = vehicle_fsm.process_command(takeoff_cmd);
    println!("    [CMD 102: TAKEOFF] Ack Result: {:?}", ack2.status);
    println!("    [STATE] Vehicle is now: {}\n", vehicle_fsm.state());

    // Phase 2: Live Flight Telemetry Loop (4 seconds of intermittent link)
    println!(">>> PHASE 2: Autonomous Flight & Telemetry Streams (Active Link)");
    let mut seq: u32 = 1;
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
        let hb_frame = Frame::new(MsgType::Heartbeat, seq, elapsed_ms, hb_payload);
        seq += 1;

        // Pass through lossy channel
        if let Some(wire_bytes) = channel.send(&hb_frame) {
            match Frame::decode(&wire_bytes) {
                Ok((decoded, _)) => {
                    link_monitor.record_valid_packet(decoded.sequence_id);
                    vehicle_fsm.record_heartbeat(elapsed_ms);
                }
                Err(frame::FrameError::CorruptedChecksum { expected, found }) => {
                    link_monitor.record_corrupted_crc();
                    println!("    [WARN] Bit-flip detected! Expected CRC 0x{:04X}, calculated 0x{:04X} -> Frame Dropped", expected, found);
                }
                Err(err) => {
                    println!("    [WARN] Frame decode failure: {}", err);
                }
            }
        } else {
            // Packet dropped by RF channel
        }

        altitude += 5.0;
        if tick % 2 == 0 {
            battery -= 1;
            vehicle_fsm.update_battery(battery);
        }

        // Vehicle produces outgoing telemetry
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

        let stats = link_monitor.statistics();
        println!(
            "    [t={:04}ms] State: {:<18} Alt: {:>4.1}m | Bat: {:>2}% | PDR: {:>5.1}% | Drops: {}",
            elapsed_ms,
            vehicle_fsm.state().to_string(),
            telem.altitude_m,
            telem.battery_pct,
            stats.packet_delivery_ratio_pct,
            stats.packets_dropped_estimate
        );

        tokio::time::sleep(Duration::from_millis(500)).await;
    }

    // Phase 3: Simulated Complete Radio Blackout / Link Jamming
    println!("\n>>> PHASE 3: Radio Blackout Simulated (Jamming / Ground Link Severed)");
    println!("    [ALERT] Ground Station signal completely terminated. Watchdog monitoring...\n");

    for _ in 1..=10 {
        tokio::time::sleep(Duration::from_millis(1000)).await;
        let elapsed_ms = start_time.elapsed().as_millis() as u64;

        // Watchdog tick (no incoming heartbeats recorded)
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
    let final_stats = link_monitor.statistics();
    println!("   Total Packets Received   : {}", final_stats.packets_received);
    println!("   Estimated Lost Frames    : {}", final_stats.packets_dropped_estimate);
    println!("   CRC16 Checksum Rejections: {}", final_stats.corrupted_crc_count);
    println!("   Out-of-Order Packets     : {}", final_stats.out_of_order_count);
    println!("   Packet Delivery Ratio    : {:.2}%", final_stats.packet_delivery_ratio_pct);
    println!("   Final Vehicle Safety State: {}", vehicle_fsm.state());
    println!("========================================================================\n");
}
