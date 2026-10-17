# Artemis Telemetry Link & Failsafe Daemon

> High-reliability binary datalink framing, real-time RF link quality estimation, and deterministic failsafe state machine for unmanned vehicles (UAV / UGV) operating over lossy communications channels.

[![Language](https://img.shields.io/badge/language-Rust-orange.svg)](https://www.rust-lang.org/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-passing-brightgreen.svg)]()

---

## 1. System Overview

Autonomous unmanned platforms (such as long-range surveillance drones and autonomous ground rovers) communicate with Ground Control Stations (GCS) and companion computers across contested, bandwidth-constrained RF and satellite links. Under real operating conditions, packet drop rates regularly reach 15–30%, multipath interference causes bit flips, and radio links experience intermittent blackouts.

If telemetry transport is naive (e.g. relying on standard TCP with head-of-line blocking, or unverified UDP packets), stale state piles up, corrupted command bytes can trigger uncommanded maneuvers, or a silent link loss can leave an aircraft flying blind until fuel/battery exhaustion.

`artemis-telemetry-link` is a high-reliability vehicle systems daemon built in Rust. It implements:
1. **Zero-panic binary datalink framing** with sync words, monotonic sequence numbering, and CRC-16-CCITT frame validation.
2. **Real-time RF link monitor** tracking Packet Delivery Ratio (PDR), sequence gaps, out-of-order arrivals, and bit-flip rejections.
3. **Deterministic vehicle state machine** enforcing strict safety invariants, watchdog heartbeat timeouts, battery boundary guards, and idempotent command replay handling.

```
+------------------------+                    +------------------------------------+
| Ground Control Station |                    |  Artemis Companion Computer Daemon |
|      (GCS / C2)        |                    |         (Archer / Duma OS)         |
+------------------------+                    +------------------------------------+
            |                                                   |
            | === [Heartbeat / Commands] =====================> |
            |      (0xAA55 Sync Word | Seq | CRC-16)            |
            |                                                   |
            |          ~~~~ [Contested RF Link] ~~~~            |
            |          (Packet Loss & Noise Drops)              |
            |                                                   |
            |                                                   v
            |                                       [Frame Parser & CRC-16]
            |                                         - Corrupted? -> Drop & Log
            |                                         - Valid?     -> Update PDR
            |                                                   |
            |                                                   v
            |                                       [Link Quality Monitor]
            |                                         - Compute PDR %
            |                                         - Track Out-of-Order / Gaps
            |                                                   |
            |                                                   v
            |                                      [Failsafe State Machine]
            |                                         - Heartbeat Watchdog
            |                                         - Command Idempotency
            |                                         - Transitions:
            |                                           HOLD -> RTL -> LAND
            |                                                   |
            | <== [Vehicle Telemetry & Command ACKs] ===========|
```

---

## 2. Datalink Binary Frame Specification

Every datagram on the wire conforms to the packed 17-byte header plus variable payload and 2-byte CRC-16 trailer:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|       SYNC_MAGIC (0xAA55)     |   MSG_TYPE    |               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+               +
|                   SEQUENCE_ID (u32, Big-Endian)               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                               |
+                 TIMESTAMP_MS (u64, Big-Endian)                +
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|       PAYLOAD_LEN (u16)       |    PAYLOAD BYTES (0..N) ...   |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|    ... PAYLOAD BYTES          |       CRC-16-CCITT (u16)      |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

### Frame Fields:
- **`SYNC_MAGIC` (2 bytes):** `0xAA 0x55` sync bytes to align frame boundaries.
- **`MSG_TYPE` (1 byte):** `0x01` Heartbeat, `0x02` Telemetry, `0x03` Command, `0x04` CommandAck.
- **`SEQUENCE_ID` (4 bytes):** Monotonically increasing packet counter for sequence gap tracking.
- **`TIMESTAMP_MS` (8 bytes):** Sender epoch timestamp in milliseconds.
- **`PAYLOAD_LEN` (2 bytes):** Size in bytes of the variable payload (bounded to max 1024 bytes).
- **`CRC-16-CCITT` (2 bytes):** Big-endian cyclic redundancy check polynomial `0x1021` calculated over the entire header and payload.

---

## 3. Vehicle State Machine & Safety Invariants

The vehicle software enforces a strict Deterministic Finite Automaton (DFA) avoiding any undefined state:

```
 [BOOT] ───> [DISARMED] ───> [ARMED] ───> [IN_FLIGHT_MANUAL] <───> [AUTONOMOUS_MISSION]
                 ▲                            │                               │
                 │                            │                               │
                 │                 (Lost Heartbeat > 3.0s)         (Lost Heartbeat > 3.0s)
                 │                            │                               │
                 │                            └───────────────┬───────────────┘
                 │                                            │
                 │                                            v
                 │                                   [LOST_LINK_HOLD]
                 │                                            │
                 │                            (Link Lost Timeout > 8.0s)
                 │                                            │
                 │                                            v
                 └────── (Touchdown / Disarm) <─────── [FAILSAFE_RTL]
                                                              │
                                                      (Battery < 5%)
                                                              │
                                                              v
                                                      [EMERGENCY_LAND]
```

### Core Invariants Enforced:
1. **Heartbeat Loss Watchdog:** If the vehicle is airborne (`InFlightManual` or `AutonomousMission`) and no valid heartbeat frame arrives for `> 3,000ms`, the state machine automatically transitions to `LOST_LINK_HOLD` (loitering or hovering in place).
2. **Failsafe Return-to-Launch (RTL):** If the radio link remains severed for `> 8,000ms`, the vehicle transitions to `FAILSAFE_RTL`, calculating an autonomous return vector without requiring operator intervention.
3. **Heartbeat Reconnection Recovery:** If link contact is restored while in `LOST_LINK_HOLD`, the vehicle safely returns to flight control mode.
4. **Command Idempotency:** If a command packet (`ARM`, `TAKEOFF`, `SET_WAYPOINT`) is retransmitted by GCS due to an ACK drop on the downlink, the vehicle recognizes the `command_id` in its history cache and returns `AckStatus::AlreadyExecuted` without repeating physical actions.
5. **Battery Ceilings:** 
   - Arming is rejected if battery is below 15%.
   - Autonomous missions are rejected if battery is below 20%.
   - If battery drops below 10%, autonomous `FAILSAFE_RTL` triggers.
   - If battery drops below 5%, `EMERGENCY_LAND` engages immediately.

---

## 4. Verification & Test Suite

The codebase includes exhaustive unit tests covering edge cases and adversarial inputs:

```bash
cargo test
```

### Verified Test Cases:
- `test_crc16_standard_vector`: Asserts standard ASCII CCITT vector (`"123456789"` -> `0x29B1`).
- `test_crc16_detects_single_bit_flip`: Asserts single-bit mutation detection.
- `test_frame_encode_decode_roundtrip`: End-to-end binary serialization and deserialization.
- `test_rejects_corrupted_crc`: Verifies corrupted payloads are immediately dropped with `CorruptedChecksum`.
- `test_rejects_bad_magic`: Protects against misaligned sync bytes.
- `test_rejects_truncated_buffer`: Prevents out-of-bounds reads on truncated packets.
- `test_valid_arm_and_takeoff_sequence`: Validates safe nominal launch workflow.
- `test_prevents_disarm_while_airborne`: Rejects unsafe mid-air disarm commands.
- `test_idempotent_command_replay`: Asserts duplicate command IDs return `AlreadyExecuted`.
- `test_watchdog_lost_link_and_failsafe_rtl`: Verifies 3.0s and 8.0s failsafe timeout triggers.
- `test_heartbeat_recovers_from_lost_link_hold`: Verifies automatic recovery upon signal restoration.
- `test_detects_dropped_packets_and_pdr`: Verifies link quality calculation over lossy sequence gaps.

---

## 5. Running the Simulation

Execute the interactive simulation showing nominal flight, real-time RF packet drop handling, simulated radio jamming, and autonomous failsafe engagement:

```bash
cargo run --release
```

### Sample Output:
```text
========================================================================
   ARTEMIS OS // VEHICLE SYSTEMS & AUTONOMY TELEMETRY DAEMON            
   Deterministic Failsafe State Machine & Lossy RF Link Monitor         
========================================================================

[INIT] Vehicle state: DISARMED
[INIT] Initial battery: 95%
[INIT] RF Profile: 15% packet loss, 2% bit corruption, 3000ms heartbeat timeout

>>> PHASE 1: Command Uplink & Mission Initiation
    [CMD 101: ARM] Ack Result: Accepted
    [STATE] Vehicle is now: ARMED
    [CMD 102: TAKEOFF] Ack Result: Accepted
    [STATE] Vehicle is now: IN_FLIGHT_MANUAL

>>> PHASE 2: Autonomous Flight & Telemetry Streams (Active Link)
    [t=0001ms] State: IN_FLIGHT_MANUAL   Alt: 15.0m | Bat: 95% | PDR: 100.0% | Drops: 0
    [t=0502ms] State: IN_FLIGHT_MANUAL   Alt: 20.0m | Bat: 94% | PDR: 100.0% | Drops: 0
    [WARN] Bit-flip detected! Expected CRC 0x489F, calculated 0xD321 -> Frame Dropped
    [t=1504ms] State: IN_FLIGHT_MANUAL   Alt: 30.0m | Bat: 93% | PDR:  80.0% | Drops: 1
    ...

>>> PHASE 3: Radio Blackout Simulated (Jamming / Ground Link Severed)
    [ALERT] Ground Station signal completely terminated. Watchdog monitoring...

    [t=4505ms] State: IN_FLIGHT_MANUAL   (Watchdog counting down...)
    [WATCHDOG TRIGGER] State Transition: IN_FLIGHT_MANUAL -> LOST_LINK_HOLD
    [t=6508ms] State: LOST_LINK_HOLD     (Watchdog counting down...)
    [WATCHDOG TRIGGER] State Transition: LOST_LINK_HOLD -> FAILSAFE_RTL

    [AUTONOMOUS FAILSAFE ENGAGED] Vehicle computing autonomous Return-to-Launch vector.

========================================================================
   DATALINK DIAGNOSTICS & SYSTEM METRICS REPORT                        
========================================================================
   Total Packets Received   : 7
   Estimated Lost Frames    : 1
   CRC16 Checksum Rejections: 1
   Out-of-Order Packets     : 0
   Packet Delivery Ratio    : 87.50%
   Final Vehicle Safety State: FAILSAFE_RTL
========================================================================
```

---

## 6. Architecture & Tech Stack

- **Language:** Rust (2021 Edition)
- **Async Runtime:** `tokio` (Event timers and multi-threaded datagram handling)
- **Integrity:** Standalone CRC-16-CCITT implementation (`0x1021`)
- **Optimization:** Single codegen-unit, Link-Time Optimization (LTO), and stripped release binary for resource-constrained ARM companion computers.

---

## Author

**Olumide Lala**  
- GitHub: [@Olalolo22](https://github.com/Olalolo22)  
- Engineering Dossier: [portfolio-research](https://github.com/Olalolo22/portfolio-research)
