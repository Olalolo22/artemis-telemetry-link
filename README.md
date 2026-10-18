# Artemis Telemetry Link & Failsafe Prototype

> A systems prototype in Rust: binary datalink framing, lossy RF link quality estimation, HMAC-SHA256 command authentication, and deterministic failsafe state machine for unmanned platforms (UAV / UGV).

[![Language](https://img.shields.io/badge/language-Rust-orange.svg)](https://www.rust-lang.org/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-30%20passed-brightgreen.svg)]()

---

## 1. System Overview

Autonomous unmanned platforms (such as long-range surveillance drones and autonomous ground rovers) communicate with Ground Control Stations (GCS) and companion computers across contested, bandwidth-constrained RF links. Under field conditions, packet drop rates regularly reach 15–30%, multipath interference causes bit flips, and radio links experience intermittent blackouts.

If vehicle systems software is built naively, corrupted bytes can trigger uncommanded maneuvers, stale telemetry piles up, or an unauthenticated adversary on the same frequency can inject disarm commands.

`artemis-telemetry-link` is a vehicle systems software prototype built in Rust to model datalink communications, lossy channel recovery, and fail-safe state transitions. It implements:
1. **Binary datalink framing codec** with sync words (`0xAA55`), monotonic sequence numbering, bounds checks, and CRC-16-CCITT integrity verification.
2. **HMAC-SHA256 command authentication & anti-replay** (modeling MAVLink 2 message signing) with 128-bit truncated MACs, session isolation, and temporal replay windows.
3. **Real-time RF link quality estimator** tracking Packet Delivery Ratio (PDR), sequence anomalies, wraparound rollovers, and bit-flip rejections.
4. **Deterministic vehicle state machine** enforcing strict safety invariants: no airborne disarms without touchdown, ground hold protection, battery guards, and heartbeat watchdog fail-safes.

```
+------------------------+                    +------------------------------------+
| Ground Control Station |                    |  Artemis Companion Computer Daemon |
|      (GCS / C2)        |                    |         (Archer / Duma OS)         |
+------------------------+                    +------------------------------------+
            |                                                   |
            | === [Signed Commands & Heartbeats] =============> |
            |      (0xAA55 Sync | HMAC-SHA256 | CRC-16)         |
            |                                                   |
            |          ~~~~ [Contested RF Link] ~~~~            |
            |          (Packet Loss & Noise Drops)              |
            |                                                   |
            |                                                   v
            |                                       [Frame Parser & CRC-16]
            |                                         - Corrupted? -> Drop & Log
            |                                         - Valid?     -> Update Uplink PDR
            |                                                   |
            |                                                   v
            |                                       [HMAC Authentication Engine]
            |                                         - Verify 128-bit MAC
            |                                         - Check Timestamp Replay (<5s)
            |                                         - Check Session Isolation
            |                                                   |
            |                                                   v
            |                                       [Failsafe State Machine]
            |                                         - Heartbeat Watchdog (3s)
            |                                         - Idempotency & Action Checks
            |                                         - Transitions:
            |                                           HOLD -> RTL -> LAND
            |                                                   |
            | <== [Vehicle Telemetry Downlink] =================|
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

### Wire Fields:
- **`SYNC_MAGIC` (2 bytes):** `0xAA 0x55` sync words to align frame boundaries. Includes a byte-stream scanning helper (`scan_sync_word`) to recover frame alignment on noisy serial/UART lines.
- **`MSG_TYPE` (1 byte):** `0x01` Heartbeat, `0x02` Telemetry, `0x03` Command, `0x04` CommandAck.
- **`SEQUENCE_ID` (4 bytes):** Monotonically increasing packet counter for sequence gap tracking.
- **`TIMESTAMP_MS` (8 bytes):** Epoch timestamp in milliseconds.
- **`PAYLOAD_LEN` (2 bytes):** Explicitly bounded to a maximum ceiling of 1024 bytes.
- **`CRC-16-CCITT` (2 bytes):** Polynomial `0x1021` computed over the header and payload bytes.

---

## 3. Cryptographic Command Authentication

To prevent malicious command injection or unauthorized disarms over public RF frequencies:
- Commands are signed using **HMAC-SHA256** with a Pre-Shared Key (PSK).
- The full 256-bit signature is truncated to **16 bytes (128 bits)** to minimize RF datalink bandwidth overhead while maintaining cryptographic collision resistance.
- **Anti-Replay Window:** Commands older than 5,000ms or with forward clock drift exceeding 1,000ms are rejected.
- **Session Isolation:** `session_id` ensures that if a Ground Station reboots, previously captured commands cannot be replayed.
- **Constant-Time Verification:** Signatures are evaluated via constant-time byte comparisons to eliminate side-channel timing leaks.

---

## 4. Vehicle State Machine & Safety Invariants

The vehicle software enforces an explicit Deterministic Finite Automaton (DFA) avoiding any undefined state:

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
                 └────── (Touchdown Event) <────────── [FAILSAFE_RTL]
                                                              │
                                                      (Battery < 5%)
                                                              │
                                                              v
                                                      [EMERGENCY_LAND]
```

### Core Invariants Enforced:
1. **Airborne Disarm Prevention:** Disarm commands are strictly rejected from **any airborne state** (`InFlightManual`, `AutonomousMission`, `LostLinkHold`, `FailsafeRtl`, `EmergencyLand`). The vehicle can only disarm when `Armed` on the ground, or following an explicit `Touchdown` sensor event.
2. **Ground Hold Protection:** `HoldPosition` is strictly prohibited on the ground, ensuring an armed vehicle cannot inadvertently launch into manual flight without an explicit `Takeoff` command.
3. **Heartbeat Loss Watchdog:** If airborne and no valid heartbeat frame arrives for `> 3,000ms`, the state machine automatically transitions to `LOST_LINK_HOLD` (loitering or hovering in place).
4. **Failsafe Return-to-Launch (RTL):** If the radio link remains severed for `> 8,000ms`, the vehicle transitions to `FAILSAFE_RTL`, computing an autonomous return vector without operator intervention.
5. **Heartbeat Signal Recovery:** If radio contact is restored while in `LOST_LINK_HOLD`, the vehicle safely returns to flight control mode.
6. **Command Idempotency with Action Verification:** Bounded queue tracks executed `command_id`s. Duplicate IDs return `AlreadyExecuted`. Replaying the same ID with a *different* action returns `RejectedMalformed`.
7. **Battery Boundary Ceilings:** 
   - Arming and Takeoff are rejected if battery is below 15%.
   - Autonomous missions are rejected if battery is below 20%.
   - Below 10%, autonomous `FAILSAFE_RTL` triggers.
   - Below 5%, `EMERGENCY_LAND` engages immediately.

---

## 5. Verification & Test Suite

The codebase includes **29 automated tests** covering edge cases, malformed payloads, security attacks, and state transitions:

```bash
cargo test
```

### Key Test Categories:
- **Authentication & Replay:** Valid signature verification, forged payload detection, expired timestamp rejection, cross-session isolation.
- **Codec & Framing:** Encoding/decoding roundtrip, payload boundary limits (>1024B rejection), bit-flip corruption detection, sync magic validation, stream sync recovery.
- **Link Quality Estimation:** Sequence anomaly tracking (protecting against billion-packet poisoning attacks), `u32` sequence wraparound handling, duplicate deduplication.
- **State Machine Regressions:** Airborne disarm prevention during RTL, ground hold launch prevention, battery checks on takeoff, command ID action mismatch rejection, safe ground disarm on dead battery.

---

## 6. Running the Live Simulation

Execute the interactive end-to-end simulation showing authenticated command uplinks, bidirectional telemetry downlinks, RF packet drop handling, simulated radio jamming, and autonomous failsafe engagement:

```bash
cargo run --release
```

### Verified Terminal Output:
```text
========================================================================
   ARTEMIS OS // VEHICLE SYSTEMS & AUTONOMY TELEMETRY DAEMON            
   Authenticated Datalink, RF Monitor & Deterministic Failsafe FSM      
========================================================================

[INIT] Vehicle state: DISARMED
[INIT] Initial battery: 95%
[INIT] Security: HMAC-SHA256 message signing (128-bit truncated MAC, anti-replay)
[INIT] RF Profile: 15% uplink loss, 10% downlink loss, 3000ms watchdog timeout

>>> PHASE 1: Authenticated Command Uplink (GCS -> Vehicle)
    [CMD 101: Arm    ] Authenticated & Executed -> Result: Accepted
    [CMD 102: Takeoff { target_alt_m: 50.0 }] Authenticated & Executed -> Result: Accepted
    [CURRENT VEHICLE STATE] IN_FLIGHT_MANUAL

>>> PHASE 2: Live Flight Telemetry & Uplink/Downlink Exchanges
    [t=0001ms] State: IN_FLIGHT_MANUAL   Alt: 15.0m | Bat: 95% | Up-PDR: 100.0% | Down-PDR: 100.0%
    [t=0502ms] State: IN_FLIGHT_MANUAL   Alt: 20.0m | Bat: 94% | Up-PDR: 100.0% | Down-PDR: 100.0%
    [t=1004ms] State: IN_FLIGHT_MANUAL   Alt: 25.0m | Bat: 94% | Up-PDR: 100.0% | Down-PDR: 100.0%
    [t=1506ms] State: IN_FLIGHT_MANUAL   Alt: 30.0m | Bat: 93% | Up-PDR: 100.0% | Down-PDR: 100.0%
    [t=2007ms] State: IN_FLIGHT_MANUAL   Alt: 35.0m | Bat: 93% | Up-PDR: 100.0% | Down-PDR: 100.0%
    [t=2509ms] State: IN_FLIGHT_MANUAL   Alt: 40.0m | Bat: 92% | Up-PDR: 100.0% | Down-PDR:  83.3%
    [t=3010ms] State: IN_FLIGHT_MANUAL   Alt: 45.0m | Bat: 92% | Up-PDR: 100.0% | Down-PDR:  85.7%
    [t=3512ms] State: IN_FLIGHT_MANUAL   Alt: 50.0m | Bat: 91% | Up-PDR:  80.0% | Down-PDR:  87.5%

>>> PHASE 3: Complete Radio Blackout (Jamming / Severe Link Severance)
    [ALERT] Ground Station transmission ceased. Heartbeat watchdog monitoring...

    [t=5014ms] State: IN_FLIGHT_MANUAL   (Watchdog counting down...)
    [t=6016ms] State: IN_FLIGHT_MANUAL   (Watchdog counting down...)
    [WATCHDOG TRIGGER] State Transition: IN_FLIGHT_MANUAL -> LOST_LINK_HOLD
    [t=8019ms] State: LOST_LINK_HOLD     (Watchdog counting down...)
    [t=9020ms] State: LOST_LINK_HOLD     (Watchdog counting down...)
    [t=10022ms] State: LOST_LINK_HOLD     (Watchdog counting down...)
    [t=11022ms] State: LOST_LINK_HOLD     (Watchdog counting down...)
    [WATCHDOG TRIGGER] State Transition: LOST_LINK_HOLD -> FAILSAFE_RTL

    [AUTONOMOUS FAILSAFE ENGAGED] Vehicle computing autonomous Return-to-Launch vector.

========================================================================
   DATALINK DIAGNOSTICS & SYSTEM METRICS REPORT                        
========================================================================
   Uplink Frames Received   : 8
   Uplink Frames Dropped    : 2
   Uplink CRC Corruptions   : 1
   Uplink Packet Delivery   : 80.00%
   Downlink Packet Delivery : 87.50%
   Final Vehicle Safety State: FAILSAFE_RTL
========================================================================
```

---

## 7. Threat Model & Known Limitations

### Security Guarantees:
- **RF Packet Injection:** Mitigated via HMAC-SHA256 signatures. Packets without a valid key signature are rejected before reaching the vehicle state machine.
- **Replay Attacks:** Mitigated via timestamp freshness checks (5s window) and session isolation.
- **Random RF Corruptions:** Mitigated via CRC-16-CCITT checksum validation on every frame.
- **Airborne Cutoff:** Mitigated by strict state transition rules requiring verified touchdown before allowing motors to disarm.

### Known Limitations & Production Roadmap:
- **Simulated Transport:** The current prototype demonstrates datalink handling via an in-process simulated lossy channel (`LossyChannel`). Production Artemis integration maps these frames to physical serial ports (`/dev/ttyUSB*` / `/dev/ttyTHS*`) or UDP sockets.
- **Payload Serialization:** JSON serialization is currently used inside payloads for human readability during demonstration. Production avionics should migrate to packed binary structs or CBOR to minimize over-the-air byte footprint.
- **Key Rotation:** The prototype uses a pre-shared static key (PSK). Production fleets should implement asymmetric ECDH key exchange or hardware TPM-backed key derivation per mission flight.

---

## Author

**Olumide Lala**  
- GitHub: [@Olalolo22](https://github.com/Olalolo22)  
- Engineering Dossier: [portfolio-research](https://github.com/Olalolo22/portfolio-research)
