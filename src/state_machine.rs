//! Deterministic vehicle state machine, safety transition guards, and idempotent command processor.

use crate::messages::{AckStatus, Command, CommandAck, VehicleAction};
use std::collections::HashSet;
use std::fmt;

pub const HEARTBEAT_TIMEOUT_MS: u64 = 3_000;
pub const LINK_RECOVERY_EXPIRED_MS: u64 = 8_000;
pub const MIN_BATTERY_ARM_PCT: u8 = 15;
pub const MIN_BATTERY_MISSION_PCT: u8 = 20;
pub const CRITICAL_BATTERY_RTL_PCT: u8 = 10;
pub const EMERGENCY_BATTERY_LAND_PCT: u8 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleState {
    Boot,
    Disarmed,
    Armed,
    InFlightManual,
    AutonomousMission,
    LostLinkHold,
    FailsafeRtl,
    EmergencyLand,
}

impl fmt::Display for VehicleState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            VehicleState::Boot => write!(f, "BOOT"),
            VehicleState::Disarmed => write!(f, "DISARMED"),
            VehicleState::Armed => write!(f, "ARMED"),
            VehicleState::InFlightManual => write!(f, "IN_FLIGHT_MANUAL"),
            VehicleState::AutonomousMission => write!(f, "AUTONOMOUS_MISSION"),
            VehicleState::LostLinkHold => write!(f, "LOST_LINK_HOLD"),
            VehicleState::FailsafeRtl => write!(f, "FAILSAFE_RTL"),
            VehicleState::EmergencyLand => write!(f, "EMERGENCY_LAND"),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransitionError {
    InvalidTransition { from: VehicleState, to: VehicleState },
    InsufficientBattery { required: u8, current: u8 },
    SafetyPreconditionFailed(String),
}

impl fmt::Display for TransitionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransitionError::InvalidTransition { from, to } => {
                write!(f, "Illegal state transition from {} to {}", from, to)
            }
            TransitionError::InsufficientBattery { required, current } => {
                write!(f, "Insufficient battery: required {}%, current {}%", required, current)
            }
            TransitionError::SafetyPreconditionFailed(reason) => {
                write!(f, "Safety guard prevented transition: {}", reason)
            }
        }
    }
}

impl std::error::Error for TransitionError {}

#[derive(Debug)]
pub struct VehicleStateMachine {
    current_state: VehicleState,
    battery_pct: u8,
    last_valid_heartbeat_ms: u64,
    executed_commands: HashSet<u32>,
    recent_command_queue: Vec<u32>,
    max_command_history: usize,
}

impl VehicleStateMachine {
    pub fn new(initial_battery_pct: u8, start_time_ms: u64) -> Self {
        Self {
            current_state: VehicleState::Disarmed,
            battery_pct: initial_battery_pct,
            last_valid_heartbeat_ms: start_time_ms,
            executed_commands: HashSet::new(),
            recent_command_queue: Vec::new(),
            max_command_history: 128,
        }
    }

    pub fn state(&self) -> VehicleState {
        self.current_state
    }

    pub fn battery_pct(&self) -> u8 {
        self.battery_pct
    }

    pub fn update_battery(&mut self, pct: u8) {
        self.battery_pct = pct;
        if self.battery_pct <= EMERGENCY_BATTERY_LAND_PCT
            && self.current_state != VehicleState::EmergencyLand
            && self.current_state != VehicleState::Disarmed
        {
            let _ = self.transition_to(VehicleState::EmergencyLand);
        } else if self.battery_pct <= CRITICAL_BATTERY_RTL_PCT
            && self.current_state != VehicleState::FailsafeRtl
            && self.current_state != VehicleState::EmergencyLand
            && self.current_state != VehicleState::Disarmed
        {
            let _ = self.transition_to(VehicleState::FailsafeRtl);
        }
    }

    pub fn record_heartbeat(&mut self, timestamp_ms: u64) {
        self.last_valid_heartbeat_ms = timestamp_ms;
        // If we were holding due to link loss and link recovers, recover to manual flight
        if self.current_state == VehicleState::LostLinkHold {
            let _ = self.transition_to(VehicleState::InFlightManual);
        }
    }

    /// Evaluates temporal timeouts and watchdog rules.
    pub fn evaluate_watchdog(&mut self, now_ms: u64) -> Option<VehicleState> {
        let elapsed = now_ms.saturating_sub(self.last_valid_heartbeat_ms);

        match self.current_state {
            VehicleState::InFlightManual | VehicleState::AutonomousMission => {
                if elapsed >= HEARTBEAT_TIMEOUT_MS {
                    let _ = self.transition_to(VehicleState::LostLinkHold);
                    return Some(self.current_state);
                }
            }
            VehicleState::LostLinkHold => {
                if elapsed >= LINK_RECOVERY_EXPIRED_MS {
                    let _ = self.transition_to(VehicleState::FailsafeRtl);
                    return Some(self.current_state);
                }
            }
            _ => {}
        }
        None
    }

    /// Processes an incoming command with strict idempotency and state transition guards.
    pub fn process_command(&mut self, cmd: Command) -> CommandAck {
        // Idempotency check: if already executed, return cached success without duplicate side-effects
        if self.executed_commands.contains(&cmd.command_id) {
            return CommandAck {
                command_id: cmd.command_id,
                status: AckStatus::AlreadyExecuted,
            };
        }

        let transition_result = match cmd.action {
            VehicleAction::Arm => {
                if self.battery_pct < MIN_BATTERY_ARM_PCT {
                    Err(TransitionError::InsufficientBattery {
                        required: MIN_BATTERY_ARM_PCT,
                        current: self.battery_pct,
                    })
                } else {
                    self.transition_to(VehicleState::Armed)
                }
            }
            VehicleAction::Disarm => {
                if self.current_state == VehicleState::InFlightManual
                    || self.current_state == VehicleState::AutonomousMission
                {
                    Err(TransitionError::SafetyPreconditionFailed(
                        "Cannot disarm while vehicle is airborne".to_string(),
                    ))
                } else {
                    self.transition_to(VehicleState::Disarmed)
                }
            }
            VehicleAction::Takeoff { .. } => self.transition_to(VehicleState::InFlightManual),
            VehicleAction::ExecuteWaypoint { .. } => {
                if self.battery_pct < MIN_BATTERY_MISSION_PCT {
                    Err(TransitionError::InsufficientBattery {
                        required: MIN_BATTERY_MISSION_PCT,
                        current: self.battery_pct,
                    })
                } else {
                    self.transition_to(VehicleState::AutonomousMission)
                }
            }
            VehicleAction::HoldPosition => self.transition_to(VehicleState::InFlightManual),
            VehicleAction::ReturnToLaunch => self.transition_to(VehicleState::FailsafeRtl),
            VehicleAction::EmergencyLand => self.transition_to(VehicleState::EmergencyLand),
        };

        match transition_result {
            Ok(_) => {
                self.record_command_id(cmd.command_id);
                CommandAck {
                    command_id: cmd.command_id,
                    status: AckStatus::Accepted,
                }
            }
            Err(TransitionError::InsufficientBattery { .. }) => CommandAck {
                command_id: cmd.command_id,
                status: AckStatus::RejectedLowBattery,
            },
            Err(err) => CommandAck {
                command_id: cmd.command_id,
                status: AckStatus::RejectedInvalidState(err.to_string()),
            },
        }
    }

    fn record_command_id(&mut self, id: u32) {
        if self.recent_command_queue.len() >= self.max_command_history {
            if let Some(oldest) = self.recent_command_queue.first().copied() {
                self.executed_commands.remove(&oldest);
                self.recent_command_queue.remove(0);
            }
        }
        self.executed_commands.insert(id);
        self.recent_command_queue.push(id);
    }

    fn transition_to(&mut self, next: VehicleState) -> Result<(), TransitionError> {
        let current = self.current_state;
        let is_valid = match (current, next) {
            // Self transitions are no-ops
            (a, b) if a == b => true,

            // Disarmed -> Armed
            (VehicleState::Disarmed, VehicleState::Armed) => true,

            // Armed -> Disarmed or InFlightManual
            (VehicleState::Armed, VehicleState::Disarmed) => true,
            (VehicleState::Armed, VehicleState::InFlightManual) => true,

            // InFlightManual <-> AutonomousMission
            (VehicleState::InFlightManual, VehicleState::AutonomousMission) => true,
            (VehicleState::AutonomousMission, VehicleState::InFlightManual) => true,

            // Airborne states -> LostLinkHold
            (VehicleState::InFlightManual, VehicleState::LostLinkHold) => true,
            (VehicleState::AutonomousMission, VehicleState::LostLinkHold) => true,

            // LostLinkHold -> InFlightManual (Link recovery) or FailsafeRtl (Timeout)
            (VehicleState::LostLinkHold, VehicleState::InFlightManual) => true,
            (VehicleState::LostLinkHold, VehicleState::FailsafeRtl) => true,

            // Failsafe RTL -> EmergencyLand or Disarmed (upon landing)
            (VehicleState::FailsafeRtl, VehicleState::EmergencyLand) => true,
            (VehicleState::FailsafeRtl, VehicleState::Disarmed) => true,

            // Any airborne state can execute emergency RTL or Land
            (VehicleState::InFlightManual, VehicleState::FailsafeRtl) => true,
            (VehicleState::AutonomousMission, VehicleState::FailsafeRtl) => true,
            (VehicleState::InFlightManual, VehicleState::EmergencyLand) => true,
            (VehicleState::AutonomousMission, VehicleState::EmergencyLand) => true,
            (VehicleState::LostLinkHold, VehicleState::EmergencyLand) => true,

            // EmergencyLand completes by disarming on touchdown
            (VehicleState::EmergencyLand, VehicleState::Disarmed) => true,

            _ => false,
        };

        if is_valid {
            self.current_state = next;
            Ok(())
        } else {
            Err(TransitionError::InvalidTransition {
                from: current,
                to: next,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_arm_and_takeoff_sequence() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        assert_eq!(sm.state(), VehicleState::Disarmed);

        let ack = sm.process_command(Command {
            command_id: 1,
            action: VehicleAction::Arm,
        });
        assert_eq!(ack.status, AckStatus::Accepted);
        assert_eq!(sm.state(), VehicleState::Armed);

        let ack = sm.process_command(Command {
            command_id: 2,
            action: VehicleAction::Takeoff { target_alt_m: 50.0 },
        });
        assert_eq!(ack.status, AckStatus::Accepted);
        assert_eq!(sm.state(), VehicleState::InFlightManual);
    }

    #[test]
    fn test_prevents_disarm_while_airborne() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        sm.process_command(Command {
            command_id: 1,
            action: VehicleAction::Arm,
        });
        sm.process_command(Command {
            command_id: 2,
            action: VehicleAction::Takeoff { target_alt_m: 50.0 },
        });

        // Attempt airborne disarm
        let ack = sm.process_command(Command {
            command_id: 3,
            action: VehicleAction::Disarm,
        });
        match ack.status {
            AckStatus::RejectedInvalidState(msg) => {
                assert!(msg.contains("Cannot disarm while vehicle is airborne"));
            }
            other => panic!("Expected RejectedInvalidState, got {:?}", other),
        }
        assert_eq!(sm.state(), VehicleState::InFlightManual);
    }

    #[test]
    fn test_idempotent_command_replay() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        let cmd = Command {
            command_id: 42,
            action: VehicleAction::Arm,
        };

        let ack1 = sm.process_command(cmd.clone());
        assert_eq!(ack1.status, AckStatus::Accepted);

        // Ground station retransmits due to lost ACK packet
        let ack2 = sm.process_command(cmd);
        assert_eq!(ack2.status, AckStatus::AlreadyExecuted);
    }

    #[test]
    fn test_watchdog_lost_link_and_failsafe_rtl() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        sm.process_command(Command {
            command_id: 1,
            action: VehicleAction::Arm,
        });
        sm.process_command(Command {
            command_id: 2,
            action: VehicleAction::Takeoff { target_alt_m: 50.0 },
        });

        // Advance clock by 3.5 seconds without heartbeat -> triggers LostLinkHold
        let event = sm.evaluate_watchdog(4500);
        assert_eq!(event, Some(VehicleState::LostLinkHold));
        assert_eq!(sm.state(), VehicleState::LostLinkHold);

        // Advance clock past 8 seconds -> triggers FailsafeRtl
        let event2 = sm.evaluate_watchdog(9500);
        assert_eq!(event2, Some(VehicleState::FailsafeRtl));
        assert_eq!(sm.state(), VehicleState::FailsafeRtl);
    }

    #[test]
    fn test_heartbeat_recovers_from_lost_link_hold() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        sm.process_command(Command {
            command_id: 1,
            action: VehicleAction::Arm,
        });
        sm.process_command(Command {
            command_id: 2,
            action: VehicleAction::Takeoff { target_alt_m: 50.0 },
        });

        // Trip link loss
        sm.evaluate_watchdog(4500);
        assert_eq!(sm.state(), VehicleState::LostLinkHold);

        // Ground station signal restored at t=5000ms
        sm.record_heartbeat(5000);
        assert_eq!(sm.state(), VehicleState::InFlightManual);
    }

    #[test]
    fn test_low_battery_rejection_and_emergency_land() {
        let mut sm = VehicleStateMachine::new(10, 1000);
        let ack = sm.process_command(Command {
            command_id: 1,
            action: VehicleAction::Arm,
        });
        assert_eq!(ack.status, AckStatus::RejectedLowBattery);

        // In flight, drops to 4% -> EmergencyLand
        let mut sm_airborne = VehicleStateMachine::new(80, 1000);
        sm_airborne.process_command(Command {
            command_id: 1,
            action: VehicleAction::Arm,
        });
        sm_airborne.process_command(Command {
            command_id: 2,
            action: VehicleAction::Takeoff { target_alt_m: 50.0 },
        });

        sm_airborne.update_battery(4);
        assert_eq!(sm_airborne.state(), VehicleState::EmergencyLand);
    }
}
