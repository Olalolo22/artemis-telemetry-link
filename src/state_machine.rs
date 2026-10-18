//! Deterministic vehicle state machine, safety transition guards, and idempotent command processor.

use crate::messages::{AckStatus, Command, CommandAck, VehicleAction};
use std::collections::HashMap;
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

impl VehicleState {
    pub fn is_airborne(&self) -> bool {
        matches!(
            self,
            VehicleState::InFlightManual
                | VehicleState::AutonomousMission
                | VehicleState::LostLinkHold
                | VehicleState::FailsafeRtl
                | VehicleState::EmergencyLand
        )
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
    // Maps command_id -> serialized representation of action to detect payload mismatch on ID replay
    executed_commands: HashMap<u32, String>,
    recent_command_queue: Vec<u32>,
    max_command_history: usize,
    pre_loss_state: Option<VehicleState>,
}

impl VehicleStateMachine {
    pub fn new(initial_battery_pct: u8, start_time_ms: u64) -> Self {
        Self {
            current_state: VehicleState::Disarmed,
            battery_pct: initial_battery_pct.min(100),
            last_valid_heartbeat_ms: start_time_ms,
            executed_commands: HashMap::new(),
            recent_command_queue: Vec::new(),
            max_command_history: 128,
            pre_loss_state: None,
        }
    }

    pub fn state(&self) -> VehicleState {
        self.current_state
    }

    pub fn battery_pct(&self) -> u8 {
        self.battery_pct
    }

    /// Updates vehicle battery percentage with strict failsafe state transitions.
    pub fn update_battery(&mut self, pct: u8) -> Result<(), TransitionError> {
        self.battery_pct = pct.min(100);

        if self.battery_pct <= EMERGENCY_BATTERY_LAND_PCT {
            if self.current_state.is_airborne() && self.current_state != VehicleState::EmergencyLand {
                self.transition_to(VehicleState::EmergencyLand)?;
            } else if self.current_state == VehicleState::Armed {
                // If on ground and battery dies, disarm immediately to prevent accident
                self.transition_to(VehicleState::Disarmed)?;
            }
        } else if self.battery_pct <= CRITICAL_BATTERY_RTL_PCT {
            if self.current_state.is_airborne()
                && self.current_state != VehicleState::FailsafeRtl
                && self.current_state != VehicleState::EmergencyLand
            {
                self.transition_to(VehicleState::FailsafeRtl)?;
            } else if self.current_state == VehicleState::Armed {
                self.transition_to(VehicleState::Disarmed)?;
            }
        }
        Ok(())
    }

    /// Records a valid incoming heartbeat timestamp. Ignores stale / backward-moving timestamps.
    /// If recovering from LostLinkHold, restores the operational mode (InFlightManual or AutonomousMission)
    /// that was active prior to link loss.
    pub fn record_heartbeat(&mut self, timestamp_ms: u64) {
        if timestamp_ms >= self.last_valid_heartbeat_ms {
            self.last_valid_heartbeat_ms = timestamp_ms;
            if self.current_state == VehicleState::LostLinkHold {
                let target_mode = self.pre_loss_state.take().unwrap_or(VehicleState::InFlightManual);
                let _ = self.transition_to(target_mode);
            }
        }
    }

    /// Evaluates temporal timeouts and watchdog rules.
    pub fn evaluate_watchdog(&mut self, now_ms: u64) -> Option<VehicleState> {
        let elapsed = now_ms.saturating_sub(self.last_valid_heartbeat_ms);

        match self.current_state {
            VehicleState::InFlightManual | VehicleState::AutonomousMission => {
                if elapsed >= HEARTBEAT_TIMEOUT_MS {
                    self.pre_loss_state = Some(self.current_state);
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

    /// Processes an incoming command with parameter validation, idempotency, and safety guards.
    pub fn process_command(&mut self, cmd: Command) -> CommandAck {
        // 1. Parameter boundary validation
        if let Err(val_err) = cmd.action.validate() {
            return CommandAck {
                command_id: cmd.command_id,
                status: AckStatus::RejectedMalformed(val_err.to_string()),
            };
        }

        // 2. Command Idempotency with Action Integrity check
        let action_repr = format!("{:?}", cmd.action);
        if let Some(recorded_action) = self.executed_commands.get(&cmd.command_id) {
            if recorded_action == &action_repr {
                return CommandAck {
                    command_id: cmd.command_id,
                    status: AckStatus::AlreadyExecuted,
                };
            } else {
                return CommandAck {
                    command_id: cmd.command_id,
                    status: AckStatus::RejectedMalformed(format!(
                        "Command ID {} reused with mismatched action: was {}, requested {}",
                        cmd.command_id, recorded_action, action_repr
                    )),
                };
            }
        }

        // 3. State transition execution
        let transition_result = match cmd.action {
            VehicleAction::Arm => {
                if self.current_state != VehicleState::Disarmed {
                    Err(TransitionError::SafetyPreconditionFailed(
                        format!("Cannot arm vehicle when currently in state {}", self.current_state)
                    ))
                } else if self.battery_pct < MIN_BATTERY_ARM_PCT {
                    Err(TransitionError::InsufficientBattery {
                        required: MIN_BATTERY_ARM_PCT,
                        current: self.battery_pct,
                    })
                } else {
                    self.transition_to(VehicleState::Armed)
                }
            }
            VehicleAction::Disarm => {
                // Bug fix: Strictly prohibit Disarm from ANY airborne state (Manual, Mission, Hold, RTL, EmergencyLand)
                if self.current_state.is_airborne() {
                    Err(TransitionError::SafetyPreconditionFailed(
                        format!("Cannot disarm vehicle while airborne in state {}", self.current_state)
                    ))
                } else {
                    self.transition_to(VehicleState::Disarmed)
                }
            }
            VehicleAction::Takeoff { .. } => {
                if self.current_state != VehicleState::Armed {
                    Err(TransitionError::SafetyPreconditionFailed(
                        format!("Takeoff requires vehicle to be in Armed state (currently {})", self.current_state)
                    ))
                } else if self.battery_pct < MIN_BATTERY_ARM_PCT {
                    Err(TransitionError::InsufficientBattery {
                        required: MIN_BATTERY_ARM_PCT,
                        current: self.battery_pct,
                    })
                } else {
                    self.transition_to(VehicleState::InFlightManual)
                }
            }
            VehicleAction::ExecuteWaypoint { .. } => {
                if !self.current_state.is_airborne() {
                    Err(TransitionError::SafetyPreconditionFailed(
                        "Cannot execute waypoint while vehicle is on the ground".to_string()
                    ))
                } else if self.battery_pct < MIN_BATTERY_MISSION_PCT {
                    Err(TransitionError::InsufficientBattery {
                        required: MIN_BATTERY_MISSION_PCT,
                        current: self.battery_pct,
                    })
                } else {
                    self.transition_to(VehicleState::AutonomousMission)
                }
            }
            VehicleAction::HoldPosition => {
                // Bug fix: HoldPosition must NEVER launch an armed vehicle on the ground!
                if !self.current_state.is_airborne() {
                    Err(TransitionError::SafetyPreconditionFailed(
                        "HoldPosition is only valid when vehicle is airborne".to_string()
                    ))
                } else {
                    self.transition_to(VehicleState::InFlightManual)
                }
            }
            VehicleAction::ReturnToLaunch => {
                if !self.current_state.is_airborne() {
                    Err(TransitionError::SafetyPreconditionFailed(
                        "ReturnToLaunch is only valid when vehicle is airborne".to_string()
                    ))
                } else {
                    self.transition_to(VehicleState::FailsafeRtl)
                }
            }
            VehicleAction::EmergencyLand => {
                if !self.current_state.is_airborne() {
                    Err(TransitionError::SafetyPreconditionFailed(
                        "EmergencyLand is only valid when vehicle is airborne".to_string()
                    ))
                } else {
                    self.transition_to(VehicleState::EmergencyLand)
                }
            }
            VehicleAction::Touchdown => {
                // Bug fix: Explicit touchdown event transition from landing states to Disarmed
                if self.current_state == VehicleState::EmergencyLand
                    || self.current_state == VehicleState::FailsafeRtl
                    || self.current_state == VehicleState::InFlightManual
                {
                    self.transition_to(VehicleState::Disarmed)
                } else {
                    Err(TransitionError::SafetyPreconditionFailed(
                        format!("Touchdown event not applicable in state {}", self.current_state)
                    ))
                }
            }
        };

        match transition_result {
            Ok(_) => {
                self.record_command_id(cmd.command_id, action_repr);
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

    fn record_command_id(&mut self, id: u32, action_repr: String) {
        if self.recent_command_queue.len() >= self.max_command_history {
            if let Some(oldest) = self.recent_command_queue.first().copied() {
                self.executed_commands.remove(&oldest);
                self.recent_command_queue.remove(0);
            }
        }
        self.executed_commands.insert(id, action_repr);
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

            // LostLinkHold -> InFlightManual / AutonomousMission (Link recovery) or FailsafeRtl (Timeout)
            (VehicleState::LostLinkHold, VehicleState::InFlightManual) => true,
            (VehicleState::LostLinkHold, VehicleState::AutonomousMission) => true,
            (VehicleState::LostLinkHold, VehicleState::FailsafeRtl) => true,

            // Airborne states can enter RTL or EmergencyLand
            (VehicleState::InFlightManual, VehicleState::FailsafeRtl) => true,
            (VehicleState::AutonomousMission, VehicleState::FailsafeRtl) => true,
            (VehicleState::InFlightManual, VehicleState::EmergencyLand) => true,
            (VehicleState::AutonomousMission, VehicleState::EmergencyLand) => true,
            (VehicleState::LostLinkHold, VehicleState::EmergencyLand) => true,
            (VehicleState::FailsafeRtl, VehicleState::EmergencyLand) => true,

            // Touchdown transitions to Disarmed
            (VehicleState::EmergencyLand, VehicleState::Disarmed) => true,
            (VehicleState::FailsafeRtl, VehicleState::Disarmed) => true,
            (VehicleState::InFlightManual, VehicleState::Disarmed) => true,

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
    fn test_regression_strictly_prevents_disarm_during_rtl_and_emergency_land() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        sm.process_command(Command { command_id: 1, action: VehicleAction::Arm });
        sm.process_command(Command { command_id: 2, action: VehicleAction::Takeoff { target_alt_m: 50.0 } });
        sm.process_command(Command { command_id: 3, action: VehicleAction::ReturnToLaunch });
        assert_eq!(sm.state(), VehicleState::FailsafeRtl);

        // Attempting to disarm mid-air during RTL MUST fail
        let ack = sm.process_command(Command { command_id: 4, action: VehicleAction::Disarm });
        assert!(matches!(ack.status, AckStatus::RejectedInvalidState(_)));
        assert_eq!(sm.state(), VehicleState::FailsafeRtl);

        // Transition to EmergencyLand
        sm.process_command(Command { command_id: 5, action: VehicleAction::EmergencyLand });
        assert_eq!(sm.state(), VehicleState::EmergencyLand);

        // Attempting to disarm during EmergencyLand MUST fail
        let ack2 = sm.process_command(Command { command_id: 6, action: VehicleAction::Disarm });
        assert!(matches!(ack2.status, AckStatus::RejectedInvalidState(_)));
        assert_eq!(sm.state(), VehicleState::EmergencyLand);

        // Sensor touchdown event safely disarms
        let ack3 = sm.process_command(Command { command_id: 7, action: VehicleAction::Touchdown });
        assert_eq!(ack3.status, AckStatus::Accepted);
        assert_eq!(sm.state(), VehicleState::Disarmed);
    }

    #[test]
    fn test_regression_hold_position_cannot_launch_from_ground() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        sm.process_command(Command { command_id: 1, action: VehicleAction::Arm });
        assert_eq!(sm.state(), VehicleState::Armed);

        // HoldPosition on ground must be rejected
        let ack = sm.process_command(Command { command_id: 2, action: VehicleAction::HoldPosition });
        assert!(matches!(ack.status, AckStatus::RejectedInvalidState(_)));
        assert_eq!(sm.state(), VehicleState::Armed);
    }

    #[test]
    fn test_regression_takeoff_checks_battery() {
        let mut sm = VehicleStateMachine::new(12, 1000); // 12% is below minimum arm/takeoff 15%
        let ack = sm.process_command(Command { command_id: 1, action: VehicleAction::Arm });
        assert_eq!(ack.status, AckStatus::RejectedLowBattery);
    }

    #[test]
    fn test_regression_idempotent_command_detects_action_mismatch() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        let cmd1 = Command { command_id: 42, action: VehicleAction::Arm };
        let ack1 = sm.process_command(cmd1);
        assert_eq!(ack1.status, AckStatus::Accepted);

        // Same ID with identical action -> AlreadyExecuted
        let ack2 = sm.process_command(Command { command_id: 42, action: VehicleAction::Arm });
        assert_eq!(ack2.status, AckStatus::AlreadyExecuted);

        // Same ID with DIFFERENT action -> RejectedMalformed
        let ack3 = sm.process_command(Command { command_id: 42, action: VehicleAction::Disarm });
        assert!(matches!(ack3.status, AckStatus::RejectedMalformed(_)));
    }

    #[test]
    fn test_regression_heartbeat_timestamp_ordering() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        sm.record_heartbeat(5000);
        assert_eq!(sm.last_valid_heartbeat_ms, 5000);

        // Stale heartbeat with old timestamp (3000ms) should be ignored
        sm.record_heartbeat(3000);
        assert_eq!(sm.last_valid_heartbeat_ms, 5000);
    }

    #[test]
    fn test_regression_low_battery_on_ground_disarms_cleanly() {
        let mut sm = VehicleStateMachine::new(50, 1000);
        sm.process_command(Command { command_id: 1, action: VehicleAction::Arm });
        assert_eq!(sm.state(), VehicleState::Armed);

        // Battery drops to critical while on ground -> safe disarm, not flight RTL
        let res = sm.update_battery(4);
        assert!(res.is_ok());
        assert_eq!(sm.state(), VehicleState::Disarmed);
    }

    #[test]
    fn test_regression_link_recovery_restores_autonomous_mission() {
        let mut sm = VehicleStateMachine::new(80, 1000);
        sm.process_command(Command { command_id: 1, action: VehicleAction::Arm });
        sm.process_command(Command { command_id: 2, action: VehicleAction::Takeoff { target_alt_m: 50.0 } });
        sm.process_command(Command {
            command_id: 3,
            action: VehicleAction::ExecuteWaypoint {
                waypoint_id: 1,
                lat: 9.0765,
                lon: 7.3986,
                alt_m: 100.0,
            },
        });
        assert_eq!(sm.state(), VehicleState::AutonomousMission);

        // Heartbeat timeout occurs -> enters LostLinkHold
        let event = sm.evaluate_watchdog(4500);
        assert_eq!(event, Some(VehicleState::LostLinkHold));
        assert_eq!(sm.state(), VehicleState::LostLinkHold);

        // Ground station signal restored at t=5000ms -> restores AutonomousMission!
        sm.record_heartbeat(5000);
        assert_eq!(sm.state(), VehicleState::AutonomousMission);
    }
}
