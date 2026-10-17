//! Strongly-typed telemetry, heartbeat, and command payloads for Artemis vehicle systems.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Heartbeat {
    pub vehicle_id: u32,
    pub uptime_secs: u64,
    pub armed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Telemetry {
    pub vehicle_id: u32,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub altitude_m: f32,
    pub ground_speed_mps: f32,
    pub battery_mv: u16,
    pub battery_pct: u8,
    pub rssi_dbm: i8,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum VehicleAction {
    Arm,
    Disarm,
    Takeoff { target_alt_m: f32 },
    ExecuteWaypoint { waypoint_id: u32, lat: f64, lon: f64, alt_m: f32 },
    HoldPosition,
    ReturnToLaunch,
    EmergencyLand,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Command {
    pub command_id: u32,
    pub action: VehicleAction,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum AckStatus {
    Accepted,
    RejectedInvalidState(String),
    RejectedLowBattery,
    AlreadyExecuted,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct CommandAck {
    pub command_id: u32,
    pub status: AckStatus,
}

impl Heartbeat {
    pub fn to_bytes(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec(self)
    }

    pub fn from_bytes(slice: &[u8]) -> Result<Self, serde_json::Error> {
        serde_json::from_slice(slice)
    }
}

impl Telemetry {
    pub fn to_bytes(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec(self)
    }

    pub fn from_bytes(slice: &[u8]) -> Result<Self, serde_json::Error> {
        serde_json::from_slice(slice)
    }
}

impl Command {
    pub fn to_bytes(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec(self)
    }

    pub fn from_bytes(slice: &[u8]) -> Result<Self, serde_json::Error> {
        serde_json::from_slice(slice)
    }
}

impl CommandAck {
    pub fn to_bytes(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec(self)
    }

    pub fn from_bytes(slice: &[u8]) -> Result<Self, serde_json::Error> {
        serde_json::from_slice(slice)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_telemetry_serde_roundtrip() {
        let telem = Telemetry {
            vehicle_id: 101,
            roll_deg: 1.25,
            pitch_deg: -0.84,
            yaw_deg: 180.5,
            altitude_m: 120.4,
            ground_speed_mps: 18.2,
            battery_mv: 24600,
            battery_pct: 88,
            rssi_dbm: -65,
        };

        let bytes = telem.to_bytes().expect("Serialization failed");
        let parsed = Telemetry::from_bytes(&bytes).expect("Deserialization failed");
        assert_eq!(telem, parsed);
    }

    #[test]
    fn test_command_serde_roundtrip() {
        let cmd = Command {
            command_id: 991,
            action: VehicleAction::ExecuteWaypoint {
                waypoint_id: 4,
                lat: 9.0765,
                lon: 7.3986,
                alt_m: 150.0,
            },
        };

        let bytes = cmd.to_bytes().expect("Serialization failed");
        let parsed = Command::from_bytes(&bytes).expect("Deserialization failed");
        assert_eq!(cmd, parsed);
    }
}
