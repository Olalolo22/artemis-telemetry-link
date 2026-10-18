//! Strongly-typed telemetry, heartbeat, and authenticated command payloads for Artemis vehicle systems.

use crate::auth::MAC_LEN;
use serde::{Deserialize, Serialize};
use std::fmt;

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
    Touchdown, // Hardware/sensor touchdown event confirming vehicle is on the ground
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Command {
    pub command_id: u32,
    pub action: VehicleAction,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SignedCommand {
    pub session_id: u32,
    pub timestamp_ms: u64,
    pub command: Command,
    pub signature: [u8; MAC_LEN],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum AckStatus {
    Accepted,
    RejectedInvalidState(String),
    RejectedLowBattery,
    RejectedMalformed(String),
    RejectedUnauthorized(String),
    AlreadyExecuted,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct CommandAck {
    pub command_id: u32,
    pub status: AckStatus,
}

#[derive(Debug, PartialEq)]
pub enum ValidationError {
    InvalidAltitude { alt: f32, min: f32, max: f32 },
    InvalidLatitude { lat: f64 },
    InvalidLongitude { lon: f64 },
    InvalidBatteryPercentage { pct: u8 },
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ValidationError::InvalidAltitude { alt, min, max } => {
                write!(f, "Invalid altitude {}m: must be between {}m and {}m", alt, min, max)
            }
            ValidationError::InvalidLatitude { lat } => {
                write!(f, "Invalid latitude {}: must be between -90.0 and 90.0 degrees", lat)
            }
            ValidationError::InvalidLongitude { lon } => {
                write!(f, "Invalid longitude {}: must be between -180.0 and 180.0 degrees", lon)
            }
            ValidationError::InvalidBatteryPercentage { pct } => {
                write!(f, "Invalid battery percentage {}: must be between 0 and 100", pct)
            }
        }
    }
}

impl std::error::Error for ValidationError {}

impl VehicleAction {
    /// Validates safety boundaries for command parameters (geofencing and altitude ceilings).
    pub fn validate(&self) -> Result<(), ValidationError> {
        match self {
            VehicleAction::Takeoff { target_alt_m } => {
                if *target_alt_m < 1.0 || *target_alt_m > 3000.0 {
                    return Err(ValidationError::InvalidAltitude {
                        alt: *target_alt_m,
                        min: 1.0,
                        max: 3000.0,
                    });
                }
            }
            VehicleAction::ExecuteWaypoint { lat, lon, alt_m, .. } => {
                if *lat < -90.0 || *lat > 90.0 {
                    return Err(ValidationError::InvalidLatitude { lat: *lat });
                }
                if *lon < -180.0 || *lon > 180.0 {
                    return Err(ValidationError::InvalidLongitude { lon: *lon });
                }
                if *alt_m < 1.0 || *alt_m > 3000.0 {
                    return Err(ValidationError::InvalidAltitude {
                        alt: *alt_m,
                        min: 1.0,
                        max: 3000.0,
                    });
                }
            }
            _ => {}
        }
        Ok(())
    }
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

impl SignedCommand {
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
    fn test_validates_safe_takeoff_altitude() {
        let valid = VehicleAction::Takeoff { target_alt_m: 50.0 };
        assert!(valid.validate().is_ok());

        let negative = VehicleAction::Takeoff { target_alt_m: -10.0 };
        assert!(negative.validate().is_err());

        let stratospheric = VehicleAction::Takeoff { target_alt_m: 10_000.0 };
        assert!(stratospheric.validate().is_err());
    }

    #[test]
    fn test_validates_waypoint_coordinates() {
        let valid_wp = VehicleAction::ExecuteWaypoint {
            waypoint_id: 1,
            lat: 9.0765,
            lon: 7.3986,
            alt_m: 100.0,
        };
        assert!(valid_wp.validate().is_ok());

        let invalid_lat = VehicleAction::ExecuteWaypoint {
            waypoint_id: 2,
            lat: 99.9,
            lon: 7.3986,
            alt_m: 100.0,
        };
        assert!(invalid_lat.validate().is_err());
    }
}
