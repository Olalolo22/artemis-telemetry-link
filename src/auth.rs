//! Cryptographic command authentication and anti-replay engine using HMAC-SHA256.
//!
//! Protects vehicle control links against RF command injection, spoofing, and replay attacks.
//! Models MAVLink 2 message signing with 128-bit truncated MACs and temporal replay windows.

use hmac::{Hmac, Mac};
use sha2::Sha256;
use std::fmt;

type HmacSha256 = Hmac<Sha256>;

pub const MAC_LEN: usize = 16;
pub const MAX_REPLAY_AGE_MS: u64 = 5_000;
pub const MAX_CLOCK_DRIFT_MS: u64 = 1_000;

#[derive(Debug, PartialEq, Eq)]
pub enum AuthError {
    SignatureMismatch,
    ExpiredTimestamp { age_ms: u64, max_ms: u64 },
    FutureTimestamp { future_ms: u64, max_ms: u64 },
    InvalidKeyLength,
}

impl fmt::Display for AuthError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AuthError::SignatureMismatch => write!(f, "HMAC-SHA256 signature verification failed (unauthorized command)"),
            AuthError::ExpiredTimestamp { age_ms, max_ms } => {
                write!(f, "Command timestamp expired: {}ms old (max allowed {}ms)", age_ms, max_ms)
            }
            AuthError::FutureTimestamp { future_ms, max_ms } => {
                write!(f, "Command timestamp is in the future by {}ms (max drift {}ms)", future_ms, max_ms)
            }
            AuthError::InvalidKeyLength => write!(f, "Secret key provided has invalid length"),
        }
    }
}

impl std::error::Error for AuthError {}

/// Constant-time 128-bit MAC equality comparison to prevent timing attacks.
fn constant_time_eq_16(a: &[u8; MAC_LEN], b: &[u8; MAC_LEN]) -> bool {
    let mut diff = 0u8;
    for i in 0..MAC_LEN {
        diff |= a[i] ^ b[i];
    }
    diff == 0
}

#[derive(Debug, Clone)]
pub struct CommandAuthenticator {
    psk: Vec<u8>,
}

impl CommandAuthenticator {
    pub fn new(shared_key: &[u8]) -> Self {
        Self {
            psk: shared_key.to_vec(),
        }
    }

    /// Signs a command with a session ID, timestamp, and command ID.
    /// Returns a 16-byte truncated HMAC-SHA256 signature.
    pub fn sign_command(
        &self,
        session_id: u32,
        timestamp_ms: u64,
        command_id: u32,
        payload: &[u8],
    ) -> [u8; MAC_LEN] {
        let mut mac = HmacSha256::new_from_slice(&self.psk).expect("HMAC accepts any key length");
        mac.update(&session_id.to_be_bytes());
        mac.update(&timestamp_ms.to_be_bytes());
        mac.update(&command_id.to_be_bytes());
        mac.update(payload);

        let full_signature = mac.finalize().into_bytes();
        let mut truncated = [0u8; MAC_LEN];
        truncated.copy_from_slice(&full_signature[..MAC_LEN]);
        truncated
    }

    /// Verifies a command's signature and timestamp freshness against the current system clock.
    pub fn verify_command(
        &self,
        session_id: u32,
        timestamp_ms: u64,
        command_id: u32,
        payload: &[u8],
        signature: &[u8; MAC_LEN],
        now_ms: u64,
    ) -> Result<(), AuthError> {
        // 1. Clock drift & freshness check
        if timestamp_ms > now_ms {
            let drift = timestamp_ms - now_ms;
            if drift > MAX_CLOCK_DRIFT_MS {
                return Err(AuthError::FutureTimestamp {
                    future_ms: drift,
                    max_ms: MAX_CLOCK_DRIFT_MS,
                });
            }
        } else {
            let age = now_ms - timestamp_ms;
            if age > MAX_REPLAY_AGE_MS {
                return Err(AuthError::ExpiredTimestamp {
                    age_ms: age,
                    max_ms: MAX_REPLAY_AGE_MS,
                });
            }
        }

        // 2. Cryptographic signature check with constant-time equality
        let expected_mac = self.sign_command(session_id, timestamp_ms, command_id, payload);
        if constant_time_eq_16(&expected_mac, signature) {
            Ok(())
        } else {
            Err(AuthError::SignatureMismatch)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_signature_passes() {
        let auth = CommandAuthenticator::new(b"ARTEMIS_SECRET_PSK_DEFENSE_2026");
        let payload = b"ACTION:TAKEOFF:ALT_50";
        let sig = auth.sign_command(1, 1000, 42, payload);

        let result = auth.verify_command(1, 1000, 42, payload, &sig, 1500);
        assert!(result.is_ok());
    }

    #[test]
    fn test_forged_payload_fails() {
        let auth = CommandAuthenticator::new(b"ARTEMIS_SECRET_PSK_DEFENSE_2026");
        let payload = b"ACTION:TAKEOFF:ALT_50";
        let sig = auth.sign_command(1, 1000, 42, payload);

        // Adversary tampers with altitude or action
        let tampered = b"ACTION:DISARM_AIRBORNE";
        let result = auth.verify_command(1, 1000, 42, tampered, &sig, 1500);
        assert_eq!(result, Err(AuthError::SignatureMismatch));
    }

    #[test]
    fn test_expired_timestamp_replay_rejected() {
        let auth = CommandAuthenticator::new(b"ARTEMIS_SECRET_PSK_DEFENSE_2026");
        let payload = b"ACTION:ARM";
        let sig = auth.sign_command(1, 1000, 42, payload);

        // Replay attempt 10 seconds later
        let result = auth.verify_command(1, 1000, 42, payload, &sig, 11_000);
        match result {
            Err(AuthError::ExpiredTimestamp { age_ms, .. }) => assert_eq!(age_ms, 10_000),
            other => panic!("Expected ExpiredTimestamp, got {:?}", other),
        }
    }

    #[test]
    fn test_cross_session_replay_rejected() {
        let auth = CommandAuthenticator::new(b"ARTEMIS_SECRET_PSK_DEFENSE_2026");
        let payload = b"ACTION:ARM";
        let sig = auth.sign_command(1, 1000, 42, payload);

        // Ground station rebooted, session is now 2
        let result = auth.verify_command(2, 1000, 42, payload, &sig, 1200);
        assert_eq!(result, Err(AuthError::SignatureMismatch));
    }
}
