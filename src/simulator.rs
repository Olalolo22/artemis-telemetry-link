//! Lossy RF network channel simulator modeling real-world UAV radio link degradation.
//! Simulates random packet loss, jitter, and bit-flip corruptions.

use crate::frame::Frame;
use rand::Rng;

#[derive(Debug, Clone)]
pub struct ChannelProfile {
    pub drop_probability: f32,       // e.g., 0.15 for 15% packet drop
    pub corruption_probability: f32, // e.g., 0.02 for 2% bit-flip corruption
}

impl ChannelProfile {
    pub fn perfect() -> Self {
        Self {
            drop_probability: 0.0,
            corruption_probability: 0.0,
        }
    }

    pub fn contested_rf() -> Self {
        Self {
            drop_probability: 0.20,
            corruption_probability: 0.04,
        }
    }
}

pub struct LossyChannel {
    profile: ChannelProfile,
}

impl LossyChannel {
    pub fn new(profile: ChannelProfile) -> Self {
        Self { profile }
    }

    /// Processes an outgoing frame through the simulated channel using thread-local RNG.
    /// Returns `None` if the packet was dropped over the air, or `Some(wire_bytes)` if delivered.
    pub fn send(&self, frame: &Frame) -> Option<Vec<u8>> {
        let mut rng = rand::thread_rng();
        self.send_with_rng(frame, &mut rng)
    }

    /// Deterministic send implementation allowing seeded RNG injection for unit testing.
    pub fn send_with_rng<R: Rng>(&self, frame: &Frame, rng: &mut R) -> Option<Vec<u8>> {
        // 1. Packet drop simulation
        if rng.gen::<f32>() < self.profile.drop_probability {
            return None;
        }

        let mut wire_bytes = match frame.encode() {
            Ok(b) => b,
            Err(_) => return None,
        };

        // 2. RF bit-flip corruption simulation
        if rng.gen::<f32>() < self.profile.corruption_probability && !wire_bytes.is_empty() {
            let corrupt_idx = rng.gen_range(0..wire_bytes.len());
            wire_bytes[corrupt_idx] ^= 0x01;
        }

        Some(wire_bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{Frame, MsgType};
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    #[test]
    fn test_perfect_channel_delivers_always() {
        let channel = LossyChannel::new(ChannelProfile::perfect());
        let frame = Frame::new(MsgType::Heartbeat, 1, 100, vec![1, 2, 3]);

        for _ in 0..50 {
            let delivered = channel.send(&frame);
            assert!(delivered.is_some());
            let (decoded, _) = Frame::decode(&delivered.unwrap()).expect("Should decode cleanly");
            assert_eq!(decoded.sequence_id, 1);
        }
    }

    #[test]
    fn test_seeded_channel_drops_and_corrupts_deterministically() {
        // Seeded RNG guarantees test repeatability without flaky failures
        let mut rng = StdRng::seed_from_u64(0x1337_CAFE);
        let channel = LossyChannel::new(ChannelProfile {
            drop_probability: 0.30,
            corruption_probability: 0.20,
        });
        let frame = Frame::new(MsgType::Heartbeat, 1, 100, vec![0xDE, 0xAD, 0xBE, 0xEF]);

        let mut drops = 0;
        let mut corruptions = 0;
        let mut clean = 0;
        let trials = 100;

        for _ in 0..trials {
            match channel.send_with_rng(&frame, &mut rng) {
                None => drops += 1,
                Some(bytes) => {
                    if Frame::decode(&bytes).is_err() {
                        corruptions += 1;
                    } else {
                        clean += 1;
                    }
                }
            }
        }

        assert!(drops > 15, "Expected dropped packets, got {}", drops);
        assert!(corruptions > 5, "Expected CRC-corrupted packets, got {}", corruptions);
        assert!(clean > 30, "Expected clean packets, got {}", clean);
    }
}
