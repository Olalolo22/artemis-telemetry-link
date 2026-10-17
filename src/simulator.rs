//! Lossy RF network channel simulator modeling real-world UAV radio link degradation.
//! Simulates random packet loss, jitter, and bit-flip corruptions.

use crate::frame::Frame;
use rand::Rng;

#[derive(Debug, Clone)]
pub struct ChannelProfile {
    pub drop_probability: f32,       // e.g., 0.20 for 20% packet drop
    pub corruption_probability: f32, // e.g., 0.05 for 5% bit-flip corruption
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
            drop_probability: 0.25,
            corruption_probability: 0.05,
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

    /// Processes an outgoing frame through the simulated channel.
    /// Returns `None` if the packet was dropped, or `Some(wire_bytes)` if delivered.
    pub fn send(&self, frame: &Frame) -> Option<Vec<u8>> {
        let mut rng = rand::thread_rng();

        // 1. Packet drop simulation
        if rng.gen::<f32>() < self.profile.drop_probability {
            return None; // Packet lost in transit
        }

        let mut wire_bytes = frame.encode();

        // 2. RF noise / Bit flip simulation
        if rng.gen::<f32>() < self.profile.corruption_probability && !wire_bytes.is_empty() {
            let corrupt_idx = rng.gen_range(0..wire_bytes.len());
            wire_bytes[corrupt_idx] ^= 0x01; // Invert a single bit
        }

        Some(wire_bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{Frame, MsgType};

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
    fn test_contested_channel_drops_and_corrupts() {
        let channel = LossyChannel::new(ChannelProfile {
            drop_probability: 0.50,
            corruption_probability: 0.0,
        });
        let frame = Frame::new(MsgType::Heartbeat, 1, 100, vec![]);

        let mut dropped = 0;
        let trials = 200;
        for _ in 0..trials {
            if channel.send(&frame).is_none() {
                dropped += 1;
            }
        }

        // With 50% drop rate over 200 trials, drops should reasonably fall between 60 and 140
        assert!(dropped > 50 && dropped < 150, "Observed drops: {}", dropped);
    }
}
