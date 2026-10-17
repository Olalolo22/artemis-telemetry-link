//! Binary datalink framing codec with sync words, sequence numbering, and CRC16 protection.
//!
//! Frame wire layout:
//! [0..2]   SYNC_MAGIC     : 0xAA 0x55 (2 bytes)
//! [2]      MSG_TYPE       : u8 (1 byte)
//! [3..7]   SEQUENCE_ID    : u32 big-endian (4 bytes)
//! [7..15]  TIMESTAMP_MS   : u64 big-endian (8 bytes)
//! [15..17] PAYLOAD_LEN    : u16 big-endian (2 bytes)
//! [17..17+N] PAYLOAD      : N bytes
//! [17+N..19+N] CRC16      : u16 big-endian (calculated over 0..17+N)

use crate::crc::crc16_ccitt;
use std::fmt;

pub const SYNC_MAGIC_0: u8 = 0xAA;
pub const SYNC_MAGIC_1: u8 = 0x55;
pub const HEADER_LEN: usize = 17;
pub const CRC_LEN: usize = 2;
pub const MIN_FRAME_LEN: usize = HEADER_LEN + CRC_LEN;
pub const MAX_PAYLOAD_LEN: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MsgType {
    Heartbeat = 0x01,
    Telemetry = 0x02,
    Command = 0x03,
    CommandAck = 0x04,
    Unknown(u8),
}

impl From<u8> for MsgType {
    fn from(b: u8) -> Self {
        match b {
            0x01 => MsgType::Heartbeat,
            0x02 => MsgType::Telemetry,
            0x03 => MsgType::Command,
            0x04 => MsgType::CommandAck,
            other => MsgType::Unknown(other),
        }
    }
}

impl From<MsgType> for u8 {
    fn from(t: MsgType) -> Self {
        match t {
            MsgType::Heartbeat => 0x01,
            MsgType::Telemetry => 0x02,
            MsgType::Command => 0x03,
            MsgType::CommandAck => 0x04,
            MsgType::Unknown(b) => b,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub msg_type: MsgType,
    pub sequence_id: u32,
    pub timestamp_ms: u64,
    pub payload: Vec<u8>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum FrameError {
    BufferTooShort { needed: usize, provided: usize },
    InvalidMagic { b0: u8, b1: u8 },
    PayloadTooLarge { length: usize, max: usize },
    CorruptedChecksum { expected: u16, found: u16 },
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FrameError::BufferTooShort { needed, provided } => {
                write!(f, "Frame buffer too short: needed {} bytes, got {}", needed, provided)
            }
            FrameError::InvalidMagic { b0, b1 } => {
                write!(f, "Invalid sync magic: expected [0xAA, 0x55], got [0x{:02X}, 0x{:02X}]", b0, b1)
            }
            FrameError::PayloadTooLarge { length, max } => {
                write!(f, "Payload length {} exceeds maximum allowed {}", length, max)
            }
            FrameError::CorruptedChecksum { expected, found } => {
                write!(f, "CRC16 checksum mismatch: expected 0x{:04X}, calculated 0x{:04X}", expected, found)
            }
        }
    }
}

impl std::error::Error for FrameError {}

impl Frame {
    pub fn new(msg_type: MsgType, sequence_id: u32, timestamp_ms: u64, payload: Vec<u8>) -> Self {
        Self {
            msg_type,
            sequence_id,
            timestamp_ms,
            payload,
        }
    }

    /// Serializes the frame into wire format with magic header and trailing CRC16.
    pub fn encode(&self) -> Vec<u8> {
        let payload_len = self.payload.len() as u16;
        let mut buf = Vec::with_capacity(HEADER_LEN + self.payload.len() + CRC_LEN);

        buf.push(SYNC_MAGIC_0);
        buf.push(SYNC_MAGIC_1);
        buf.push(self.msg_type.into());
        buf.extend_from_slice(&self.sequence_id.to_be_bytes());
        buf.extend_from_slice(&self.timestamp_ms.to_be_bytes());
        buf.extend_from_slice(&payload_len.to_be_bytes());
        buf.extend_from_slice(&self.payload);

        let calculated_crc = crc16_ccitt(&buf);
        buf.extend_from_slice(&calculated_crc.to_be_bytes());
        buf
    }

    /// Attempts to decode a single frame from the provided buffer.
    /// Returns the parsed `Frame` along with total bytes consumed.
    pub fn decode(buf: &[u8]) -> Result<(Self, usize), FrameError> {
        if buf.len() < MIN_FRAME_LEN {
            return Err(FrameError::BufferTooShort {
                needed: MIN_FRAME_LEN,
                provided: buf.len(),
            });
        }

        if buf[0] != SYNC_MAGIC_0 || buf[1] != SYNC_MAGIC_1 {
            return Err(FrameError::InvalidMagic {
                b0: buf[0],
                b1: buf[1],
            });
        }

        let msg_type = MsgType::from(buf[2]);
        let sequence_id = u32::from_be_bytes([buf[3], buf[4], buf[5], buf[6]]);
        let timestamp_ms = u64::from_be_bytes([
            buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14],
        ]);
        let payload_len = u16::from_be_bytes([buf[15], buf[16]]) as usize;

        if payload_len > MAX_PAYLOAD_LEN {
            return Err(FrameError::PayloadTooLarge {
                length: payload_len,
                max: MAX_PAYLOAD_LEN,
            });
        }

        let total_frame_len = HEADER_LEN + payload_len + CRC_LEN;
        if buf.len() < total_frame_len {
            return Err(FrameError::BufferTooShort {
                needed: total_frame_len,
                provided: buf.len(),
            });
        }

        // Verify CRC16 over Header + Payload
        let data_to_check = &buf[..HEADER_LEN + payload_len];
        let calculated_crc = crc16_ccitt(data_to_check);

        let crc_offset = HEADER_LEN + payload_len;
        let wire_crc = u16::from_be_bytes([buf[crc_offset], buf[crc_offset + 1]]);

        if calculated_crc != wire_crc {
            return Err(FrameError::CorruptedChecksum {
                expected: wire_crc,
                found: calculated_crc,
            });
        }

        let payload = buf[HEADER_LEN..HEADER_LEN + payload_len].to_vec();

        Ok((
            Frame {
                msg_type,
                sequence_id,
                timestamp_ms,
                payload,
            },
            total_frame_len,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_encode_decode_roundtrip() {
        let payload = b"ALTITUDE_100M_BATTERY_98PCT".to_vec();
        let frame = Frame::new(MsgType::Telemetry, 42, 1729182300000, payload.clone());

        let encoded = frame.encode();
        assert!(encoded.len() > MIN_FRAME_LEN);

        let (decoded, consumed) = Frame::decode(&encoded).expect("Decode must succeed");
        assert_eq!(consumed, encoded.len());
        assert_eq!(decoded.msg_type, MsgType::Telemetry);
        assert_eq!(decoded.sequence_id, 42);
        assert_eq!(decoded.timestamp_ms, 1729182300000);
        assert_eq!(decoded.payload, payload);
    }

    #[test]
    fn test_rejects_corrupted_crc() {
        let payload = b"SECRET_MISSION_COMMAND".to_vec();
        let frame = Frame::new(MsgType::Command, 10, 1000, payload);
        let mut encoded = frame.encode();

        // Corrupt one payload byte
        let payload_idx = HEADER_LEN + 2;
        encoded[payload_idx] ^= 0xFF;

        let res = Frame::decode(&encoded);
        match res {
            Err(FrameError::CorruptedChecksum { .. }) => {} // Expected
            other => panic!("Expected CorruptedChecksum error, got {:?}", other),
        }
    }

    #[test]
    fn test_rejects_bad_magic() {
        let mut encoded = Frame::new(MsgType::Heartbeat, 1, 500, vec![]).encode();
        encoded[0] = 0x00; // Invalidate sync word

        let res = Frame::decode(&encoded);
        match res {
            Err(FrameError::InvalidMagic { b0, .. }) => assert_eq!(b0, 0x00),
            other => panic!("Expected InvalidMagic error, got {:?}", other),
        }
    }

    #[test]
    fn test_rejects_truncated_buffer() {
        let encoded = Frame::new(MsgType::Heartbeat, 1, 500, vec![1, 2, 3]).encode();
        let truncated = &encoded[..HEADER_LEN]; // Misses payload & CRC

        let res = Frame::decode(truncated);
        match res {
            Err(FrameError::BufferTooShort { .. }) => {}
            other => panic!("Expected BufferTooShort, got {:?}", other),
        }
    }
}
