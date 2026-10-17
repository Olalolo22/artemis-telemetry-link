//! CRC-16-CCITT implementation for Artemis telemetry frame integrity verification.
//! Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1), Initial value: 0xFFFF.

/// Computes the CRC-16-CCITT checksum over a slice of bytes.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_standard_vector() {
        // Standard check vector: "123456789" (ASCII bytes) -> 0x29B1 for CCITT with init 0xFFFF
        let input = b"123456789";
        assert_eq!(crc16_ccitt(input), 0x29B1);
    }

    #[test]
    fn test_crc16_empty_slice() {
        assert_eq!(crc16_ccitt(&[]), 0xFFFF);
    }

    #[test]
    fn test_crc16_detects_single_bit_flip() {
        let mut data = *b"ARTEMIS_TELEMETRY_PACKET_001";
        let original_crc = crc16_ccitt(&data);

        // Flip one bit in payload
        data[5] ^= 0x01;
        let corrupted_crc = crc16_ccitt(&data);

        assert_ne!(original_crc, corrupted_crc, "CRC must detect single bit corruption");
    }
}
