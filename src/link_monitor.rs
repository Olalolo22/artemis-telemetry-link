//! Real-time RF datalink quality estimator, packet loss tracking, and telemetry health metrics.
//! Hardened against sequence number poisoning, sequence wraparound, and duplicate packet inflation.

pub const MAX_ACCEPTABLE_SEQUENCE_GAP: u32 = 1_000;

#[derive(Debug, Clone)]
pub struct LinkStatistics {
    pub packets_received: u64,
    pub packets_dropped_estimate: u64,
    pub corrupted_crc_count: u64,
    pub out_of_order_count: u64,
    pub sequence_anomalies_count: u64,
    pub packet_delivery_ratio_pct: f32,
    pub last_sequence_id: Option<u32>,
}

#[derive(Debug)]
pub struct LinkMonitor {
    packets_received: u64,
    packets_dropped: u64,
    corrupted_crc: u64,
    out_of_order: u64,
    sequence_anomalies: u64,
    last_seq: Option<u32>,
}

impl Default for LinkMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl LinkMonitor {
    pub fn new() -> Self {
        Self {
            packets_received: 0,
            packets_dropped: 0,
            corrupted_crc: 0,
            out_of_order: 0,
            sequence_anomalies: 0,
            last_seq: None,
        }
    }

    pub fn record_corrupted_crc(&mut self) {
        self.corrupted_crc += 1;
    }

    /// Records an ingested packet with bounded sequence gap tracking and wraparound protection.
    pub fn record_valid_packet(&mut self, seq: u32) {
        match self.last_seq {
            None => {
                self.last_seq = Some(seq);
                self.packets_received += 1;
            }
            Some(last) => {
                if seq > last {
                    let gap = seq - last - 1;
                    if gap <= MAX_ACCEPTABLE_SEQUENCE_GAP {
                        self.packets_dropped += gap as u64;
                        self.packets_received += 1;
                        self.last_seq = Some(seq);
                    } else {
                        // Sequence anomaly or transmitter reboot/jump: re-anchor baseline without skewing dropped metrics
                        self.sequence_anomalies += 1;
                        self.packets_received += 1;
                        self.last_seq = Some(seq);
                    }
                } else if seq < last {
                    // Check for standard u32 wraparound
                    let wrap_gap = (u32::MAX - last).saturating_add(seq);
                    if wrap_gap <= MAX_ACCEPTABLE_SEQUENCE_GAP {
                        self.packets_dropped += wrap_gap as u64;
                        self.packets_received += 1;
                        self.last_seq = Some(seq);
                    } else {
                        // Out-of-order or duplicate packet; do not increment unique received packets
                        self.out_of_order += 1;
                    }
                } else {
                    // Exact duplicate packet (seq == last)
                    self.out_of_order += 1;
                }
            }
        }
    }

    pub fn statistics(&self) -> LinkStatistics {
        let total_expected = self.packets_received + self.packets_dropped;
        let pdr = if total_expected == 0 {
            100.0
        } else {
            (self.packets_received as f32 / total_expected as f32) * 100.0
        };

        LinkStatistics {
            packets_received: self.packets_received,
            packets_dropped_estimate: self.packets_dropped,
            corrupted_crc_count: self.corrupted_crc,
            out_of_order_count: self.out_of_order,
            sequence_anomalies_count: self.sequence_anomalies,
            packet_delivery_ratio_pct: pdr,
            last_sequence_id: self.last_seq,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perfect_link_pdr() {
        let mut monitor = LinkMonitor::new();
        for seq in 1..=10 {
            monitor.record_valid_packet(seq);
        }

        let stats = monitor.statistics();
        assert_eq!(stats.packets_received, 10);
        assert_eq!(stats.packets_dropped_estimate, 0);
        assert_eq!(stats.packet_delivery_ratio_pct, 100.0);
    }

    #[test]
    fn test_detects_dropped_packets_and_pdr() {
        let mut monitor = LinkMonitor::new();
        monitor.record_valid_packet(1);
        monitor.record_valid_packet(2);
        // Packets 3 and 4 dropped over RF
        monitor.record_valid_packet(5);

        let stats = monitor.statistics();
        assert_eq!(stats.packets_received, 3);
        assert_eq!(stats.packets_dropped_estimate, 2);
        assert!((stats.packet_delivery_ratio_pct - 60.0).abs() < 0.001);
    }

    #[test]
    fn test_sequence_anomaly_poisoning_prevented() {
        let mut monitor = LinkMonitor::new();
        monitor.record_valid_packet(10);

        // Huge forward jump (simulating unauthenticated packet or transmitter reset)
        monitor.record_valid_packet(4_000_000_000);

        let stats = monitor.statistics();
        assert_eq!(stats.sequence_anomalies_count, 1);
        // Dropped packets should NOT be 4 billion!
        assert_eq!(stats.packets_dropped_estimate, 0);
        assert_eq!(stats.packets_received, 2);
    }

    #[test]
    fn test_sequence_wraparound_handled() {
        let mut monitor = LinkMonitor::new();
        monitor.record_valid_packet(u32::MAX);
        // Seamless rollover from u32::MAX to 0
        monitor.record_valid_packet(0);

        let stats = monitor.statistics();
        assert_eq!(stats.packets_dropped_estimate, 0);
        assert_eq!(stats.packets_received, 2);

        // Rollover with 1 dropped packet (u32::MAX skipped)
        let mut monitor_gap = LinkMonitor::new();
        monitor_gap.record_valid_packet(u32::MAX - 1);
        monitor_gap.record_valid_packet(0); // u32::MAX was dropped
        assert_eq!(monitor_gap.statistics().packets_dropped_estimate, 1);
    }

    #[test]
    fn test_duplicate_does_not_inflate_received_count() {
        let mut monitor = LinkMonitor::new();
        monitor.record_valid_packet(1);
        monitor.record_valid_packet(1); // Duplicate
        monitor.record_valid_packet(1); // Duplicate

        let stats = monitor.statistics();
        assert_eq!(stats.packets_received, 1);
        assert_eq!(stats.out_of_order_count, 2);
    }
}
