//! Real-time RF datalink quality estimator, packet loss tracking, and telemetry health metrics.

#[derive(Debug, Clone)]
pub struct LinkStatistics {
    pub packets_received: u64,
    pub packets_dropped_estimate: u64,
    pub corrupted_crc_count: u64,
    pub out_of_order_count: u64,
    pub packet_delivery_ratio_pct: f32,
    pub last_sequence_id: Option<u32>,
}

#[derive(Debug)]
pub struct LinkMonitor {
    packets_received: u64,
    packets_dropped: u64,
    corrupted_crc: u64,
    out_of_order: u64,
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
            last_seq: None,
        }
    }

    pub fn record_corrupted_crc(&mut self) {
        self.corrupted_crc += 1;
    }

    pub fn record_valid_packet(&mut self, seq: u32) {
        self.packets_received += 1;

        match self.last_seq {
            None => {
                self.last_seq = Some(seq);
            }
            Some(last) => {
                if seq > last {
                    let gap = seq - last - 1;
                    self.packets_dropped += gap as u64;
                    self.last_seq = Some(seq);
                } else {
                    // Out-of-order or duplicate packet
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
        // Total expected = 5, received = 3 -> 60.0%
        assert!((stats.packet_delivery_ratio_pct - 60.0).abs() < 0.001);
    }

    #[test]
    fn test_detects_out_of_order() {
        let mut monitor = LinkMonitor::new();
        monitor.record_valid_packet(1);
        monitor.record_valid_packet(3); // Gap of 1 (seq 2 dropped)
        monitor.record_valid_packet(2); // Delayed seq 2 arrives out-of-order

        let stats = monitor.statistics();
        assert_eq!(stats.out_of_order_count, 1);
    }
}
