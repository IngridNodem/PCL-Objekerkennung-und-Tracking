#!/usr/bin/env python3
import argparse
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray


@dataclass
class TopicStats:
    name: str
    count_total: int = 0
    count_window: int = 0
    last_arrival_monotonic: float | None = None
    avg_dt: float | None = None
    gaps: int = 0

    def on_msg(self, now_mono: float, gap_threshold_s: float | None = None):
        # Update counters
        self.count_total += 1
        self.count_window += 1
        # Gap detection
        if self.last_arrival_monotonic is not None:
            dt = now_mono - self.last_arrival_monotonic
            # EWMA for avg_dt (stabilize on varying rates)
            if self.avg_dt is None:
                self.avg_dt = dt
            else:
                self.avg_dt = 0.9 * self.avg_dt + 0.1 * dt
            # Threshold: explicit or 2x average period (fallback 0.25s)
            thr = gap_threshold_s if gap_threshold_s is not None else max(0.25, 2.0 * (self.avg_dt or 0.0))
            if dt > thr:
                self.gaps += 1
        self.last_arrival_monotonic = now_mono


class FrameLossProbe(Node):
    def __init__(self, ns: str, report_interval: float, gap_threshold: float | None):
        super().__init__('frame_loss_probe')
        self.ns = ns.strip('/')
        self.report_interval = report_interval
        self.gap_threshold = gap_threshold
        self.t_last = time.monotonic()

        # Topics to watch (ordered pipeline)
        self.topic_names = [
            '/ouster/points',
            f'/{self.ns}/points_cropped',
            f'/{self.ns}/points_voxel',
            f'/{self.ns}/obstacle_points',
            f'/{self.ns}/detections_raw',
            f'/{self.ns}/tracks_raw',
        ]
        # Initialize stats per topic
        self.stats: dict[str, TopicStats] = {name: TopicStats(name) for name in self.topic_names}

        # QoS: match actual topics as best as possible
        qos_best_effort = qos_profile_sensor_data
        qos_reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions (bind topic name into callback)
        self._make_sub(PointCloud2, '/ouster/points', qos_best_effort)
        self._make_sub(PointCloud2, f'/{self.ns}/points_cropped', qos_best_effort)
        self._make_sub(PointCloud2, f'/{self.ns}/points_voxel', qos_reliable)
        self._make_sub(PointCloud2, f'/{self.ns}/obstacle_points', qos_reliable)
        self._make_sub(Detection3DArray, f'/{self.ns}/detections_raw', qos_best_effort)
        self._make_sub(Detection3DArray, f'/{self.ns}/tracks_raw', qos_reliable)

        # Periodic report
        self.timer = self.create_timer(self.report_interval, self._report)
        self.get_logger().info(f'FrameLossProbe watching ns=/{self.ns} (interval={self.report_interval}s, gap_thr={self.gap_threshold or "auto"}s)')

    def _make_sub(self, msg_type, topic: str, qos: QoSProfile):
        def _cb(_):
            st = self.stats.get(topic)
            if st is None:
                return
            st.on_msg(time.monotonic(), self.gap_threshold)
        self.create_subscription(msg_type, topic, _cb, qos)

    def _report(self):
        now = time.monotonic()
        dt = max(now - self.t_last, 1e-6)
        self.t_last = now

        # Compute rates
        rates = {}
        for name in self.topic_names:
            st = self.stats[name]
            hz = st.count_window / dt
            rates[name] = hz
        
        # Pass-through ratios between adjacent stages
        def ratio(a, b):
            ra = rates.get(a, 0.0)
            rb = rates.get(b, 0.0)
            if ra <= 1e-6:
                return float('nan')
            return 100.0 * (rb / ra)

        lines = []
        lines.append(f"in={rates[self.topic_names[0]]:5.2f}Hz crop={rates[self.topic_names[1]]:5.2f}Hz voxel={rates[self.topic_names[2]]:5.2f}Hz")
        lines.append(f"ransac={rates[self.topic_names[3]]:5.2f}Hz clust={rates[self.topic_names[4]]:5.2f}Hz track={rates[self.topic_names[5]]:5.2f}Hz")
        lines.append(
            f"pass%: crop/in={ratio(self.topic_names[0], self.topic_names[1]):5.1f}  "
            f"vox/crop={ratio(self.topic_names[1], self.topic_names[2]):5.1f}  "
            f"ran/vox={ratio(self.topic_names[2], self.topic_names[3]):5.1f}  "
            f"cl/ran={ratio(self.topic_names[3], self.topic_names[4]):5.1f}  "
            f"trk/cl={ratio(self.topic_names[4], self.topic_names[5]):5.1f}"
        )
        gap_line = "gaps:" + " ".join(
            f"{name.split('/')[-1]}={self.stats[name].gaps}" for name in self.topic_names
        )

        self.get_logger().info(" | ".join(lines) + " | " + gap_line)

        # Reset window counters and gaps for next interval
        for st in self.stats.values():
            st.count_window = 0
            st.gaps = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='bench', help='Namespace (default: bench)')
    ap.add_argument('--interval', type=float, default=2.0, help='Report interval seconds (default: 2.0)')
    ap.add_argument('--gap-threshold', type=float, default=None, help='Gap threshold seconds (default: auto=2x period, min 0.25s)')
    args = ap.parse_args()

    rclpy.init()
    node = FrameLossProbe(args.ns, report_interval=args.interval, gap_threshold=args.gap_threshold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
