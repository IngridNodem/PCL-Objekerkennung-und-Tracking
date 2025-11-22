#!/usr/bin/env python3
import argparse
import time
from statistics import median
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray, Marker


@dataclass
class TrackLife:
    first: float
    last: float
    alive: bool = True


class IdsProbe(Node):
    def __init__(self, ns: str, report_interval: float, idle_timeout: float):
        super().__init__('ids_probe')
        self.ns = ns.strip('/')
        self.report_interval = report_interval
        self.idle_timeout = idle_timeout

        self.t_last = time.monotonic()
        self.births_window = 0
        self.completions_window = 0

        # id -> TrackLife (monotonic seconds)
        self.tracks: dict[int, TrackLife] = {}
        self.completed_lifetimes: list[float] = []

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            MarkerArray,
            f'/{self.ns}/tracks_markers',
            self._cb_markers,
            qos,
        )

        self.timer = self.create_timer(self.report_interval, self._report)
        self.get_logger().info(f'IdsProbe on /{self.ns}/tracks_markers (interval={self.report_interval}s, idle_timeout={self.idle_timeout}s)')

    def _cb_markers(self, msg: MarkerArray):
        now = time.monotonic()

        # Collect seen IDs for this frame
        seen_ids = set()
        for m in msg.markers:
            # Only consider the CUBE boxes (namespace 'tracks'); ignore labels and DELETEALL commands
            if m.action == Marker.DELETEALL:
                continue
            if m.ns and m.ns != 'tracks':
                continue
            if m.type != Marker.CUBE:
                continue
            tid = int(m.id)
            seen_ids.add(tid)
            tl = self.tracks.get(tid)
            if tl is None:
                self.tracks[tid] = TrackLife(first=now, last=now, alive=True)
                self.births_window += 1
            else:
                tl.last = now

        # Finalize tracks that were not seen recently
        to_finalize = []
        for tid, tl in self.tracks.items():
            if not tl.alive:
                continue
            if (now - tl.last) >= self.idle_timeout:
                to_finalize.append(tid)
        for tid in to_finalize:
            tl = self.tracks[tid]
            tl.alive = False
            self.completed_lifetimes.append(max(0.0, tl.last - tl.first))
            self.completions_window += 1

    def _report(self):
        now = time.monotonic()
        dt = max(now - self.t_last, 1e-6)
        self.t_last = now

        active = [tl for tl in self.tracks.values() if tl.alive]
        active_count = len(active)
        active_lifetimes = [max(0.0, (now - tl.first)) for tl in active]
        med_active = median(active_lifetimes) if active_lifetimes else float('nan')
        med_completed = median(self.completed_lifetimes) if self.completed_lifetimes else float('nan')

        births_per_min = (self.births_window / dt) * 60.0
        comp_per_min = (self.completions_window / dt) * 60.0

        self.get_logger().info(
            f"active={active_count}  births/min={births_per_min:.1f}  completed/min={comp_per_min:.1f}  "
            f"med_active_life={med_active:.1f}s  med_completed_life={med_completed:.1f}s"
        )

        # reset window counters
        self.births_window = 0
        self.completions_window = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='bench', help='Namespace (default: bench)')
    ap.add_argument('--interval', type=float, default=5.0, help='Report interval seconds (default: 5.0)')
    ap.add_argument('--idle-timeout', type=float, default=0.6, help='Consider a track finished if unseen for this many seconds (default: 0.6)')
    args = ap.parse_args()

    rclpy.init()
    node = IdsProbe(args.ns, report_interval=args.interval, idle_timeout=args.idle_timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

