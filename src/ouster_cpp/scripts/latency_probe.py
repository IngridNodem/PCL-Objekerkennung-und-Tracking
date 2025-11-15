#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray
import argparse
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

def t2sec(t):
    # builtin_interfaces/Time -> float seconds
    return float(t.sec) + float(t.nanosec) * 1e-9

class LatencyProbe(Node):
    def __init__(self, ns: str, duration: float = None, idle_timeout: float = None):
        super().__init__('latency_probe')
        self.ns = ns.strip('/')
        self.duration = duration
        self.idle_timeout = idle_timeout
        self.start_time = time.monotonic()
        self.last_msg_time = self.start_time
        self.stopped = False

        # Last header.stamp (seconds) per stage
        self.stamps = {
            'crop': None,
            'voxel': None,
            'ransac': None,
            'cluster': None,
            'track': None,
        }

        # Running sums and counts for averages
        self.sum = { 'crop->voxel':0.0, 'voxel->ransac':0.0, 'ransac->cluster':0.0, 'cluster->track':0.0,
                     'crop->track':0.0 }
        self.cnt = { k:0 for k in self.sum.keys() }

        # QoS: certains topics publient en BEST_EFFORT (SensorData). Adapter les abonnements.
        qos_best_effort = qos_profile_sensor_data
        qos_reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)

        self.sub_crop   = self.create_subscription(PointCloud2, f'/{self.ns}/points_cropped',  self.on_crop,   qos_best_effort)
        self.sub_voxel  = self.create_subscription(PointCloud2, f'/{self.ns}/points_voxel',    self.on_voxel,  qos_reliable)
        self.sub_ransac = self.create_subscription(PointCloud2, f'/{self.ns}/obstacle_points', self.on_ransac, qos_reliable)
        self.sub_clust  = self.create_subscription(Detection3DArray, f'/{self.ns}/detections_raw', self.on_cluster, qos_best_effort)
        self.sub_track  = self.create_subscription(Detection3DArray, f'/{self.ns}/tracks_raw',     self.on_track,   qos_reliable)

        self.timer = self.create_timer(2.0, self.report)
        self.get_logger().info(f'LatencyProbe listening under ns=/{self.ns}')

    def on_crop(self, msg: PointCloud2):
        self.stamps['crop'] = t2sec(msg.header.stamp)
        self.last_msg_time = time.monotonic()

    def on_voxel(self, msg: PointCloud2):
        s = t2sec(msg.header.stamp)
        self.stamps['voxel'] = s
        c = self.stamps['crop']
        if c is not None and s >= c:
            self._acc('crop->voxel', s - c)
        self.last_msg_time = time.monotonic()

    def on_ransac(self, msg: PointCloud2):
        s = t2sec(msg.header.stamp)
        self.stamps['ransac'] = s
        v = self.stamps['voxel']
        if v is not None and s >= v:
            self._acc('voxel->ransac', s - v)
        self.last_msg_time = time.monotonic()

    def on_cluster(self, msg: Detection3DArray):
        s = t2sec(msg.header.stamp)
        self.stamps['cluster'] = s
        r = self.stamps['ransac']
        if r is not None and s >= r:
            self._acc('ransac->cluster', s - r)
        self.last_msg_time = time.monotonic()

    def on_track(self, msg: Detection3DArray):
        s = t2sec(msg.header.stamp)
        self.stamps['track'] = s
        c = self.stamps['cluster']
        if c is not None and s >= c:
            self._acc('cluster->track', s - c)
        crop = self.stamps['crop']
        if crop is not None and s >= crop:
            self._acc('crop->track', s - crop)
        self.last_msg_time = time.monotonic()

    def _acc(self, key: str, dt: float):
        if math.isfinite(dt):
            self.sum[key] += dt
            self.cnt[key] += 1

    def report(self):
        lines = []
        for k in ['crop->voxel','voxel->ransac','ransac->cluster','cluster->track','crop->track']:
            n = self.cnt[k]
            ms = (self.sum[k]/n*1000.0) if n>0 else float('nan')
            lines.append(f"{k}: {ms:.2f} ms (n={n})")
        self.get_logger().info(' | '.join(lines))
        now = time.monotonic()
        # Check auto-stop conditions
        if not self.stopped:
            if self.duration is not None and (now - self.start_time) >= self.duration:
                self.get_logger().info('Duration reached — stopping.')
                self.stopped = True
                rclpy.shutdown()
                return
            if self.idle_timeout is not None and (now - self.last_msg_time) >= self.idle_timeout:
                self.get_logger().info('Idle timeout reached — stopping.')
                self.stopped = True
                rclpy.shutdown()
                return

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', default='bench', help='Namespace (default: bench)')
    parser.add_argument('--duration', type=float, default=None, help='Stop after N seconds (wall clock).')
    parser.add_argument('--idle-timeout', type=float, default=None, help='Stop if no messages for N seconds (wall clock).')
    args = parser.parse_args()

    rclpy.init()
    node = LatencyProbe(args.ns, duration=args.duration, idle_timeout=args.idle_timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
