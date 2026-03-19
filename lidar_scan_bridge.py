#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class LidarScanBridge(Node):
    def __init__(self):
        super().__init__('lidar_scan_bridge')

        self.declare_parameter('input_topic', '/utlidar/cloud_base_synced')
        self.declare_parameter('output_topic', '/lidar_scan')
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment_deg', 0.5)
        self.declare_parameter('range_min', 0.20)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('min_height', -0.25)
        self.declare_parameter('max_height', 0.25)
        self.declare_parameter('default_scan_time', 0.10)
        self.declare_parameter('use_inf', True)
        self.declare_parameter('filter_isolated_ranges', True)
        self.declare_parameter('neighbor_window', 2)
        self.declare_parameter('neighbor_tolerance', 0.30)
        self.declare_parameter('min_neighbor_count', 1)
        self.declare_parameter('filter_small_clusters', True)
        self.declare_parameter('cluster_distance_tolerance', 0.35)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('apply_median_filter', True)
        self.declare_parameter('median_filter_window', 2)
        self.declare_parameter('median_filter_min_samples', 3)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max_limit = float(self.get_parameter('angle_max').value)
        self.angle_increment = math.radians(float(self.get_parameter('angle_increment_deg').value))
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.default_scan_time = float(self.get_parameter('default_scan_time').value)
        self.use_inf = bool(self.get_parameter('use_inf').value)
        self.filter_isolated_ranges = bool(self.get_parameter('filter_isolated_ranges').value)
        self.neighbor_window = int(self.get_parameter('neighbor_window').value)
        self.neighbor_tolerance = float(self.get_parameter('neighbor_tolerance').value)
        self.min_neighbor_count = int(self.get_parameter('min_neighbor_count').value)
        self.filter_small_clusters = bool(self.get_parameter('filter_small_clusters').value)
        self.cluster_distance_tolerance = float(self.get_parameter('cluster_distance_tolerance').value)
        self.min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        self.apply_median_filter = bool(self.get_parameter('apply_median_filter').value)
        self.median_filter_window = int(self.get_parameter('median_filter_window').value)
        self.median_filter_min_samples = int(self.get_parameter('median_filter_min_samples').value)

        if self.angle_max_limit <= self.angle_min:
            raise ValueError('angle_max must be greater than angle_min')
        if self.angle_increment <= 0.0:
            raise ValueError('angle_increment_deg must be positive')
        if self.max_height < self.min_height:
            raise ValueError('max_height must be >= min_height')
        if self.neighbor_window < 1:
            raise ValueError('neighbor_window must be >= 1')
        if self.min_neighbor_count < 0:
            raise ValueError('min_neighbor_count must be >= 0')
        if self.cluster_distance_tolerance < 0.0:
            raise ValueError('cluster_distance_tolerance must be >= 0')
        if self.min_cluster_size < 1:
            raise ValueError('min_cluster_size must be >= 1')
        if self.median_filter_window < 1:
            raise ValueError('median_filter_window must be >= 1')
        if self.median_filter_min_samples < 1:
            raise ValueError('median_filter_min_samples must be >= 1')

        self.beam_count = max(
            1,
            int(math.ceil((self.angle_max_limit - self.angle_min) / self.angle_increment)),
        )
        self.output_angle_max = self.angle_min + (self.beam_count - 1) * self.angle_increment
        self.last_stamp_sec = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scan_pub = self.create_publisher(LaserScan, self.output_topic, qos)
        self.cloud_sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos)

        self.get_logger().info(
            f'Publishing {self.output_topic} from {self.input_topic} '
            f'with {self.beam_count} beams, height filter [{self.min_height:.2f}, {self.max_height:.2f}] m'
        )

    def cloud_callback(self, msg: PointCloud2):
        ranges = [math.inf] * self.beam_count

        for x, y, z in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if z < self.min_height or z > self.max_height:
                continue

            planar_range = math.hypot(x, y)
            if planar_range < self.range_min or planar_range > self.range_max:
                continue

            angle = math.atan2(y, x)
            if angle < self.angle_min or angle >= self.angle_max_limit:
                continue

            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < self.beam_count and planar_range < ranges[index]:
                ranges[index] = planar_range

        if self.filter_isolated_ranges:
            ranges = self.remove_isolated_ranges(ranges)
        if self.filter_small_clusters:
            ranges = self.remove_small_clusters(ranges)
        if self.apply_median_filter:
            ranges = self.apply_median_filter_to_ranges(ranges)

        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.output_angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.compute_scan_time(msg.header.stamp)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges if self.use_inf else [
            value if math.isfinite(value) else self.range_max + 1.0
            for value in ranges
        ]

        self.scan_pub.publish(scan)

    def remove_isolated_ranges(self, ranges):
        filtered = list(ranges)

        for index, value in enumerate(ranges):
            if not math.isfinite(value):
                continue

            neighbor_count = 0
            start = max(0, index - self.neighbor_window)
            end = min(self.beam_count, index + self.neighbor_window + 1)
            for neighbor_index in range(start, end):
                if neighbor_index == index:
                    continue

                neighbor_value = ranges[neighbor_index]
                if not math.isfinite(neighbor_value):
                    continue

                if abs(neighbor_value - value) <= self.neighbor_tolerance:
                    neighbor_count += 1

            if neighbor_count < self.min_neighbor_count:
                filtered[index] = math.inf

        return filtered

    def remove_small_clusters(self, ranges):
        filtered = list(ranges)
        cluster_start = None

        def clear_cluster(start_index, end_index):
            for cluster_index in range(start_index, end_index):
                filtered[cluster_index] = math.inf

        for index, value in enumerate(ranges):
            if not math.isfinite(value):
                if cluster_start is not None:
                    if (index - cluster_start) < self.min_cluster_size:
                        clear_cluster(cluster_start, index)
                    cluster_start = None
                continue

            if cluster_start is None:
                cluster_start = index
                continue

            previous_value = ranges[index - 1]
            if not math.isfinite(previous_value):
                if (index - cluster_start) < self.min_cluster_size:
                    clear_cluster(cluster_start, index)
                cluster_start = index
                continue

            if abs(value - previous_value) > self.cluster_distance_tolerance:
                if (index - cluster_start) < self.min_cluster_size:
                    clear_cluster(cluster_start, index)
                cluster_start = index

        if cluster_start is not None and (self.beam_count - cluster_start) < self.min_cluster_size:
            clear_cluster(cluster_start, self.beam_count)

        return filtered

    def apply_median_filter_to_ranges(self, ranges):
        filtered = list(ranges)

        for index, value in enumerate(ranges):
            if not math.isfinite(value):
                continue

            samples = []
            start = max(0, index - self.median_filter_window)
            end = min(self.beam_count, index + self.median_filter_window + 1)
            for neighbor_index in range(start, end):
                neighbor_value = ranges[neighbor_index]
                if math.isfinite(neighbor_value):
                    samples.append(neighbor_value)

            if len(samples) < self.median_filter_min_samples:
                continue

            samples.sort()
            mid = len(samples) // 2
            if len(samples) % 2 == 1:
                filtered[index] = samples[mid]
            else:
                filtered[index] = 0.5 * (samples[mid - 1] + samples[mid])

        return filtered

    def compute_scan_time(self, stamp):
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.last_stamp_sec is None or stamp_sec <= self.last_stamp_sec:
            self.last_stamp_sec = stamp_sec
            return self.default_scan_time

        scan_time = stamp_sec - self.last_stamp_sec
        self.last_stamp_sec = stamp_sec
        return scan_time


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
