import time
from math import cos, sin
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import sensor_msgs_py.point_cloud2 as pc2
import yaml

class TimeSyncBridge(Node):
    def __init__(self):
        super().__init__('time_sync_bridge')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.last_external_tf_monotonic = None
        self.external_tf_timeout_sec = 1.0
        self.time_offset_ns_by_stream = {}
        self.last_raw_stamp_ns = {}
        self.last_synced_stamp_ns = {}
        self.backward_jump_reset_ns = 1_000_000_000  # 1s
        self.forward_limit_ns = 2_147_483_647 * 1_000_000_000 + 999_999_999
        self.published_tf_static_children = set()
        
        # 새로운 시간으로 발행할 Publisher들
        # TF 데이터는 RViz에서 무조건 원본 토픽(/tf)을 바라보기 때문에 같은 이름으로 재발행합니다.
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        # tf_static은 RViz가 과거 데이터를 계속 기억해야 하므로 TRANSIENT_LOCAL로 설정합니다.
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', qos_profile)
        
        # 맵과 오도메트리 데이터는 충돌을 막기 위해 끝에 '_synced'를 붙입니다. 
        # (RViz에서는 이 _synced 토픽들을 화면에 띄우시면 됩니다!)
        self.grid_map_pub = self.create_publisher(PointCloud2, '/utlidar/grid_map_synced', 10)
        self.odom_pub = self.create_publisher(Odometry, '/utlidar/robot_odom_synced', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/utlidar/robot_pose_synced', 10)
        self.imu_pub = self.create_publisher(Imu, '/utlidar/imu_synced', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/utlidar/cloud_deskewed_synced', 10)
        self.raw_cloud_pub = self.create_publisher(PointCloud2, '/utlidar/cloud_synced', 10)
        self.cloud_base_pub = self.create_publisher(PointCloud2, '/utlidar/cloud_base_synced', 10)
        self.pointlio_imu_pub = self.create_publisher(Imu, '/utlidar/pointlio_imu_synced', 10)
        self.pointlio_cloud_pub = self.create_publisher(PointCloud2, '/utlidar/pointlio_cloud_synced', 10)
        
        # 과거 시간으로 들어오는 원본 Subscriber들
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, qos_profile)
        self.create_subscription(PointCloud2, '/utlidar/grid_map', self.grid_map_callback, 10)
        self.create_subscription(Imu, '/utlidar/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/utlidar/robot_pose', self.pose_callback, 10)
        self.create_subscription(PointCloud2, '/utlidar/cloud', self.raw_cloud_callback, 10)
        self.create_subscription(PointCloud2, '/utlidar/cloud_base', self.cloud_base_callback, 10)
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.cloud_callback, 10)

        self.publish_sensor_static_transforms()
        self.load_pointlio_calibration()
        self.init_pointlio_transform()

        self.get_logger().info("==============================================")
        self.get_logger().info("🚀 Time Sync Bridge 노드가 성공적으로 시작되었습니다!")
        self.get_logger().info("과거 데이터를 실시간 데이터(현재시간)로 덮어쓰는 중...")
        self.get_logger().info("==============================================")

    def load_pointlio_calibration(self):
        calib_data = {
            'acc_bias_x': 0.0,
            'acc_bias_y': 0.0,
            'acc_bias_z': 0.0,
            'ang_bias_x': 0.0,
            'ang_bias_y': 0.0,
            'ang_bias_z': 0.0,
            'ang_z2x_proj': 0.15,
            'ang_z2y_proj': -0.28,
        }
        try:
            calib_path = os.path.join(os.path.expanduser('~'), 'Desktop/imu_calib_data.yaml')
            with open(calib_path, 'r', encoding='utf-8') as calib_file:
                calib_data = yaml.load(calib_file, Loader=yaml.FullLoader)
            self.get_logger().info('imu_calib_data.yaml loaded for Point-LIO input transform')
        except Exception:
            self.get_logger().warn('imu_calib_data.yaml not found, using default Point-LIO transform values')

        self.acc_bias_x = calib_data['acc_bias_x']
        self.acc_bias_y = calib_data['acc_bias_y']
        self.acc_bias_z = calib_data['acc_bias_z']
        self.ang_bias_x = calib_data['ang_bias_x']
        self.ang_bias_y = calib_data['ang_bias_y']
        self.ang_bias_z = calib_data['ang_bias_z']
        self.ang_z2x_proj = calib_data['ang_z2x_proj']
        self.ang_z2y_proj = calib_data['ang_z2y_proj']

    def init_pointlio_transform(self):
        self.cam_offset = 0.046825
        self.cloud_pitch_rad = 2.8782025850555556
        self.imu_tilt_rad = 15.1 / 180.0 * 3.1415926
        self.cloud_cos = cos(self.cloud_pitch_rad)
        self.cloud_sin = sin(self.cloud_pitch_rad)
        self.imu_cos = cos(self.imu_tilt_rad)
        self.imu_sin = sin(self.imu_tilt_rad)
        self.x_filter_min = -0.7
        self.x_filter_max = -0.1
        self.y_filter_min = -0.3
        self.y_filter_max = 0.3
        self.z_filter_min = -0.6 - self.cam_offset
        self.z_filter_max = 0.0 - self.cam_offset

    def is_in_pointlio_filter_box(self, x, y, z):
        return (
            self.x_filter_min < x < self.x_filter_max and
            self.y_filter_min < y < self.y_filter_max and
            self.z_filter_min < z < self.z_filter_max
        )

    def publish_pointlio_cloud(self, msg):
        transformed_points = []
        for point in pc2.read_points_list(msg, skip_nans=False):
            values = list(point)
            if len(values) < 3:
                continue

            x = values[0]
            y = values[1]
            z = values[2]

            tx = self.cloud_cos * x + self.cloud_sin * z
            ty = y
            tz = -self.cloud_sin * x + self.cloud_cos * z - self.cam_offset

            if self.is_in_pointlio_filter_box(tx, ty, tz):
                continue

            values[0] = tx
            values[1] = ty
            values[2] = tz
            if len(values) > 4:
                values[4] = int(values[4])
            transformed_points.append(values)

        pointlio_cloud = pc2.create_cloud(msg.header, msg.fields, transformed_points)
        pointlio_cloud.header.stamp = msg.header.stamp
        pointlio_cloud.header.frame_id = 'body'
        pointlio_cloud.is_dense = msg.is_dense
        self.pointlio_cloud_pub.publish(pointlio_cloud)

    def publish_pointlio_imu(self, msg):
        x = msg.angular_velocity.x
        y = -msg.angular_velocity.y
        z = -msg.angular_velocity.z

        x2 = self.imu_cos * x - self.imu_sin * z
        y2 = y
        z2 = self.imu_sin * x + self.imu_cos * z

        x2 -= self.ang_bias_x
        y2 -= self.ang_bias_y
        z2 -= self.ang_bias_z
        x2 += self.ang_z2x_proj * z2
        y2 += self.ang_z2y_proj * z2

        pointlio_imu = Imu()
        pointlio_imu.header.stamp = msg.header.stamp
        pointlio_imu.header.frame_id = 'body'
        pointlio_imu.orientation.x = 0.0
        pointlio_imu.orientation.y = 0.0
        pointlio_imu.orientation.z = 0.0
        pointlio_imu.orientation.w = 1.0
        pointlio_imu.angular_velocity.x = x2
        pointlio_imu.angular_velocity.y = y2
        pointlio_imu.angular_velocity.z = z2
        pointlio_imu.linear_acceleration.x = 0.0
        pointlio_imu.linear_acceleration.y = 0.0
        pointlio_imu.linear_acceleration.z = 0.0

        self.pointlio_imu_pub.publish(pointlio_imu)

    def should_ignore_republished_tf(self, stamp):
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds

        # The bridge republishes on /tf itself, so only drop messages that are
        # already on the local wall-clock timeline. Raw robot TF can still be
        # "recent" in absolute epoch terms, so avoid hard-coded year cutoffs.
        return abs(now_ns - stamp_ns) < 5_000_000_000

    def apply_time_offset(self, old_stamp, stream_name):
        old_ns = old_stamp.sec * 1_000_000_000 + old_stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        
        # 딱 한 번만 오프셋 계산 (시스템 현재시간 - 2024년 센서시간)
        if stream_name not in self.time_offset_ns_by_stream:
            self.time_offset_ns_by_stream[stream_name] = now_ns - old_ns
            self.get_logger().info(
                f"⌚ {stream_name} 시간 브릿지 오프셋 고정 완료: "
                f"+{self.time_offset_ns_by_stream[stream_name] / 1e9:.3f} 초"
            )

        last_raw_ns = self.last_raw_stamp_ns.get(stream_name)
        if last_raw_ns is not None and old_ns + self.backward_jump_reset_ns < last_raw_ns:
            self.time_offset_ns_by_stream[stream_name] = now_ns - old_ns
            self.get_logger().warn(
                f"{stream_name} raw timestamp jumped backwards. "
                f"Resetting bridge offset to +{self.time_offset_ns_by_stream[stream_name] / 1e9:.3f} s"
            )

        new_ns = old_ns + self.time_offset_ns_by_stream[stream_name]

        last_synced_ns = self.last_synced_stamp_ns.get(stream_name)
        if last_synced_ns is not None and new_ns <= last_synced_ns:
            new_ns = last_synced_ns + 1

        if new_ns < 0 or new_ns > self.forward_limit_ns:
            self.get_logger().warn(
                f"{stream_name} synced timestamp overflow detected. Falling back to local time."
            )
            new_ns = now_ns

        self.last_raw_stamp_ns[stream_name] = old_ns
        self.last_synced_stamp_ns[stream_name] = new_ns
        
        # python의 int와 ROS2 Time 포맷 연결
        new_stamp_msg = rclpy.time.Time(nanoseconds=new_ns).to_msg()
        return new_stamp_msg

    def tf_callback(self, msg):
        if not msg.transforms: return
        if self.should_ignore_republished_tf(msg.transforms[0].header.stamp): return

        self.last_external_tf_monotonic = time.monotonic()
        for transform in msg.transforms:
            stream_name = f"tf:{transform.child_frame_id or 'unknown'}"
            transform.header.stamp = self.apply_time_offset(transform.header.stamp, stream_name)
        self.tf_pub.publish(msg)

    def tf_static_callback(self, msg):
        if not msg.transforms: return
        if self.should_ignore_republished_tf(msg.transforms[0].header.stamp): return

        now_msg = self.get_clock().now().to_msg()
        transforms_to_publish = []
        for transform in msg.transforms:
            child_frame_id = transform.child_frame_id or 'unknown'
            if child_frame_id in self.published_tf_static_children:
                continue
            transform.header.stamp = now_msg
            transforms_to_publish.append(transform)
            self.published_tf_static_children.add(child_frame_id)

        if transforms_to_publish:
            self.tf_static_pub.publish(TFMessage(transforms=transforms_to_publish))

    def grid_map_callback(self, msg):
        msg.header.stamp = self.apply_time_offset(msg.header.stamp, 'grid_map')
        self.grid_map_pub.publish(msg)

    def imu_callback(self, msg):
        msg.header.stamp = self.apply_time_offset(msg.header.stamp, 'imu')
        self.imu_pub.publish(msg)
        self.publish_pointlio_imu(msg)

    def odom_callback(self, msg):
        new_stamp = self.apply_time_offset(msg.header.stamp, 'odom')
        msg.header.stamp = new_stamp
        self.odom_pub.publish(msg)

        if self.should_publish_odom_tf(msg):
            transform = TransformStamped()
            transform.header.stamp = new_stamp
            transform.header.frame_id = msg.header.frame_id
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def pose_callback(self, msg):
        msg.header.stamp = self.apply_time_offset(msg.header.stamp, 'pose')
        self.pose_pub.publish(msg)

    def cloud_callback(self, msg):
        msg.header.stamp = self.apply_time_offset(msg.header.stamp, 'cloud_deskewed')
        self.cloud_pub.publish(msg)

    def raw_cloud_callback(self, msg):
        msg.header.stamp = self.apply_time_offset(msg.header.stamp, 'cloud')
        self.raw_cloud_pub.publish(msg)
        self.publish_pointlio_cloud(msg)

    def cloud_base_callback(self, msg):
        msg.header.stamp = self.apply_time_offset(msg.header.stamp, 'cloud_base')
        self.cloud_base_pub.publish(msg)

    def publish_sensor_static_transforms(self):
        # The raw Unitree topics use utlidar_* frame ids that are absent from the
        # default TF tree, so publish the required static sensor extrinsics here.
        self.static_tf_broadcaster.sendTransform([
            self.make_static_transform(
                parent='base_link',
                child='utlidar_imu',
                xyz=(-0.02557, 0.0, 0.04232),
                rpy=(0.0, 0.0, 0.0),
            ),
            self.make_static_transform(
                parent='base_link',
                child='utlidar_lidar',
                xyz=(0.28945, 0.0, -0.046825),
                rpy=(0.0, 2.8782, 0.0),
            ),
        ])

    def make_static_transform(self, parent, child, xyz, rpy):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]
        qx, qy, qz, qw = self.quaternion_from_euler(*rpy)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def quaternion_from_euler(self, roll, pitch, yaw):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5
        cr = cos(half_roll)
        sr = sin(half_roll)
        cp = cos(half_pitch)
        sp = sin(half_pitch)
        cy = cos(half_yaw)
        sy = sin(half_yaw)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def should_publish_odom_tf(self, msg):
        # RViz needs a live odom->base_link transform even when the robot
        # publishes only odometry and no dynamic TF.
        if not msg.header.frame_id or not msg.child_frame_id:
            return False
        if self.last_external_tf_monotonic is None:
            return True
        return (time.monotonic() - self.last_external_tf_monotonic) > self.external_tf_timeout_sec

def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncBridge()
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
