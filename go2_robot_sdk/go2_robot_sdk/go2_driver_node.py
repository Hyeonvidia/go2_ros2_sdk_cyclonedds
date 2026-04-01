# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Go2 CycloneDDS Driver Node

Subscribes to the Go2 EDU robot's native DDS topics using the official
unitree_go message types. Republishes as standard ROS2 topics.

Native Go2 DDS topics (published by the robot):
  - rt/sportmodestate       → unitree_go/msg/SportModeState
  - rt/lf/lowstate           → unitree_go/msg/LowState
  - rt/utlidar/robot_pose    → geometry_msgs/PoseStamped
  - rt/utlidar/cloud         → sensor_msgs/PointCloud2

Republished ROS2 topics:
  - joint_states             → sensor_msgs/JointState
  - go2_states               → unitree_go/msg/SportModeState (pass-through)
  - imu                      → sensor_msgs/Imu (standard)
  - odom                     → nav_msgs/Odometry
  - point_cloud2             → sensor_msgs/PointCloud2
  - TF (odom → base_link)
"""

import math
import json
import time
import datetime
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState, Imu, PointCloud2
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from unitree_go.msg import SportModeState, LowState
from unitree_api.msg import Request, RequestHeader, RequestIdentity, RequestLease, RequestPolicy


class Go2DriverNode(Node):
    """CycloneDDS driver node for Unitree Go2 EDU robot."""

    # Joint name mapping: Go2 motor indices → URDF joint names
    JOINT_NAMES = [
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    ]

    # Motor index mapping: FL(3,4,5), FR(0,1,2), RL(9,10,11), RR(6,7,8)
    MOTOR_MAP = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

    # Sport API command IDs
    SPORT_CMD_MOVE = 1008
    SPORT_CMD_BALANCE_STAND = 1002

    # Velocity safety limits (m/s and rad/s)
    MAX_VX = 0.8    # forward/backward
    MAX_VY = 0.3    # lateral
    MAX_WZ = 1.0    # yaw rate

    # Minimum interval between commands (seconds) — 20 Hz
    CMD_VEL_MIN_INTERVAL = 0.05

    def __init__(self):
        super().__init__('go2_driver_node')

        # --- QoS profiles ---
        # Go2 publishes SportModeState/LowState with RELIABLE QoS
        self._robot_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_qos = QoSProfile(depth=10)

        # --- TF broadcaster ---
        self._tf_broadcaster = TransformBroadcaster(self, qos=QoSProfile(depth=10))

        # --- Publishers (standard ROS2 topics) ---
        self._pub_joint = self.create_publisher(
            JointState, 'joint_states', self._pub_qos)
        self._pub_imu = self.create_publisher(
            Imu, 'imu', self._pub_qos)
        self._pub_odom = self.create_publisher(
            Odometry, 'odom', self._pub_qos)

        # --- LiDAR cloud QoS bridge ---
        # cloud_deskewed: 11k pts, 15Hz, already in odom frame (matches WebRTC approach)
        self._pub_cloud = self.create_publisher(
            PointCloud2, 'go2/cloud', qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, 'utlidar/cloud_deskewed',
            self._on_cloud_deskewed, self._lidar_qos)

        # --- Subscribers (Go2 native DDS topics using unitree_go types) ---
        self.create_subscription(
            SportModeState, 'sportmodestate',
            self._on_sport_mode_state, self._robot_qos)
        self.create_subscription(
            SportModeState, 'lf/sportmodestate',
            self._on_sport_mode_state, self._robot_qos)
        self.create_subscription(
            LowState, 'lowstate',
            self._on_low_state, self._robot_qos)
        self.create_subscription(
            LowState, 'lf/lowstate',
            self._on_low_state, self._robot_qos)

        # --- ROS2 parameters ---
        self.declare_parameter('z_offset', 0.07)
        self._z_offset = self.get_parameter('z_offset').value

        self.declare_parameter('watchdog_timeout_sec', 5.0)
        self._watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value

        self.declare_parameter('enable_cmd_vel', False)
        self._enable_cmd_vel = self.get_parameter('enable_cmd_vel').value

        # --- cmd_vel → Go2 sport API (only when explicitly enabled) ---
        self._pub_sport_request = None
        self._last_cmd_vel_time = 0.0
        if self._enable_cmd_vel:
            self._pub_sport_request = self.create_publisher(
                Request, 'api/sport/request', QoSProfile(depth=10))
            self.create_subscription(
                Twist, 'cmd_vel', self._on_cmd_vel, QoSProfile(depth=10))
            self.get_logger().warn('cmd_vel forwarding ENABLED — robot will move!')
            # Auto-activate sport mode once robot data is received
            self._sport_mode_activated = False
            self._sport_mode_timer = None
        else:
            self.get_logger().warn(
                'cmd_vel forwarding DISABLED. If Nav2 is running, '
                'the robot will NOT move. Set enable_cmd_vel:=true to enable.')

        # --- Health monitoring ---
        self._got_data = False
        self._last_data_time = None
        self._data_lost = False
        self._watchdog_timer = self.create_timer(1.0, self._watchdog_callback)

        self.get_logger().info('Go2 CycloneDDS driver node started')
        self.get_logger().info('Waiting for Go2 DDS topics on ethernet...')

    # ------------------------------------------------------------------ #
    #  DDS callbacks — robot → ROS2
    # ------------------------------------------------------------------ #

    def _on_sport_mode_state(self, msg: SportModeState) -> None:
        """Process SportModeState from Go2 → publish IMU, Odom, TF."""
        self._last_data_time = self.get_clock().now()
        if self._data_lost:
            self._data_lost = False
            self.get_logger().warn('Go2 data stream recovered')
        if not self._got_data:
            self._got_data = True
            self.get_logger().info('Receiving data from Go2!')
            # Activate sport mode automatically after first data received
            if self._enable_cmd_vel and not self._sport_mode_activated:
                self._sport_mode_activated = True
                self._sport_mode_timer = self.create_timer(
                    2.0, self._activate_sport_mode)

        now = self.get_clock().now().to_msg()

        # --- IMU (standard sensor_msgs/Imu) ---
        imu = msg.imu_state
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu'
        # Go2 quaternion: [w, x, y, z]
        imu_msg.orientation.w = float(imu.quaternion[0])
        imu_msg.orientation.x = float(imu.quaternion[1])
        imu_msg.orientation.y = float(imu.quaternion[2])
        imu_msg.orientation.z = float(imu.quaternion[3])
        imu_msg.angular_velocity.x = float(imu.gyroscope[0])
        imu_msg.angular_velocity.y = float(imu.gyroscope[1])
        imu_msg.angular_velocity.z = float(imu.gyroscope[2])
        imu_msg.linear_acceleration.x = float(imu.accelerometer[0])
        imu_msg.linear_acceleration.y = float(imu.accelerometer[1])
        imu_msg.linear_acceleration.z = float(imu.accelerometer[2])
        self._pub_imu.publish(imu_msg)

        # --- Odometry from sport mode position/quaternion ---
        quat = imu.quaternion  # [w, x, y, z]
        pos = msg.position     # [x, y, z]
        self.get_logger().info(f'Odom: pos={pos}, quat={quat}', throttle_duration_sec=2.0)

        vals = list(pos) + list(quat)
        try:
            float_vals = [float(v) for v in vals]
            if sum(math.isnan(v) or math.isinf(v) for v in float_vals) > 0:
                return
        except (ValueError, TypeError):
            return

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2]) + self._z_offset
        odom.pose.pose.orientation.w = float(quat[0])
        odom.pose.pose.orientation.x = float(quat[1])
        odom.pose.pose.orientation.y = float(quat[2])
        odom.pose.pose.orientation.z = float(quat[3])
        self._pub_odom.publish(odom)

        # --- TF: odom → base_link ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2]) + self._z_offset
        t.transform.rotation.w = float(quat[0])
        t.transform.rotation.x = float(quat[1])
        t.transform.rotation.y = float(quat[2])
        t.transform.rotation.z = float(quat[3])
        self._tf_broadcaster.sendTransform(t)

    def _on_low_state(self, msg: LowState) -> None:
        """Process LowState from Go2 → publish JointState."""
        self._last_data_time = self.get_clock().now()
        if self._data_lost:
            self._data_lost = False
            self.get_logger().warn('Go2 data stream recovered')
        if not self._got_data:
            self._got_data = True
            self.get_logger().info('Receiving data from Go2!')

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.JOINT_NAMES

        positions = []
        velocities = []
        efforts = []
        for idx in self.MOTOR_MAP:
            motor = msg.motor_state[idx]
            positions.append(float(motor.q))
            velocities.append(float(motor.dq))
            efforts.append(float(motor.tau_est))

        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        self._pub_joint.publish(joint_state)

    # ------------------------------------------------------------------ #
    #  LiDAR cloud relay (QoS bridge)
    # ------------------------------------------------------------------ #

    def _on_cloud_deskewed(self, msg: PointCloud2) -> None:
        """Relay deskewed cloud (already in odom frame, 11k pts) as QoS bridge.
        Matches WebRTC approach: cloud in odom frame, TF handles base_link transform."""
        self._pub_cloud.publish(msg)

    # ------------------------------------------------------------------ #
    #  Command callbacks — ROS2 → robot
    # ------------------------------------------------------------------ #

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Convert cmd_vel (Twist) to Go2 sport API Move command via DDS."""
        if not self._enable_cmd_vel or self._pub_sport_request is None:
            return

        now = time.monotonic()
        if now - self._last_cmd_vel_time < self.CMD_VEL_MIN_INTERVAL:
            return
        self._last_cmd_vel_time = now

        vx = max(-self.MAX_VX, min(self.MAX_VX, msg.linear.x))
        vy = max(-self.MAX_VY, min(self.MAX_VY, msg.linear.y))
        wz = max(-self.MAX_WZ, min(self.MAX_WZ, msg.angular.z))

        self._publish_move_command(vx, vy, wz)

    def _activate_sport_mode(self) -> None:
        """Send BalanceStand command once to activate sport mode for API control."""
        if self._pub_sport_request is None:
            return
        self.get_logger().info('Activating sport mode (BalanceStand)...')
        self._publish_sport_command(self.SPORT_CMD_BALANCE_STAND, '{}')
        # Cancel the timer — one-shot only
        if self._sport_mode_timer is not None:
            self._sport_mode_timer.cancel()
            self._sport_mode_timer = None

    def _publish_sport_command(self, api_id: int, parameter: str) -> None:
        """Build and publish a unitree_api/Request for any Sport API command."""
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.id = self._generate_request_id()
        req.header.identity.api_id = api_id
        req.header.lease = RequestLease()
        req.header.lease.id = 0
        req.header.policy = RequestPolicy()
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = parameter
        req.binary = []
        self._pub_sport_request.publish(req)

    def _publish_move_command(self, vx: float, vy: float, wz: float) -> None:
        """Build and publish a unitree_api/Request for Move (API 1008)."""
        param = json.dumps({'x': round(vx, 3), 'y': round(vy, 3), 'z': round(wz, 3)})
        self._publish_sport_command(self.SPORT_CMD_MOVE, param)

    @staticmethod
    def _generate_request_id() -> int:
        """Generate unique request ID from timestamp + random component."""
        ts = int(datetime.datetime.now().timestamp() * 1000 % 2147483648)
        return ts + random.randint(0, 999)

    # ------------------------------------------------------------------ #
    #  Health monitoring
    # ------------------------------------------------------------------ #

    def _watchdog_callback(self) -> None:
        """Periodic check for data timeout."""
        if self._last_data_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_data_time).nanoseconds / 1e9
        if elapsed > self._watchdog_timeout and not self._data_lost:
            self._data_lost = True
            self.get_logger().warn(
                f'No data from Go2 for {elapsed:.1f}s '
                f'(timeout={self._watchdog_timeout}s). '
                'Check ethernet connection and robot power.')



def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = Go2DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._enable_cmd_vel and node._pub_sport_request is not None:
            node.get_logger().info('Shutdown: sending zero velocity to Go2')
            node._publish_move_command(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
