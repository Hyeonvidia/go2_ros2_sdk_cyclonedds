#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Waypoint Navigation Node

Provides waypoint-based navigation using Nav2's FollowWaypoints action.
Supports saving/loading waypoints from YAML files and speech announcements.
"""

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import FollowWaypoints
from tf2_ros import Buffer, TransformListener, TransformException


class WaypointNavNode(Node):
    """Waypoint navigation with Nav2 FollowWaypoints action."""

    def __init__(self):
        super().__init__('waypoint_nav_node')

        # Parameters
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('announce_progress', True)

        self._waypoints_file = self.get_parameter('waypoints_file').value
        self._announce = self.get_parameter('announce_progress').value

        # Callback group for concurrent service handling
        self._cb_group = ReentrantCallbackGroup()

        # Action client for FollowWaypoints
        self._waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints')

        # TTS publisher
        self._pub_tts = self.create_publisher(String, '/tts', 10)

        # TF listener for saving current pose
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Services
        self.create_service(
            Trigger, '~/start_waypoint_nav',
            self._start_nav_cb, callback_group=self._cb_group)
        self.create_service(
            Trigger, '~/stop_waypoint_nav',
            self._stop_nav_cb, callback_group=self._cb_group)
        self.create_service(
            Trigger, '~/save_waypoint',
            self._save_waypoint_cb, callback_group=self._cb_group)
        self.create_service(
            Trigger, '~/clear_waypoints',
            self._clear_waypoints_cb, callback_group=self._cb_group)
        self.create_service(
            Trigger, '~/load_waypoints',
            self._load_waypoints_cb, callback_group=self._cb_group)

        # Waypoint list subscriber (from external sources like RViz)
        self.create_subscription(
            PoseArray, '~/waypoints', self._on_waypoints, 10)

        # State
        self._waypoints: list[PoseStamped] = []
        self._navigating = False
        self._goal_handle = None

        # Load waypoints from file if specified
        if self._waypoints_file:
            self._load_from_file(self._waypoints_file)

        self.get_logger().info('Waypoint Navigation Node initialized')
        self.get_logger().info(f'  Loaded {len(self._waypoints)} waypoints')

    # ------------------------------------------------------------------ #
    #  Service callbacks
    # ------------------------------------------------------------------ #

    def _start_nav_cb(self, request, response):
        """Start waypoint navigation."""
        if not self._waypoints:
            response.success = False
            response.message = 'No waypoints loaded'
            return response

        if self._navigating:
            response.success = False
            response.message = 'Already navigating'
            return response

        if not self._waypoint_client.server_is_ready():
            response.success = False
            response.message = 'FollowWaypoints action server not available'
            return response

        # Send waypoints
        goal = FollowWaypoints.Goal()
        goal.poses = list(self._waypoints)

        self._navigating = True
        count = len(self._waypoints)
        self._speak(f'웨이포인트 내비게이션을 시작합니다. 총 {count}개의 웨이포인트입니다.')
        self.get_logger().info(f'Starting waypoint navigation with {count} waypoints')

        future = self._waypoint_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

        response.success = True
        response.message = f'Navigation started with {count} waypoints'
        return response

    def _stop_nav_cb(self, request, response):
        """Cancel current waypoint navigation."""
        if not self._navigating or self._goal_handle is None:
            response.success = False
            response.message = 'Not currently navigating'
            return response

        self._goal_handle.cancel_goal_async()
        self._navigating = False
        self._speak('웨이포인트 내비게이션이 취소되었습니다.')
        response.success = True
        response.message = 'Navigation cancelled'
        return response

    def _save_waypoint_cb(self, request, response):
        """Save current robot pose as a waypoint."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except TransformException as e:
            response.success = False
            response.message = f'TF lookup failed: {e}'
            return response

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = transform.transform.rotation

        self._waypoints.append(pose)
        idx = len(self._waypoints)

        # Auto-save to file if configured
        if self._waypoints_file:
            self._save_to_file(self._waypoints_file)

        self.get_logger().info(
            f'Saved waypoint {idx}: ({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f})')
        self._speak(f'웨이포인트 {idx}번이 저장되었습니다.')

        response.success = True
        response.message = (
            f'Waypoint {idx} saved at '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        return response

    def _clear_waypoints_cb(self, request, response):
        """Clear all waypoints."""
        self._waypoints.clear()
        self.get_logger().info('All waypoints cleared')

        response.success = True
        response.message = 'All waypoints cleared'
        return response

    def _load_waypoints_cb(self, request, response):
        """Load waypoints from the configured YAML file."""
        if not self._waypoints_file:
            response.success = False
            response.message = 'No waypoints_file parameter set'
            return response

        if self._load_from_file(self._waypoints_file):
            response.success = True
            response.message = f'Loaded {len(self._waypoints)} waypoints'
        else:
            response.success = False
            response.message = f'Failed to load from {self._waypoints_file}'
        return response

    # ------------------------------------------------------------------ #
    #  Subscriber callback
    # ------------------------------------------------------------------ #

    def _on_waypoints(self, msg: PoseArray):
        """Receive waypoints from external source (e.g. RViz PoseArray)."""
        self._waypoints.clear()
        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = pose
            self._waypoints.append(ps)

        self.get_logger().info(f'Received {len(self._waypoints)} waypoints from topic')

    # ------------------------------------------------------------------ #
    #  Action callbacks
    # ------------------------------------------------------------------ #

    def _goal_response_cb(self, future):
        """Handle goal acceptance/rejection."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._navigating = False
            self._speak('웨이포인트 내비게이션이 거부되었습니다.')
            self.get_logger().warn('Waypoint navigation goal rejected')
            return

        self.get_logger().info('Waypoint navigation goal accepted')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        """Handle navigation feedback — announce waypoint progress."""
        feedback = feedback_msg.feedback
        current_wp = feedback.current_waypoint

        self.get_logger().info(
            f'Moving to waypoint {current_wp + 1}/{len(self._waypoints)}')

        if self._announce:
            self._speak(f'웨이포인트 {current_wp + 1}번으로 이동 중입니다.')

    def _result_cb(self, future):
        """Handle navigation result."""
        result = future.result().result
        self._navigating = False
        self._goal_handle = None

        missed = result.missed_waypoints
        total = len(self._waypoints)

        if not missed:
            self._speak('모든 웨이포인트를 완료했습니다.')
            self.get_logger().info(f'All {total} waypoints completed successfully')
        else:
            missed_str = ', '.join(str(i + 1) for i in missed)
            self._speak(f'웨이포인트 내비게이션이 완료되었습니다. {len(missed)}개 실패.')
            self.get_logger().warn(
                f'Navigation done. Missed waypoints: {missed_str}')

    # ------------------------------------------------------------------ #
    #  YAML file I/O
    # ------------------------------------------------------------------ #

    def _load_from_file(self, path: str) -> bool:
        """Load waypoints from a YAML file."""
        if not os.path.isfile(path):
            self.get_logger().warn(f'Waypoints file not found: {path}')
            return False

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            if not data or 'waypoints' not in data:
                self.get_logger().warn(f'No waypoints key in {path}')
                return False

            self._waypoints.clear()
            for wp in data['waypoints']:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(wp.get('x', 0.0))
                pose.pose.position.y = float(wp.get('y', 0.0))
                pose.pose.position.z = 0.0

                yaw = float(wp.get('yaw', 0.0))
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

                self._waypoints.append(pose)

            self.get_logger().info(
                f'Loaded {len(self._waypoints)} waypoints from {path}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
            return False

    def _save_to_file(self, path: str) -> bool:
        """Save current waypoints to a YAML file."""
        try:
            waypoints_data = []
            for pose in self._waypoints:
                # Extract yaw from quaternion
                qz = pose.pose.orientation.z
                qw = pose.pose.orientation.w
                yaw = 2.0 * math.atan2(qz, qw)

                waypoints_data.append({
                    'x': round(pose.pose.position.x, 3),
                    'y': round(pose.pose.position.y, 3),
                    'yaw': round(yaw, 4),
                })

            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            with open(path, 'w') as f:
                yaml.dump({'waypoints': waypoints_data}, f, default_flow_style=False)

            self.get_logger().info(f'Saved {len(waypoints_data)} waypoints to {path}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error saving waypoints: {e}')
            return False

    # ------------------------------------------------------------------ #
    #  TTS helper
    # ------------------------------------------------------------------ #

    def _speak(self, text: str):
        """Publish text to /tts for speech synthesis."""
        if not self._announce:
            return
        msg = String()
        msg.data = text
        self._pub_tts.publish(msg)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    try:
        node = WaypointNavNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
