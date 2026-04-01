#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Nav2 Status Node

Monitors Nav2 lifecycle state and navigation goal status,
publishing Korean speech announcements via the /tts topic.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatusArray, GoalStatus
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from std_msgs.msg import String
from std_srvs.srv import Trigger


# Korean speech messages
MESSAGES_KO = {
    'nav2_active': '자율 내비게이션이 활성화 되었습니다',
    'nav2_inactive': '자율 내비게이션이 비활성화 되었습니다',
    'goal_accepted': '목표 지점으로 이동을 시작합니다',
    'goal_succeeded': '목표 지점에 도착했습니다',
    'goal_aborted': '내비게이션이 실패했습니다',
    'goal_canceled': '내비게이션이 취소되었습니다',
}

MESSAGES_EN = {
    'nav2_active': 'Autonomous navigation is now active',
    'nav2_inactive': 'Autonomous navigation is inactive',
    'goal_accepted': 'Moving to target position',
    'goal_succeeded': 'Arrived at destination',
    'goal_aborted': 'Navigation failed',
    'goal_canceled': 'Navigation canceled',
}


class Nav2StatusNode(Node):
    """Monitors Nav2 state and announces status changes via TTS."""

    def __init__(self):
        super().__init__('nav2_status_node')

        # Parameters
        self.declare_parameter('check_interval', 5.0)
        self.declare_parameter('debounce_sec', 10.0)
        self.declare_parameter('language', 'ko')

        self._check_interval = self.get_parameter('check_interval').value
        self._debounce_sec = self.get_parameter('debounce_sec').value
        lang = self.get_parameter('language').value
        self._messages = MESSAGES_KO if lang == 'ko' else MESSAGES_EN

        # TTS publisher
        self._pub_tts = self.create_publisher(String, '/tts', 10)

        # Navigation goal status subscriptions
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._on_nav_to_pose_status, 10)
        self.create_subscription(
            GoalStatusArray,
            '/navigate_through_poses/_action/status',
            self._on_nav_through_poses_status, 10)

        # Lifecycle check via action server availability
        self._nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self._nav2_active = None  # unknown initially
        self._lifecycle_timer = self.create_timer(
            self._check_interval, self._check_nav2_lifecycle)

        # State tracking
        self._known_goals = {}  # goal_id bytes -> last announced status
        self._last_announce_time = {}  # message_key -> timestamp

        self.get_logger().info('Nav2 Status Node initialized')

    def _check_nav2_lifecycle(self):
        """Periodically check if Nav2 action server is available."""
        is_active = self._nav_action_client.server_is_ready()

        if self._nav2_active is None:
            # First check — announce current state
            self._nav2_active = is_active
            key = 'nav2_active' if is_active else 'nav2_inactive'
            self._announce(key)
        elif is_active != self._nav2_active:
            # State changed
            self._nav2_active = is_active
            key = 'nav2_active' if is_active else 'nav2_inactive'
            self._announce(key)

    def _on_nav_to_pose_status(self, msg: GoalStatusArray):
        """Handle NavigateToPose goal status updates."""
        self._process_goal_statuses(msg)

    def _on_nav_through_poses_status(self, msg: GoalStatusArray):
        """Handle NavigateThroughPoses goal status updates."""
        self._process_goal_statuses(msg)

    def _process_goal_statuses(self, msg: GoalStatusArray):
        """Process goal status array and announce changes."""
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            current_status = status.status
            prev_status = self._known_goals.get(goal_id)

            if prev_status == current_status:
                continue

            self._known_goals[goal_id] = current_status

            if current_status == GoalStatus.STATUS_ACCEPTED:
                self._announce('goal_accepted')
            elif current_status == GoalStatus.STATUS_SUCCEEDED:
                self._announce('goal_succeeded')
                self._cleanup_goal(goal_id)
            elif current_status == GoalStatus.STATUS_ABORTED:
                self._announce('goal_aborted')
                self._cleanup_goal(goal_id)
            elif current_status == GoalStatus.STATUS_CANCELED:
                self._announce('goal_canceled')
                self._cleanup_goal(goal_id)

    def _cleanup_goal(self, goal_id: bytes):
        """Remove completed goal from tracking after a delay."""
        # Keep for debounce period, then allow cleanup by dict size limit
        if len(self._known_goals) > 50:
            # Trim old entries
            oldest_keys = list(self._known_goals.keys())[:25]
            for k in oldest_keys:
                del self._known_goals[k]

    def _announce(self, message_key: str):
        """Publish a TTS message with debounce protection."""
        now = time.monotonic()
        last = self._last_announce_time.get(message_key, 0.0)

        if now - last < self._debounce_sec:
            return

        self._last_announce_time[message_key] = now
        text = self._messages.get(message_key, '')
        if not text:
            return

        msg = String()
        msg.data = text
        self._pub_tts.publish(msg)
        self.get_logger().info(f'Announced: {text}')


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    try:
        node = Nav2StatusNode()
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
