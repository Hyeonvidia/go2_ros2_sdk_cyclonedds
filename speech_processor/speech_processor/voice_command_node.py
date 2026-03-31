#!/usr/bin/env python3

# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Voice Command Node for Go2 CycloneDDS

Subscribes to /stt (recognized text), interprets Korean/English voice commands,
and controls the robot via cmd_vel and Sport API.

Supported commands (Korean):
  앞으로 / 전진    → move forward
  뒤로 / 후진      → move backward
  왼쪽 / 좌회전    → turn left
  오른쪽 / 우회전   → turn right
  멈춰 / 정지      → stop
  앉아             → sit down (StandDown)
  일어서 / 일어나   → stand up (StandUp)
  빨리             → speed up
  천천히            → slow down
  인사             → hello gesture

Supported commands (English):
  forward / go     → move forward
  back / backward  → move backward
  left             → turn left
  right            → turn right
  stop             → stop
  sit / down       → sit down
  stand / up       → stand up
  fast             → speed up
  slow             → slow down
  hello            → hello gesture
"""

import json
import time
import datetime
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from unitree_api.msg import Request, RequestHeader, RequestIdentity, RequestLease, RequestPolicy


class VoiceCommandNode(Node):
    """Interprets voice commands and controls Go2 robot."""

    # Sport API command IDs
    API_STAND_UP = 1004
    API_STAND_DOWN = 1005
    API_RECOVER = 1006
    API_HELLO = 1016
    API_STOP_MOVE = 1003

    def __init__(self):
        super().__init__('voice_command_node')

        # Parameters
        self.declare_parameter('base_linear_speed', 0.3)
        self.declare_parameter('base_angular_speed', 0.5)
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('cmd_duration', 2.0)

        self._base_linear = self.get_parameter('base_linear_speed').value
        self._base_angular = self.get_parameter('base_angular_speed').value
        self._speed_step = self.get_parameter('speed_step').value
        self._max_linear = self.get_parameter('max_linear_speed').value
        self._cmd_duration = self.get_parameter('cmd_duration').value

        self._current_speed = self._base_linear

        # Publishers
        self._pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self._pub_sport = self.create_publisher(
            Request, 'api/sport/request', QoSProfile(depth=10))
        self._pub_feedback = self.create_publisher(String, '/voice_feedback', 10)

        # Subscriber
        self.create_subscription(String, '/stt', self._on_stt, 10)

        # Auto-stop timer
        self._last_cmd_time = 0.0
        self._is_moving = False
        self._stop_timer = self.create_timer(0.5, self._check_auto_stop)

        # Command mapping
        self._commands = self._build_command_map()

        self.get_logger().info(f'Voice Command Node initialized '
                               f'(speed={self._current_speed:.1f}m/s)')

    def _build_command_map(self):
        """Build keyword → action mapping."""
        return {
            # Forward
            '앞으로': ('move', 'forward'),
            '전진': ('move', 'forward'),
            'forward': ('move', 'forward'),
            'go': ('move', 'forward'),
            # Backward
            '뒤로': ('move', 'backward'),
            '후진': ('move', 'backward'),
            'back': ('move', 'backward'),
            'backward': ('move', 'backward'),
            # Left
            '왼쪽': ('move', 'left'),
            '좌회전': ('move', 'left'),
            'left': ('move', 'left'),
            # Right
            '오른쪽': ('move', 'right'),
            '우회전': ('move', 'right'),
            'right': ('move', 'right'),
            # Stop
            '멈춰': ('move', 'stop'),
            '정지': ('move', 'stop'),
            '스톱': ('move', 'stop'),
            'stop': ('move', 'stop'),
            # Sit
            '앉아': ('sport', self.API_STAND_DOWN),
            '앉으': ('sport', self.API_STAND_DOWN),
            'sit': ('sport', self.API_STAND_DOWN),
            'down': ('sport', self.API_STAND_DOWN),
            # Stand
            '일어서': ('sport', self.API_STAND_UP),
            '일어나': ('sport', self.API_STAND_UP),
            'stand': ('sport', self.API_STAND_UP),
            'up': ('sport', self.API_STAND_UP),
            # Hello
            '인사': ('sport', self.API_HELLO),
            'hello': ('sport', self.API_HELLO),
            # Speed
            '빨리': ('speed', 'up'),
            '빠르게': ('speed', 'up'),
            'fast': ('speed', 'up'),
            '천천히': ('speed', 'down'),
            '느리게': ('speed', 'down'),
            'slow': ('speed', 'down'),
        }

    def _on_stt(self, msg: String) -> None:
        """Process recognized speech text."""
        text = msg.data.strip().lower()
        if not text:
            return

        self.get_logger().info(f'Voice: "{text}"')

        # Find matching command
        for keyword, (action_type, action) in self._commands.items():
            if keyword in text:
                self._execute(action_type, action, keyword)
                return

        self.get_logger().info(f'Unknown command: "{text}"')
        self._publish_feedback(f'알 수 없는 명령: {text}')

    def _execute(self, action_type: str, action, keyword: str) -> None:
        """Execute a recognized command."""
        if action_type == 'move':
            self._execute_move(action)
        elif action_type == 'sport':
            self._execute_sport(action)
        elif action_type == 'speed':
            self._execute_speed(action)

        self.get_logger().info(f'Executed: {keyword} → {action_type}:{action}')

    def _execute_move(self, direction: str) -> None:
        """Publish cmd_vel for movement."""
        twist = Twist()

        if direction == 'forward':
            twist.linear.x = self._current_speed
            self._publish_feedback(f'전진 ({self._current_speed:.1f}m/s)')
        elif direction == 'backward':
            twist.linear.x = -self._current_speed
            self._publish_feedback(f'후진 ({self._current_speed:.1f}m/s)')
        elif direction == 'left':
            twist.angular.z = self._base_angular
            self._publish_feedback('좌회전')
        elif direction == 'right':
            twist.angular.z = -self._base_angular
            self._publish_feedback('우회전')
        elif direction == 'stop':
            self._is_moving = False
            self._publish_feedback('정지')

        self._pub_cmd_vel.publish(twist)

        if direction != 'stop':
            self._is_moving = True
            self._last_cmd_time = time.time()

    def _execute_sport(self, api_id: int) -> None:
        """Send Sport API command."""
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.id = int(
            datetime.datetime.now().timestamp() * 1000 % 2147483648
        ) + random.randint(0, 999)
        req.header.identity.api_id = api_id
        req.header.lease = RequestLease()
        req.header.lease.id = 0
        req.header.policy = RequestPolicy()
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = ''
        req.binary = []
        self._pub_sport.publish(req)

        names = {
            self.API_STAND_UP: '일어서기',
            self.API_STAND_DOWN: '앉기',
            self.API_HELLO: '인사',
            self.API_STOP_MOVE: '정지',
        }
        self._publish_feedback(names.get(api_id, f'API {api_id}'))

    def _execute_speed(self, direction: str) -> None:
        """Adjust movement speed."""
        if direction == 'up':
            self._current_speed = min(self._current_speed + self._speed_step,
                                      self._max_linear)
        else:
            self._current_speed = max(self._current_speed - self._speed_step, 0.1)

        self._publish_feedback(f'속도: {self._current_speed:.1f}m/s')

    def _check_auto_stop(self) -> None:
        """Auto-stop after cmd_duration seconds of no new voice command."""
        if self._is_moving and (time.time() - self._last_cmd_time > self._cmd_duration):
            twist = Twist()  # All zeros
            self._pub_cmd_vel.publish(twist)
            self._is_moving = False
            self.get_logger().info('Auto-stop: no new voice command')

    def _publish_feedback(self, text: str) -> None:
        """Publish feedback text (can be chained to TTS)."""
        msg = String()
        msg.data = text
        self._pub_feedback.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VoiceCommandNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
