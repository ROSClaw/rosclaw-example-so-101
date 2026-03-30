from __future__ import annotations

import math
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray

from so101_ros2_bridge import CALIBRATION_BASE_DIR
from so101_ros2_bridge.bridge import FollowerBridge

from .follower_bridge_runtime import coerce_max_relative_target, normalize_calibration_dir


class LocalFollowerBridge(FollowerBridge):
    def read_parameters(self) -> dict[str, Any]:
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('id', 'Tzili')
        self.declare_parameter('calibration_dir', str(CALIBRATION_BASE_DIR))
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('max_relative_target', 0.0)
        self.declare_parameter('disable_torque_on_disconnect', True)
        self.declare_parameter('publish_rate', 30.0)

        return {
            'port': str(self.get_parameter('port').value),
            'id': str(self.get_parameter('id').value),
            'calibration_dir': normalize_calibration_dir(self.get_parameter('calibration_dir').value),
            'use_degrees': bool(self.get_parameter('use_degrees').value),
            'max_relative_target': coerce_max_relative_target(
                self.get_parameter('max_relative_target').value
            ),
            'disable_torque_on_disconnect': bool(
                self.get_parameter('disable_torque_on_disconnect').value
            ),
            'publish_rate': float(self.get_parameter('publish_rate').value),
        }

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(self.JOINT_NAMES):
            self.get_logger().error(
                f'Received command with {len(msg.data)} joints, but expected {len(self.JOINT_NAMES)}.'
            )
            return

        target_positions = {}
        for i, joint in enumerate(self.JOINT_NAMES):
            if joint != 'gripper' and self.use_degrees:
                target_positions[f'{joint}.pos'] = math.degrees(msg.data[i])
            else:
                target_positions[f'{joint}.pos'] = (msg.data[i] / math.pi) * 100.0

        try:
            self.robot.send_action(target_positions)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to send commands to robot ({type(exc).__name__}): {exc}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LocalFollowerBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
