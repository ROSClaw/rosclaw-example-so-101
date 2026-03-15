from __future__ import annotations

import argparse
import json
import sys
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from so101_pose_registry_interfaces.srv import DeleteNamedPose, GetNamedPose, ListNamedPoses, MoveToNamedPose, SaveCurrentPose


class PoseCli(Node):
    def __init__(self) -> None:
        super().__init__('so101_pose_cli')

    def call_service(self, client_type, service_name: str, request) -> Any:
        client = self.create_client(client_type, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'Service {service_name!r} is unavailable.')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Service {service_name!r} did not return a result.')
        return future.result()

    def wait_for_agent_state(self, topic_name: str = '/follower/agent_state', timeout_sec: float = 3.0) -> dict[str, Any]:
        result: dict[str, Any] | None = None

        def _handler(msg: String) -> None:
            nonlocal result
            result = json.loads(msg.data)

        sub = self.create_subscription(String, topic_name, _handler, 1)
        try:
            end_time = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
            while result is None and self.get_clock().now().nanoseconds < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(sub)

        if result is None:
            raise RuntimeError(f'Timed out waiting for {topic_name!r}.')
        return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='so101_pose_cli')
    subparsers = parser.add_subparsers(dest='command', required=True)

    subparsers.add_parser('stats')
    subparsers.add_parser('list')

    get_parser = subparsers.add_parser('get')
    get_parser.add_argument('name')

    save_parser = subparsers.add_parser('save')
    save_parser.add_argument('name')
    save_parser.add_argument('--overwrite', action='store_true')
    save_parser.add_argument('--gripper-open-fraction', type=float, default=None)

    delete_parser = subparsers.add_parser('delete')
    delete_parser.add_argument('name')

    move_parser = subparsers.add_parser('move')
    move_parser.add_argument('name')
    move_parser.add_argument('--duration-sec', type=float, default=3.0)

    return parser


def main(args=None) -> None:
    parser = build_parser()
    parsed = parser.parse_args(args=args)

    rclpy.init(args=None)
    node = PoseCli()
    try:
        if parsed.command == 'stats':
            print(json.dumps(node.wait_for_agent_state(), indent=2, sort_keys=True))
            return

        if parsed.command == 'list':
            request = ListNamedPoses.Request()
            response = node.call_service(ListNamedPoses, '/follower/list_named_poses', request)
            payload = {
                'workspace_bounds': {
                    'x_min': response.workspace_bounds.x_min,
                    'x_max': response.workspace_bounds.x_max,
                    'y_min': response.workspace_bounds.y_min,
                    'y_max': response.workspace_bounds.y_max,
                    'z_min': response.workspace_bounds.z_min,
                    'z_max': response.workspace_bounds.z_max,
                },
                'pose_names': [pose.name for pose in response.poses],
            }
            print(json.dumps(payload, indent=2, sort_keys=True))
            return

        if parsed.command == 'get':
            request = GetNamedPose.Request()
            request.name = parsed.name
            response = node.call_service(GetNamedPose, '/follower/get_named_pose', request)
            if not response.found:
                raise RuntimeError(response.message)
            payload = {
                'name': response.pose.name,
                'joint_names': list(response.pose.joint_names),
                'joint_positions': list(response.pose.joint_positions),
                'base_frame': response.pose.base_frame,
                'tool_frame': response.pose.tool_frame,
                'gripper_open_fraction': response.pose.gripper_open_fraction,
            }
            print(json.dumps(payload, indent=2, sort_keys=True))
            return

        if parsed.command == 'save':
            request = SaveCurrentPose.Request()
            request.name = parsed.name
            request.overwrite = bool(parsed.overwrite)
            if parsed.gripper_open_fraction is None:
                request.use_gripper_open_fraction = False
                request.gripper_open_fraction = 0.0
            else:
                request.use_gripper_open_fraction = True
                request.gripper_open_fraction = float(parsed.gripper_open_fraction)
            response = node.call_service(SaveCurrentPose, '/follower/save_current_pose', request)
            if not response.success:
                raise RuntimeError(response.message)
            print(response.message)
            return

        if parsed.command == 'delete':
            request = DeleteNamedPose.Request()
            request.name = parsed.name
            response = node.call_service(DeleteNamedPose, '/follower/delete_named_pose', request)
            if not response.success:
                raise RuntimeError(response.message)
            print(response.message)
            return

        if parsed.command == 'move':
            request = MoveToNamedPose.Request()
            request.name = parsed.name
            request.duration_sec = float(parsed.duration_sec)
            response = node.call_service(MoveToNamedPose, '/follower/move_to_named_pose', request)
            if not response.success:
                raise RuntimeError(response.message)
            print(response.message)
            return
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        rclpy.shutdown()
