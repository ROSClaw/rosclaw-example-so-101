import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateEcho(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_echo')
        self.declare_parameter('topic', '/follower/joint_states')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.create_subscription(JointState, topic, self._callback, 10)

    def _callback(self, msg: JointState) -> None:
        pairs = ', '.join(
            f'{name}={position:.3f}'
            for name, position in zip(msg.name, msg.position)
        )
        self.get_logger().info(pairs)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateEcho()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
