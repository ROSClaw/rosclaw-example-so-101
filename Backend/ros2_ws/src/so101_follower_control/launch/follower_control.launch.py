import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('so101_follower_control')
    bridge_share = get_package_share_directory('so101_ros2_bridge')
    controller_share = get_package_share_directory('so101_controller')
    description_share = get_package_share_directory('so101_description')

    port = LaunchConfiguration('port')
    robot_id = LaunchConfiguration('id')
    calibration_dir = LaunchConfiguration('calibration_dir')
    use_degrees = LaunchConfiguration('use_degrees')
    max_relative_target = LaunchConfiguration('max_relative_target')
    disable_torque_on_disconnect = LaunchConfiguration('disable_torque_on_disconnect')
    publish_rate = LaunchConfiguration('publish_rate')
    model = LaunchConfiguration('model')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    cartesian_goal_enabled = LaunchConfiguration('cartesian_goal_enabled')
    cartesian_goal_topic = LaunchConfiguration('cartesian_goal_topic')
    cartesian_goal_duration_sec = LaunchConfiguration('cartesian_goal_duration_sec')
    cartesian_base_frame = LaunchConfiguration('cartesian_base_frame')
    cartesian_tool_frame = LaunchConfiguration('cartesian_tool_frame')
    agent_bridge_enabled = LaunchConfiguration('agent_bridge_enabled')
    agent_state_topic = LaunchConfiguration('agent_state_topic')
    agent_state_publish_rate = LaunchConfiguration('agent_state_publish_rate')
    gripper_command_topic = LaunchConfiguration('gripper_command_topic')
    open_gripper_service = LaunchConfiguration('open_gripper_service')
    close_gripper_service = LaunchConfiguration('close_gripper_service')
    gripper_closed_position = LaunchConfiguration('gripper_closed_position')
    gripper_open_position = LaunchConfiguration('gripper_open_position')
    gripper_max_effort = LaunchConfiguration('gripper_max_effort')

    follower_rsp_group = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(description_share, 'launch', 'rsp.launch.py')
                ),
                launch_arguments={
                    'model': model,
                    'mode': 'real',
                    'type': 'follower',
                }.items(),
            ),
        ],
    )

    follower_bridge_node = Node(
        package='so101_ros2_bridge',
        executable='follower_ros2_node',
        name='so101_follower_ros2_bridge',
        namespace='follower',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'follower_bridge.yaml'),
            {
                'port': port,
                'id': robot_id,
                'calibration_dir': calibration_dir,
                'use_degrees': use_degrees,
                'max_relative_target': max_relative_target,
                'disable_torque_on_disconnect': disable_torque_on_disconnect,
                'publish_rate': publish_rate,
                'type': 'follower',
            },
        ],
    )

    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller_manager.launch.py')
        ),
        launch_arguments={'type': 'follower'}.items(),
    )

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'so101_controllers.launch.py')
        ),
        launch_arguments={'type': 'follower'}.items(),
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager_launch])
    delayed_spawn_controllers = TimerAction(period=6.0, actions=[spawn_controllers_launch])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='so101_follower_control_rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen',
    )

    cartesian_goal_node = Node(
        package='so101_follower_control',
        executable='cartesian_goal_node',
        name='cartesian_goal_node',
        namespace='follower',
        output='screen',
        condition=IfCondition(cartesian_goal_enabled),
        parameters=[
            {
                'goal_topic': cartesian_goal_topic,
                'goal_duration_sec': cartesian_goal_duration_sec,
                'cartesian_base_frame': cartesian_base_frame,
                'cartesian_tool_frame': cartesian_tool_frame,
            }
        ],
    )

    agent_bridge_node = Node(
        package='so101_follower_control',
        executable='agent_bridge_node',
        name='agent_bridge_node',
        namespace='follower',
        output='screen',
        condition=IfCondition(agent_bridge_enabled),
        parameters=[
            {
                'state_topic': agent_state_topic,
                'state_publish_rate': agent_state_publish_rate,
                'gripper_command_topic': gripper_command_topic,
                'open_gripper_service': open_gripper_service,
                'close_gripper_service': close_gripper_service,
                'gripper_closed_position': gripper_closed_position,
                'gripper_open_position': gripper_open_position,
                'gripper_max_effort': gripper_max_effort,
                'cartesian_base_frame': cartesian_base_frame,
                'cartesian_tool_frame': cartesian_tool_frame,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
            DeclareLaunchArgument('id', default_value='Tzili'),
            DeclareLaunchArgument(
                'calibration_dir',
                default_value=os.path.join(bridge_share, 'config', 'calibration'),
            ),
            DeclareLaunchArgument('use_degrees', default_value='true'),
            DeclareLaunchArgument('max_relative_target', default_value='10'),
            DeclareLaunchArgument('disable_torque_on_disconnect', default_value='true'),
            DeclareLaunchArgument('publish_rate', default_value='50.0'),
            DeclareLaunchArgument(
                'model',
                default_value=os.path.join(
                    description_share,
                    'urdf',
                    'so101_new_calib.urdf.xacro',
                ),
            ),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=os.path.join(package_share, 'rviz', 'follower_control.rviz'),
            ),
            DeclareLaunchArgument('cartesian_goal_enabled', default_value='true'),
            DeclareLaunchArgument('cartesian_goal_topic', default_value='cartesian_goal'),
            DeclareLaunchArgument('cartesian_goal_duration_sec', default_value='3.0'),
            DeclareLaunchArgument('cartesian_base_frame', default_value='follower/base_link'),
            DeclareLaunchArgument(
                'cartesian_tool_frame',
                default_value='follower/gripper_frame_link',
            ),
            DeclareLaunchArgument('agent_bridge_enabled', default_value='true'),
            DeclareLaunchArgument('agent_state_topic', default_value='agent_state'),
            DeclareLaunchArgument('agent_state_publish_rate', default_value='2.0'),
            DeclareLaunchArgument('gripper_command_topic', default_value='gripper_command'),
            DeclareLaunchArgument('open_gripper_service', default_value='open_gripper'),
            DeclareLaunchArgument('close_gripper_service', default_value='close_gripper'),
            DeclareLaunchArgument('gripper_closed_position', default_value='0.0'),
            DeclareLaunchArgument('gripper_open_position', default_value='1.0'),
            DeclareLaunchArgument('gripper_max_effort', default_value='10.0'),
            follower_rsp_group,
            follower_bridge_node,
            cartesian_goal_node,
            agent_bridge_node,
            delayed_controller_manager,
            delayed_spawn_controllers,
            rviz_node,
        ]
    )
