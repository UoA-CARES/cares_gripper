from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    gripper_config_launch_arg = DeclareLaunchArgument(
      'gripper_config', default_value=TextSubstitution(text='0')
    )

    return LaunchDescription([
        gripper_config_launch_arg,
        Node(
            package='gripper_control',
            namespace='gripper',
            executable='gripper_node',
            name='gripper_node',
            emulate_tty=True,
            parameters=[{
                'gripper_config': LaunchConfiguration('gripper_config')
            }]
        )
    ])