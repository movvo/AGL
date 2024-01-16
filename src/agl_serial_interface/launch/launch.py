from launch import LaunchDescription
from launch_ros.actions import Node

#if you run the launch file using ros2 launch after changing the parameter values, you need to rerun colcon build

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agl_serial_interface',
            executable='serial_server',
            name='serial_server_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                ('device', '/dev/ttyACM0'), #device we are trasmitting to & recieving messages from
                ('topic', 'TwoAngularSpeeds'),
                ('subs_topic', '/cmd_vel'),
                ('radius', 0.05),
                ('wheel_separation', 0.32)
            ]
        ),
    ])