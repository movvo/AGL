from launch import LaunchDescription
from launch_ros.actions import Node

#if you run the launch file using ros2 launch after changing the parameter values, you need to rerun colcon build

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agl_odometry',
            executable='odometry',
            name='odometry_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'subs_topic': '/TwoAngularSpeeds'}
            ]
        ),
    ])