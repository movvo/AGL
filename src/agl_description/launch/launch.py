import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = 'agl_description'

def generate_launch_description():
    agl_description_path = get_package_share_path('agl_description')
    default_model_path = agl_description_path / 'urdf/andresito.urdf'
    default_rviz_config_path = agl_description_path / 'rviz/agl.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    andresito = os.path.join(get_package_share_directory(PACKAGE_NAME), "urdf", "andresito.urdf")
    turtle_burg  = os.path.join(get_package_share_directory(PACKAGE_NAME), "urdf", "turtlebot3_waffle.urdf")

    # Launch arguments
    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='False', 
        choices=['True', 'False'],
        description='Parameter use_sim_time')

    # Launch arguments
    sim_time = LaunchConfiguration('use_sim_time')

    remaps=[
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    andresito = Node(
        name="Description",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[andresito],
        remappings=[
            *remaps,
            ("robot_description", "~/robot_description")
        ],
        condition=IfCondition(PythonExpression(['not ', sim_time])),
        output="screen",
        emulate_tty=True
    )

    turtle_burg = Node(
        name="Description",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[turtle_burg],
        remappings=[
            *remaps,
            ("robot_description", "~/robot_description")
        ],
        condition=IfCondition(sim_time),
        output="screen",
        emulate_tty=True
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        namespace="agl",
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        namespace="agl",
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        gui_arg,
        sim_time_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        andresito,
        turtle_burg,
        rviz_node
    ])