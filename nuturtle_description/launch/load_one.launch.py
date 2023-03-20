from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(name='use_jsp', default_value='true',
                              choices=['true', 'false'],
                              description='Enable joint_state_publisher_node'),
        DeclareLaunchArgument(name='model',
                              default_value=str(get_package_share_path('nuturtle_description')
                                                / 'urdf/turtlebot3_burger.urdf.xacro'),
                              description='Path to robot urdf file'),
        DeclareLaunchArgument(name='use_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Enable visualization in rviz'),
        DeclareLaunchArgument(name='color', default_value='purple',
                              choices=['purple', 'red', 'blue', 'green', ''],
                              description='Select the color of the turtlebot baselink'),
        SetLaunchConfiguration(name="rviz_color",
                               value=[FindPackageShare("nuturtle_description"),
                                      TextSubstitution(text="/config/basic_"),
                                      LaunchConfiguration("color"),
                                      TextSubstitution(text=".rviz")]),
        SetLaunchConfiguration(name='frameprefix',
                               value=[LaunchConfiguration("color"),
                                      TextSubstitution(text="/")]
                               ),
        SetLaunchConfiguration(name='fixedframe',
                               value=[LaunchConfiguration("color"),
                                      TextSubstitution(text="/base_link")]
                               ),
        Node(
            package='joint_state_publisher',
            namespace=LaunchConfiguration('color'),
            executable='joint_state_publisher',
            condition=LaunchConfigurationEquals('use_jsp', 'true')),
        Node(
            package='robot_state_publisher',
            namespace=LaunchConfiguration('color'),
            executable='robot_state_publisher',
            parameters=[{'robot_description':
                        Command([ExecutableInPackage("xacro", "xacro"), " ",
                                 PathJoinSubstitution(
                                 [FindPackageShare("nuturtle_description"),
                                  "urdf/turtlebot3_burger.urdf.xacro"]),
                                 " color:=",
                                 LaunchConfiguration('color')]
                                ), 'frame_prefix': LaunchConfiguration('frameprefix')}
                        ]
        ),
        Node(
            package='rviz2',
            namespace=LaunchConfiguration('color'),
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_color'), '-f',
                       LaunchConfiguration('fixedframe')],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown())
    ])
