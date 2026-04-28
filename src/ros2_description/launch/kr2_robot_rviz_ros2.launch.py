import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command, PythonExpression
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions


def generate_launch_description():
    model_number = LaunchConfiguration('model_number')

    model_number_arg = DeclareLaunchArgument(
        'model_number',
        default_value='S00V0000M0810',
        description='String for robot model version/number, please enter SxxVxxxxMxxxx'
    )


    rviz_config_dir = os.path.join(
            get_package_share_directory('ros2_description'),
            'launch',
            'kr2_robot_ros2.rviz')
    print(rviz_config_dir)

    #result = subprocess.run(["xacro", "-i", "install/kr2_robot_model/share/kr2_robot_model/urdf/kr2_robot_a810_tio.xacro"], stdout=subprocess.PIPE)
    #xml = result.stdout.decode('utf-8')

    bringup_dir = get_package_share_directory('ros2_description')
    # urdf_path = os.path.join(bringup_dir, 'urdf', model_number.String() + '.urdf')
    # xml = open(urdf_path).read()
    #print(xml)

    urdf_filename = PythonExpression([
        "'kr2_robot_' + '", LaunchConfiguration('model_number'), "' + '.urdf'"
    ])

    urdf_path = PathJoinSubstitution([bringup_dir, 'urdf', urdf_filename])
    xml = Command(['cat ', urdf_path])

    return LaunchDescription([
        model_number_arg,

        launch.actions.DeclareLaunchArgument(
            'robot_description',
            default_value=[xml, '_']),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description':xml, "publish_frequency":60.0}],
            output='screen'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            #parameters=[{'source_list':["set_joints"]}],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')


    ])

