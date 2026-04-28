# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    model_number = LaunchConfiguration('model_number')

    model_number_arg = DeclareLaunchArgument(
        'model_number',
        default_value='S00V0000M1018',
        description='String for robot model version/number, please enter SxxVxxxxMxxxx'
    )

    bringup_dir = get_package_share_directory('ros2_description')

    urdf_filename = PythonExpression([
        "'kr2_robot_' + '", LaunchConfiguration('model_number'), "' + '.urdf'"
    ])

    urdf_path = PathJoinSubstitution([
        bringup_dir,  
        'urdf', 
        urdf_filename
    ])

    robot_description_content = Command(['cat ', urdf_path])
    robot_description = {'robot_description': robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ros2_description'),
            'launch',
            'robot_controller_gazebo.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'kr2', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )


    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    #          )

    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'kr2_a810'],
    #                     output='screen')


    # Den Pfad zum 'share'-Verzeichnis deines Pakets finden
    pkg_share = get_package_share_directory('ros2_description')

    # WICHTIG: Gazebo muss wissen, wo die Meshes liegen. 
    # Wir fügen das Verzeichnis über dem 'meshes' Ordner zum GZ_SIM_RESOURCE_PATH hinzu.
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + pkg_share
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = pkg_share

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        model_number_arg,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessStart(
                target_action=gz_spawn_entity,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
    ])

