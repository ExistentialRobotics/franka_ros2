# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from uf_ros_lib.uf_robot_utils import get_xacro_command

def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]

def build_camera_description(camera_namespace=""):
    # Get the path to the camera xacro file
    xacro_file_path = os.path.join(
        get_package_share_directory('realsense2_description'),
        'urdf',
        'test_d455_camera.urdf.xacro'
    )

    # Process the xacro file with any necessary mappings
    doc = xacro.process_file(xacro_file_path, mappings={'prefix': camera_namespace})
    robot_description_xml = doc.toxml()

    # Build the robot_description dictionary
    camera_robot_description = {'robot_description': robot_description_xml}

    return camera_robot_description


def prepare_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_name,
            default_value='false',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
            arm_id_name,
            default_value='fr3',
            description='Available values: fr3, fp3 and fer')

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand])

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )
    # Spawn world
    world_description_path = '/Panda_Arm/src/franka_ros2/world/world.sdf'

    gazebo_custom_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_description_path} -r'}.items(),  # Use the custom world path
    )

    # Spawn
    robot_idx = 0
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        # arguments=['-topic', '/robot_description'],
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-allow_renaming', 'false',
            '-x', str(0.0 + robot_idx * 0.4),
            '-y', '-0.3',
            '-z', '1.021',
            '-Y', '1.571',
            '-timeout', '10000',
        ],
        parameters=[{'use_sim_time': True}],
    )
    


    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )
    
    load_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'traj_controller'],
        output='screen'
    )

    camera_namespace = LaunchConfiguration('camera_namespace', default='camera_01')
    camera_robot_description = build_camera_description(camera_namespace=camera_namespace)
    
    # Node for creating bridge between gazebo and ros2 (ros2 run ros_gz_bridge parameter_bridge)
    parameters_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            [camera_namespace, '/camera_color@sensor_msgs/msg/Image@ignition.msgs.Image'],
            [camera_namespace, '/camera_depth@sensor_msgs/msg/Image@ignition.msgs.Image'],
            [camera_namespace, '/camera_depth/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
            [camera_namespace, '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
            [camera_namespace, '/camera_ired1@sensor_msgs/msg/Image@ignition.msgs.Image'],
            [camera_namespace, '/camera_ired2@sensor_msgs/msg/Image@ignition.msgs.Image'],
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/sensor_d455/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ]
    )

    # Node for launching camera robot state publisher
    robot_state_publisher_node_camera = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=camera_namespace,
        parameters=[camera_robot_description]
    )
    
    # Node for spawning the camera in gazebo environment
    gazebo_spawn_camera_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=camera_namespace,
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-allow_renaming', 'false',
            '-x', '0.5',
            '-y', '0.9',
            '-z', '1.15',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '-1.9',
            '-timeout', '10000',
        ],
        parameters=[{'use_sim_time': True}],
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[gazebo_spawn_camera_node]
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        # gazebo_empty_world,
        gazebo_custom_world,
        robot_state_publisher,
        rviz,
        spawn,
        parameters_bridge,
        robot_state_publisher_node_camera,
        delayed_spawn,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster,load_traj_controller],
                )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        ),
    ])

def generate_launch_description():
    launch_description = prepare_launch_description()
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('franka_description')))

    launch_description.add_action(set_env_vars_resources)
    os.environ['GZ_SIM_RESOURCE_PATH'] = '/Panda_Arm/src'

    return launch_description
