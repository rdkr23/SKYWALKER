import yaml

from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
import json

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder as MoveItConfigsBuilder2
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from uf_ros_lib.uf_robot_utils import load_yaml, generate_ros2_control_params_temp_file
from launch.conditions import IfCondition, UnlessCondition




def launch_setup(context, *args, **kwargs):
    #
    # Realsense D435i
    #

    realsense_camera_base = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera_base',
        output='screen',
        parameters=[
            {'enable_gyro': False},
            {'enable_accel': False},
            {'serial_no': '336222071386'}, # 336222071386
            {'camera_name': 'camera_base'},
        ]
    )

    #
    # April tag reader
    #

    april_tag_reader_base = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_base',
        output='screen',
        remappings=[
            ('image_rect', '/camera/camera_base/color/image_raw'),
            ('camera_info', '/camera/camera_base/color/camera_info')
        ],
        parameters=[get_package_share_directory("apriltag_ros") + "/cfg/tags_36h11.yaml", {'camera_id': 'base-'}]
        # {
        #             'param_file': get_package_share_directory("apriltag_ros") + "/cfg/tags_36h11.yaml"# '`ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml'
        #         }
    )

    #
    # End effector node
    #

    end_effector = Node(
        package="p4_end_effector",
        executable="end_effector_node",
        name="end_effector_node",
        output="screen",
    )

    brake = Node(
        package="p4_brake",
        executable="brake_node",
        name="brake_node",
        output="screen",
    )

    nodes = [
        realsense_camera_base,
        april_tag_reader_base,
        end_effector,
        brake,
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
