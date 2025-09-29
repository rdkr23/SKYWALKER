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
from launch.conditions import IfCondition, UnlessCondition




def launch_setup(context, *args, **kwargs):
    #
    # Realsense D435i
    #

    realsense_camera_eef = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera_eef',
        output='screen',
        parameters=[
            {'enable_gyro': False},
            {'enable_accel': False},
            {'serial_no': '310622071850'}, # 336222071386
            {'camera_name': 'camera_eef'},
        ]
    )

    #
    # April tag reader
    #

    april_tag_reader_eef = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_eef',
        output='screen',
        remappings=[
            ('image_rect', '/camera/camera_eef/color/image_raw'),
            ('camera_info', '/camera/camera_eef/color/camera_info')
        ],
        parameters=[get_package_share_directory("apriltag_ros") + "/cfg/tags_36h11.yaml", {'camera_id': 'eef-'}]
        # {
        #             'param_file': get_package_share_directory("apriltag_ros") + "/cfg/tags_36h11.yaml"# '`ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml'
        #         }
    )

    nodes = [
        realsense_camera_eef,
        april_tag_reader_eef,
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
