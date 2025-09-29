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
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=False)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')

    moveit_config_dump = LaunchConfiguration('moveit_config_dump', default='')
    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}

    moveit_config = MoveItConfigsBuilder(
        context=context,
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,

        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    ).moveit_cpp(file_path=get_package_share_directory("xarm_moveit_config") + "/config/moveit_cpp.yaml").planning_pipelines("ompl").to_moveit_configs()
    moveit_config_dict = moveit_config.to_dict()

    xarm_type = '{}{}'.format(robot_type.perform(context),
                              dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')

    servo_yaml = load_yaml('xarm_moveit_servo', "config/xarm_moveit_servo_config.yaml")
    servo_yaml['move_group_name'] = xarm_type
    xarm_traj_controller = '{}{}_traj_controller'.format(prefix.perform(context), xarm_type)
    servo_yaml['command_out_topic'] = '/{}/joint_trajectory'.format(xarm_traj_controller)
    servo_params = {"moveit_servo": servo_yaml}

    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "xarm7"}

    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="false"
    )

    #
    # Servo node
    #

    container = ComposableNodeContainer(
        name="xarm_moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "link_base", "frame_id": "world"}],
            ),
        ],
        output="screen",
    )

    #
    # Servo node stand alone
    #

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    #
    # Servo vision node
    #

    servo_vision = Node(
        package="p4_servo_vision",
        executable="servo_vision_node",
        name="servo_vision_node",
        output="screen",
        parameters=[moveit_config_dict],
    )


    #
    # Servo vision node
    #

    docking = Node(
        package="p4_docking",
        executable="docking_node",
        name="docking_node",
        output="screen",
        parameters=[moveit_config_dict],
    )

    #
    # Servo vision node
    #

    state_machine = Node(
        package="p4_state_machine",
        executable="state_machine_node",
        name="state_machine_node",
        output="screen",
        parameters=[moveit_config_dict],
    )

    #
    # Crawling motion node
    #

    crawling_motion = Node(
        package="p4_crawling_motion",
        executable="crawling_motion_node",
        name="crawling_motion_node",
        output="screen",
    )

    #
    # Frame end effector - camera.
    #

    frame_ee_camera = Node(
        package="p4_robot_controller",
        executable="rc_frame_ee_camera",
        name="frame_ee_camera",
        output="screen",
    )

    #
    # Frame end effector - camera.
    #

    target_locator = Node(
        package="p4_target_locator",
        executable="target_locator_node",
        name="target_locator_node",
        output="screen",
    )

    rl_p2p = Node(
        package="rob10_robot_policy",
        executable="p2p_node",
        name="p2p_node",
        output="screen",
    )

    rl_move = Node(
        package="rob10_robot_policy",
        executable="move_node",
        name="move_node",
        output="screen",
    )

    nodes = [
        frame_ee_camera,
        # target_locator,
        servo_node,
        container,
        servo_vision,
        # state_machine,
        # crawling_motion,
        # docking,
        rl_p2p,
        rl_move,
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
