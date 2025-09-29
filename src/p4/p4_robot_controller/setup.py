from setuptools import find_packages, setup

package_name = 'p4_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ament_python', 'rclpy', 'std_msgs', 'custom_msg', 'moveit_py', 'kdl_parser_py', 'urdf_parser_py', 'numpy', 'PyKDL'],
    zip_safe=True,
    maintainer='rdk',
    maintainer_email='rdkr23@student.aau.dk',
    description='Controller of robot - specifically the Xarm 7 from Ufactory.',
    license='Apache License 2.0',  # Replace with your actual license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_subscriber = p4_robot_controller.robot_controller_subscriber:main',
            'rc_publisher = p4_robot_controller.robot_controller_publisher:main',
            'rc_test = p4_robot_controller.robot_controller_test_comm:main',
            'rc_ee_clc = p4_robot_controller.ee_closed_loop_conn:main',
            'rc_ee_test = p4_robot_controller.ee_moveit_test:main',
            'rc_robot_pos = p4_robot_controller.servo_robot_controller:main',
            'rc_main_node = p4_robot_controller.robot_controller_node:main',
            'rc_frame_ee_camera = p4_robot_controller.frame_ee_camera:main',
            'rc_moniter = p4_robot_controller.moniter_node:main',
            'rc_servo_test = p4_robot_controller.imp_pup:main',
            'rc_ik = p4_robot_controller.ik_node:main',
            'rc_sss = p4_robot_controller.servo:main',
            'rc_try = p4_robot_controller.try:main',
        ],
    },
)