from setuptools import find_packages, setup

package_name = 'rob10_robot_policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    include_package_data=True,
    package_data={
        "rob10_robot_policy": [
            "agents/ptp/*.pt",
            "agents/ptp/*.yaml",
            "agents/ptp_gen4/*.pt",
            "agents/ptp_gen4/*.yaml",
            "agents/move_gen4/*.pt",
            "agents/move_gen4/*.yaml",
            "agents/move_gen5/*.pt",
            "agents/move_gen5/*.yaml",
            "agents/move/*.pt",
            "agents/move/*.yaml",
        ],
    },
    install_requires=['setuptools', 'ament_python', 'rclpy', 'std_msgs', 'custom_msg',
                      'moveit_py', 'kdl_parser_py', 'urdf_parser_py', 'numpy', 'PyKDL', 'torch'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='akawar23@student.aau.com',
    description='TODO :This packages wraps the policy from the RL agent to compute robot actions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'p2p_test = rob10_robot_policy.p2p_test:main',
            'p2p_node = rob10_robot_policy.run_task_reach:main',
            'move_node = rob10_robot_policy.run_task_move_2:main',
            'robot_mover = rob10_robot_policy.robot_mover:main',
        ],
    },
)
