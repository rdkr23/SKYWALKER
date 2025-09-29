from setuptools import find_packages, setup

package_name = 'p4_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rdk',
    maintainer_email='rdkr23@student.aau.dk',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine_node = p4_state_machine.state_machine_node:main',
            'state_machine_node_small = p4_state_machine.small_state_machine_node:main'
        ],
    },
)
