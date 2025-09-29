from setuptools import setup, find_packages
import os

package_name = 'p4_extra'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        # package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # yaml / urdf assets
        (os.path.join('share', package_name, 'cfg'),  ['cfg/cali.yaml']),
        (os.path.join('share', package_name, 'urdf'), ['urdf/robot.urdf']),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'numpy',
        'urdf-parser-py',
        'python-orocos-kdl',
        'tf-transformations',
        'ament-index-python',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@domain.com',
    description='Camera calibration publisher, rectifier and XArm KDL tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_base_camera_info = p4_extra.base_camera_cali:main',
            'rectify_video        =    p4_extra.rectify:main',
            'xarm_cart_speed        =  p4_extra.cart_speed:main',
        ],
    },
)

