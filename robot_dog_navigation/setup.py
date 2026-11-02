from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_dog_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='willie',
    maintainer_email='willie040600@gmail.com',
    description='Navigation side of robot_dog in Isaac Sim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_footprint_node = robot_dog_navigation.odom_footprint_node:main',
            'scan_ground_filter_node = robot_dog_navigation.scan_ground_filter_node:main',
            'nav_eval = robot_dog_navigation.nav_eval:main',
            'check_map = robot_dog_navigation.check_map:main',
        ],
    },
)
