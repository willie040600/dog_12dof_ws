from setuptools import find_packages, setup

package_name = 'robot_dog_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='willie',
    maintainer_email='willie040600@gmail.com',
    description='Natural-language commands for robot_dog: a Gemini agent that sends goals to Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'chat = robot_dog_agent.chat:main',
        ],
    },
)
