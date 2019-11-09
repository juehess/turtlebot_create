from setuptools import find_packages
from setuptools import setup

package_name = 'create_node'

setup(
    name=package_name,
    version='2.3.1',
    packages=['create_node'],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    author='Ken Conley',
    author_email='kwc@kwc.org',
    keywords=['ROS2', 'iRobot Create', 'turtlebot'],
    description=(
"""
    iRobot Create ROS driver node

    ROS bindings for the Create/Roomba driver.

    This is based on otl_roomba driver by OTL, ported to use
    create_driver's implementation instead.
    This also contains a 'bonus' feature from the turtlebot
    driver by Xuwen Cao and Morgan Quigley.
"""
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'turtlebot_node = create_node.turtlebot_node:turtlebot_main',
        ],
    },
)
