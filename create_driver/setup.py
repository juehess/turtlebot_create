from setuptools import find_packages
from setuptools import setup

package_name = 'create_driver'

setup(
    name=package_name,
    version='2.3.1',
    packages=['create_driver'],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daemon Kohler',
    author_email='damonkohler@google.com',
    keywords=['ROS2', 'iRobot Create', 'turtlebot'],
    description=(
"""
    Driver for iRobot Create and Roomba

    This is a generic driver for iRobot Create that currently holds
    implementations for Turtlebot and Roomba. Port
    of pyrobot.py by Damon Kohler.  It is currently labeled as
    turtlebot_driver pending review by the entire create community
    before using the name create_driver.

    For ROS bindings, please see turtlebot_node.
"""
    ),
    license='MIT',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
    ],
)
