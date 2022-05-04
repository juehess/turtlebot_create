#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: __init__.py 11217 2010-09-23 21:08:11Z kwc $

# TODO(allenh1): whaaaaat
# import roslib; roslib.load_manifest('create_node')

"""
ROS Turtlebot node for ROS built on top of create_driver's
turtlebot implementation. This driver is based on
otl_roomba by OTL (otl-ros-pkg).

create_driver's turtlebot implementation is based on
Damon Kohler's pyrobot.py.
"""

import os
import sys
import select
import serial
import termios
import time

from math import sin, cos

# TODO(allenh1): is this needed for something?
# import rospkg
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

from geometry_msgs.msg import Point, Pose, Pose2D, PoseWithCovariance, \
    Quaternion, Twist, TwistWithCovariance, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from create_driver import Turtlebot, MAX_WHEEL_SPEED, DriverError
from create_msgs.msg import TurtlebotSensorState, Drive, Turtle
from create_msgs.srv import SetTurtlebotMode, SetDigitalOutputs
from create_node.diagnostics import TurtlebotDiagnostics
import create_node.robot_types as robot_types
from create_node.covariances import \
     ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE2, ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE2
from create_node.songs import bonus

class TurtlebotNode(Node):
    _SENSOR_READ_RETRY_COUNT = 5

    def __init__(self, default_port='/dev/ttyUSB0', default_update_rate=60.0):
        super().__init__('turtlebot')
        """
        @param default_port: default tty port to use for establishing
            connection to Turtlebot.  This will be overriden by ~port ROS
            param if available.
        """
        self.default_port = default_port
        self.default_update_rate = default_update_rate

        self.robot = Turtlebot()
        self.sensor_handler = None
        self.sensor_state = TurtlebotSensorState()
        self.req_cmd_vel = None

        self._init_params()
        self._init_pubsub()

        self._pos2d = Pose2D() # 2D pose for odometry

        self._diagnostics = TurtlebotDiagnostics()
#        self.has_gyro = False
        if self.has_gyro:
            from create_node.gyro import TurtlebotGyro
            self._gyro = TurtlebotGyro()
        else:
            self._gyro = None
        self.create_timer(1.0 / self.update_rate, self.spin)
        # TODO(allenh1): implement parameters
        # dynamic_reconfigure.server.Server(TurtleBotConfig, self.reconfigure)

    def start(self):
        log_once = True
        while rclpy.ok():
            try:
                self.robot.start(self.port, robot_types.ROBOT_TYPES[self.robot_type].baudrate)
                break
            except serial.serialutil.SerialException as ex:
                msg = "Failed to open port %s. Error: %s Please make sure the Create cable is plugged into the computer. \n"%((self.port), ex)
                # self._diagnostics.node_status(msg,"error")
                self.get_logger().error(msg)
                time.sleep(3.0)

        self.sensor_handler = robot_types.ROBOT_TYPES[self.robot_type].sensor_handler(self.robot)
        self.robot.safe = True

        if self.declare_parameter('bonus', False).value:        # TODO(allenh1): Enable gyro when possible (port PyKDL)

            bonus(self.robot)

        self.robot.control()
        # Write driver state to disk
        with open(connected_file(), 'w') as f:
            f.write("1")

        # Startup readings from Create can be incorrect, discard first values
        s = TurtlebotSensorState()
        try:
            self.sense(s)
        except Exception:
            # packet read can get interrupted, restart loop to
            # check for exit conditions
            pass


    def _init_params(self):
        self.port = self.declare_parameter('port', self.default_port).value
        self.robot_type = self.declare_parameter('robot_type', 'create').value
        self.update_rate = self.declare_parameter('update_rate', self.default_update_rate).value
        self.drive_mode = self.declare_parameter('drive_mode', 'twist').value
        self.has_gyro = self.declare_parameter('has_gyro', True).value
        self.odom_angular_scale_correction = self.declare_parameter(
            'odom_angular_scale_correction', 1.0
        ).value
        self.odom_linear_scale_correction = self.declare_parameter(
            'odom_linear_scale_correction', 1.0
        ).value
        self.cmd_vel_timeout = Duration(
            seconds=self.declare_parameter(
                'cmd_vel_timeout', 0.6
            ).value
        )
        self.stop_motors_on_bump = self.declare_parameter('stop_motors_on_bump', True).value
        self.min_abs_yaw_vel = self.declare_parameter('min_abs_yaw_vel', rclpy.Parameter.Type.DOUBLE).value
        self.max_abs_yaw_vel = self.declare_parameter('max_abs_yaw_vel', rclpy.Parameter.Type.DOUBLE).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.operate_mode = self.declare_parameter('operation_mode', 3).value

        self.get_logger().info("serial port: '%s'" % (self.port))
        self.get_logger().info("update_rate: '%s'" % (self.update_rate))
        self.get_logger().info("drive mode: '%s'" % (self.drive_mode))
        self.get_logger().info("has gyro: '%s'" % (self.has_gyro))

    def _init_pubsub(self):
        # create publishers
        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sensor_state_pub = self.create_publisher(TurtlebotSensorState, 'sensor_state', 10)
        # create services
        self.operating_mode_srv = self.create_service(
            SetTurtlebotMode,
            'set_operation_mode',
            self.set_operation_mode
        )
        self.digital_output_srv = self.create_service(
            SetDigitalOutputs,
            'set_digital_outputs',
            self.set_digital_outputs
        )
        # create subscribers
        if self.drive_mode == 'twist':
            self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel, 10)
            self.drive_cmd = self.robot.direct_drive
        elif self.drive_mode == 'drive':
            self.cmd_vel_sub = self.create_subscription(Drive, 'cmd_vel', self.cmd_vel, 10)
            self.drive_cmd = self.robot.drive
        elif self.drive_mode == 'turtle':
            self.cmd_vel_sub = self.create_subscription(Turtle, 'cmd_vel', self.cmd_vel, 10)
            self.drive_cmd = self.robot.direct_drive
        else:
            self.get_logger().error("unknown drive mode :%s" % (self.drive_mode))

        self.transform_broadcaster = None
        if self.publish_tf:
            self.transform_broadcaster = tf2_ros.TransformBroadcaster(self)

    def reconfigure(self, config, level):
        self.update_rate = config['update_rate']
        self.drive_mode = config['drive_mode']
        self.has_gyro = config['has_gyro']
        if self.has_gyro:
            self._gyro.reconfigure(config, level)
        self.odom_angular_scale_correction = config['odom_angular_scale_correction']
        self.odom_linear_scale_correction = config['odom_linear_scale_correction']
        self.cmd_vel_timeout = rclpy.Duration(config['cmd_vel_timeout'])
        self.stop_motors_on_bump = config['stop_motors_on_bump']
        self.min_abs_yaw_vel = config['min_abs_yaw_vel']
        self.max_abs_yaw_vel = config['max_abs_yaw_vel']
        return config

    def cmd_vel(self, msg):
        # Clamp to min abs yaw velocity, to avoid trying to rotate at low
        # speeds, which doesn't work well.
        if self.min_abs_yaw_vel is not None and msg.angular.z != 0.0 and abs(msg.angular.z) < self.min_abs_yaw_vel:
            msg.angular.z = self.min_abs_yaw_vel if msg.angular.z > 0.0 else -self.min_abs_yaw_vel
        # Limit maximum yaw to avoid saturating the gyro
        if self.max_abs_yaw_vel is not None and self.max_abs_yaw_vel > 0.0 and msg.angular.z != 0.0 and abs(msg.angular.z) > self.max_abs_yaw_vel:
            msg.angular.z = self.max_abs_yaw_vel if msg.angular.z > 0.0 else -self.max_abs_yaw_vel
        if self.drive_mode == 'twist':
            # convert twist to direct_drive args
            ts  = msg.linear.x * 1000 # m -> mm
            tw  = msg.angular.z  * (robot_types.ROBOT_TYPES[self.robot_type].wheel_separation / 2) * 1000
            # Prevent saturation at max wheel speed when a compound command is sent.
            if ts > 0:
                ts = min(ts,   MAX_WHEEL_SPEED - abs(tw))
            else:
                ts = max(ts, -(MAX_WHEEL_SPEED - abs(tw)))
            self.req_cmd_vel = int(ts - tw), int(ts + tw)
        elif self.drive_mode == 'turtle':
            # convert to direct_drive args
            ts  = msg.linear * 1000 # m -> mm
            tw  = msg.angular  * (robot_types.ROBOT_TYPES[self.robot_type].wheel_separation / 2) * 1000
            self.req_cmd_vel = int(ts - tw), int(ts + tw)
        elif self.drive_mode == 'drive':
            # convert twist to drive args, m->mm (velocity, radius)
            self.req_cmd_vel = msg.velocity * 1000, msg.radius * 1000

    def set_operation_mode(self,req):
        if not self.robot.sci:
            self.get_logger().warn("Create : robot not connected yet, sci not available")
            return SetTurtlebotModeResponse(False)

        self.operate_mode = req.mode

        if req.mode == 1: #passive
            self._robot_run_passive()
        elif req.mode == 2: #safe
            self._robot_run_safe()
        elif req.mode == 3: #full
            self._robot_run_full()
        else:
            self.get_logger().error("Requested an invalid mode.")
            return SetTurtlebotModeResponse(False)
        return SetTurtlebotModeResponse(True)

    def _robot_run_passive(self):
        """
        Set robot into passive run mode
        """
        self.get_logger().info("Setting turtlebot to passive mode.")
        #setting all the digital outputs to 0
        self._set_digital_outputs([0, 0, 0])
        self.robot.passive()

    def _robot_reboot(self):
        """
        Perform a soft-reset of the Create
        """
        self.get_logger().dbg("Soft-rebooting turtlebot to passive mode.")
        self._diagnostics.node_status(msg,"warn")
        self._set_digital_outputs([0, 0, 0])
        self.robot.soft_reset()
        time.sleep(2.0)

    def _robot_run_safe(self):
        """
        Set robot into safe run mode
        """
        self.get_logger().info("Setting turtlebot to safe mode.")
        self.robot.safe = True
        self.robot.control()
        b1 = (self.sensor_state.user_digital_inputs & 2)/2
        b2 = (self.sensor_state.user_digital_inputs & 4)/4
        self._set_digital_outputs([1, b1, b2])

    def _robot_run_full(self):
        """
        Set robot into full run mode
        """
        self.get_logger().info("Setting turtlebot to full mode.")
        self.robot.safe = False
        self.robot.control()
        b1 = (self.sensor_state.user_digital_inputs & 2)/2
        b2 = (self.sensor_state.user_digital_inputs & 4)/4
        self._set_digital_outputs([1, b1, b2])

    def _set_digital_outputs(self, outputs):
        assert len(outputs) == 3, 'Expecting 3 output states.'
        byte = 0
        for output, state in enumerate(outputs):
            byte += (2 ** output) * int(state)
        self.robot.set_digital_outputs(byte)
        self.sensor_state.user_digital_outputs = byte

    def set_digital_outputs(self,req):
        if not self.robot.sci:
            raise Exception("Robot not connected, SCI not available")

        outputs = [req.digital_out_0,req.digital_out_1, req.digital_out_2]
        self._set_digital_outputs(outputs)
        return SetDigitalOutputsResponse(True)

    def sense(self, sensor_state):
        self.sensor_handler.get_all(sensor_state)
        if self._gyro:
            self._gyro.update_calibration(sensor_state)

    def spin(self):
        # state
        s = self.sensor_state
        odom = Odometry()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        js = JointState(
            name = [
                "left_wheel_joint",
                "right_wheel_joint",
                "front_castor_joint",
                "back_castor_joint"
            ],
            position=[0,0,0,0],
            velocity=[0,0,0,0],
            effort=[0,0,0,0]
        )

        # r = rclpy.Rate(self.update_rate)
        last_cmd_vel = 0, 0
        # TODO(allenh1): we need to grab this from ...somewhere...
        last_cmd_vel_time = self.get_clock().now()
        last_js_time = self.get_clock().now()
        # We set the retry count to 0 initially to make sure that only
        # if we received at least one sensor package, we are robust
        # agains a few sensor read failures. For some strange reason,
        # sensor read failures can occur when switching to full mode
        # on the Roomba.
        sensor_read_retry_count = 0

        while rclpy.ok():
            last_time = Time.from_msg(s.header.stamp)
            curr_time = self.get_clock().now()

            # SENSE/COMPUTE STATE
            try:
                self.sense(s)
                transform = self.compute_odom(s, last_time, odom)
                # Future-date the joint states so that we don't have
                # to publish as frequently.
                js.header.stamp = Time.to_msg(curr_time + Duration(seconds=1)) 
            except select.error:
                # packet read can get interrupted, restart loop to
                # check for exit conditions
                continue
            except DriverError:
                if sensor_read_retry_count > 0:
                    self.get_logger().warn(
                        'Failed to read sensor package. %d retries left.' %
                        sensor_read_retry_count
                    )
                    sensor_read_retry_count -= 1
                    continue
                else:
                    raise
            sensor_read_retry_count = self._SENSOR_READ_RETRY_COUNT

            # Reboot Create if we detect that charging is necessary.
            if s.charging_sources_available > 0 and \
                   s.oi_mode == 1 and \
                   s.charging_state in [0, 5] and \
                   s.charge < 0.93*s.capacity:
                self.get_logger().info("going into soft-reboot and exiting driver")
                self._robot_reboot()
                self.get_logger().info("exiting driver")
                break

            # Reboot Create if we detect that battery is at critical level switch to passive mode.
            if s.charging_sources_available > 0 and \
                   s.oi_mode == 3 and \
                   s.charging_state in [0, 5] and \
                   s.charge < 0.15*s.capacity:
                self.get_logger().info("going into soft-reboot and exiting driver")
                self._robot_reboot()
                self.get_logger().info("exiting driver")
                break

            # PUBLISH STATE
            self.sensor_state_pub.publish(s)
            self.odom_pub.publish(odom)
            if self.publish_tf:
                self.publish_odometry_transform(odom)
            # 1hz, future-dated joint state
            if curr_time > last_js_time + Duration(seconds=1):
                self.joint_states_pub.publish(js)
                last_js_time = curr_time
            self._diagnostics.publish(s, self._gyro)
            if self._gyro:
                self._gyro.publish(s, last_time)

            # ACT
            if self.req_cmd_vel:
                # check for velocity command and set the robot into full mode if not plugged in
                # if s.oi_mode != self.operate_mode and s.charging_sources_available != 1:
                #    if self.operate_mode == 2:
                #        self._robot_run_safe()
                #    else:
                #        self._robot_run_full()

                # check for bumper contact and limit drive command
                req_cmd_vel = self.check_bumpers(s, self.req_cmd_vel)

                # Set to None so we know it's a new command
                # self.req_cmd_vel = None
                # reset time for timeout
                last_cmd_vel_time = last_time
            else:
                #zero commands on timeout
                if last_time - last_cmd_vel_time > self.cmd_vel_timeout:
                    last_cmd_vel = 0,0
                # double check bumpers
                req_cmd_vel = self.check_bumpers(s, last_cmd_vel)

            # send command
            self.drive_cmd(*req_cmd_vel)
            # record command
            last_cmd_vel = req_cmd_vel
            break

    def check_bumpers(self, s, cmd_vel):
        # Safety: disallow forward motion if bumpers or wheeldrops
        # are activated.
        # TODO: check bumps_wheeldrops flags more thoroughly, and disable
        # all motion (not just forward motion) when wheeldrops are activated
        forward = (cmd_vel[0] + cmd_vel[1]) > 0
        if self.stop_motors_on_bump and s.bumps_wheeldrops > 0 and forward:
            return (0,0)
        else:
            return cmd_vel

    def compute_odom(self, sensor_state, last_time, odom):
        """
        Compute current odometry.  Updates odom instance and returns tf
        transform. compute_odom() does not set frame ids or covariances in
        Odometry instance.  It will only set stamp, pose, and twist.

        @param sensor_state: Current sensor reading
        @type  sensor_state: TurtlebotSensorState
        @param last_time: time of last sensor reading
        @type  last_time: Time
        @param odom: Odometry instance to update.
        @type  odom: nav_msgs.msg.Odometry

        @return: transform
        @rtype: ( (float, float, float), (float, float, float, float) )
        """
        # based on otl_roomba by OTL <t.ogura@gmail.com>

        current_time = Time.from_msg(sensor_state.header.stamp)
        dt = current_time - last_time

        # On startup, Create can report junk readings
        if abs(sensor_state.distance) > 1.0 or abs(sensor_state.angle) > 1.0:
            raise Exception("Distance, angle displacement too big, invalid readings from robot. Distance: %.2f, Angle: %.2f" % (sensor_state.distance, sensor_state.angle))

        # this is really delta_distance, delta_angle
        d  = sensor_state.distance * self.odom_linear_scale_correction #correction factor from calibration
        angle = sensor_state.angle * self.odom_angular_scale_correction #correction factor from calibration

        x = cos(angle) * d
        y = -sin(angle) * d

        last_angle = self._pos2d.theta
        self._pos2d.x += cos(last_angle)*x - sin(last_angle)*y
        self._pos2d.y += sin(last_angle)*x + cos(last_angle)*y
        self._pos2d.theta += angle

        # Turtlebot quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))

        # construct the transform
        transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

        # update the odometry state
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = self._pos2d.x
        odom.pose.pose.position.y = self._pos2d.y
        odom.pose.pose.orientation = Quaternion(
            x=0., y=0., z=sin(self._pos2d.theta / 2.), w=cos(self._pos2d.theta / 2.)
        )
        if dt.nanoseconds > 0:
            odom.twist.twist.linear = Vector3(x=(d / (dt.nanoseconds * 1e-9)), y=0., z=0.)
            odom.twist.twist.angular = Vector3(x=0., y=0., z=(angle / (dt.nanoseconds * 1e-9)))
            if sensor_state.requested_right_velocity == 0 and \
               sensor_state.requested_left_velocity == 0 and \
               sensor_state.distance == 0:
                odom.pose.covariance = ODOM_POSE_COVARIANCE2
                odom.twist.covariance = ODOM_TWIST_COVARIANCE2
            else:
                odom.pose.covariance = ODOM_POSE_COVARIANCE
                odom.twist.covariance = ODOM_TWIST_COVARIANCE

        # return the transform
        return transform

    def publish_odometry_transform(self, odometry):
        transform = TransformStamped()
        transform.header.stamp = odometry.header.stamp
        transform.child_frame_id = odometry.child_frame_id
        transform.header.frame_id = odometry.header.frame_id
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation.x = odometry.pose.pose.orientation.x
        transform.transform.rotation.y = odometry.pose.pose.orientation.y
        transform.transform.rotation.z = odometry.pose.pose.orientation.z
        transform.transform.rotation.w = odometry.pose.pose.orientation.w
        self.transform_broadcaster.sendTransform(transform)


def connected_file():
    # TODO(allenh1): avoid hard-coding '/tmp' here
    return os.path.join('/tmp', 'turtlebot-connected')

def turtlebot_main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = TurtlebotNode()
    node.start()
    node._robot_run_full()
    time.sleep(3.0)
    # while rclpy.ok():
    rclpy.spin(node)
        # try:
            # This sleep throttles reconnecting of the driver.  It
            # appears that pyserial does not properly release the file
            # descriptor for the USB port in the event that the Create is
            # unplugged from the laptop.  This file desecriptor prevents
            # the create from reassociating with the same USB port when it
            # is plugged back in.  The solution, for now, is to quickly
            # exit the driver and let roslaunch respawn the driver until
            # reconnection occurs.  However, it order to not do bad things
            # to the Create bootloader, and also to keep relaunching at a
            # minimum, we have a 3-second sleep.
            # time.sleep(3.0)

            # node.start()
            
            # rclpy.spin(node)
            # node.spin()
            # node.destroy_node()
            # rclpy.shutdown()

#        except Exception as ex:
#            msg = "Failed to contact device with error: [%s]. Please check that the Create is powered on and that the connector is plugged into the Create."%(ex)
            # node._diagnostics.node_status(msg,"error")
#            node.get_logger().error(msg)0
#
#        finally:
#            # Driver no longer connected, delete flag from disk
#            try:
#                os.remove(connected_file())
#            except Exception:
#                pass


if __name__ == '__main__':
    turtlebot_main(sys.argv)
