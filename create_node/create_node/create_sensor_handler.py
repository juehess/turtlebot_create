# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

# from construct import Struct
import struct
import logging
import time
import rclpy

from rclpy.time import Time
from rclpy.node import Node

from math import radians
from create_driver.create_driver import SENSOR_GROUP_PACKET_LENGTHS

#_struct_I = roslib.message.struct_I
_struct_BI = struct.Struct(">BI")
_struct_12B2hBHhb7HBH5B4h = struct.Struct(">12B2hBHhb7HBH5B4h")
# TODO(allenh1): ^^ wtf?


# NOTE(allenh1): this is a hack. In ROS 2, we do not have
# pure python messages, so we construct a pure-python container
# to make the old code work.
class TurtlebotSensorStateMsg():
    def __init__(self):
        self.bumps_wheeldrops = None
        self.wall = None
        self.cliff_left = None
        self.cliff_front_left = None
        self.cliff_front_right = None
        self.cliff_right = None
        self.virtual_wall = None
        self.motor_overcurrents = None
        self.dirt_detector_left = None
        self.dirt_detector_right = None
        self.remote_opcode = None
        self.buttons = None
        self.distance = None
        self.angle = None
        self.charging_state = None
        self.voltage = None
        self.current = None
        self.temperature = None
        self.charge = None
        self.capacity = None
        self.wall_signal = None
        self.cliff_left_signal = None
        self.cliff_front_left_signal = None
        self.cliff_front_right_signal = None
        self.cliff_right_signal = None
        self.user_digital_inputs = None
        self.user_analog_input = None
        self.charging_sources_available = None
        self.oi_mode = None
        self.song_number = None
        self.song_playing = None
        self.number_of_stream_packets = None
        self.requested_velocity = None
        self.requested_radius = None
        self.requested_right_velocity = None
        self.requested_left_velocity = None


def deserialize(msg, buff, timestamp):
    """
    unpack serialized message in str into this message instance
    @param buff: byte array of serialized message
    @type  buff: str
    """
    try:
        _x = TurtlebotSensorStateMsg()
        (_x.bumps_wheeldrops, _x.wall, _x.cliff_left, _x.cliff_front_left, _x.cliff_front_right, _x.cliff_right, _x.virtual_wall, _x.motor_overcurrents, _x.dirt_detector_left, _x.dirt_detector_right, _x.remote_opcode, _x.buttons, _x.distance, _x.angle, _x.charging_state, _x.voltage, _x.current, _x.temperature, _x.charge, _x.capacity, _x.wall_signal, _x.cliff_left_signal, _x.cliff_front_left_signal, _x.cliff_front_right_signal, _x.cliff_right_signal, _x.user_digital_inputs, _x.user_analog_input, _x.charging_sources_available, _x.oi_mode, _x.song_number, _x.song_playing, _x.number_of_stream_packets, _x.requested_velocity, _x.requested_radius, _x.requested_right_velocity, _x.requested_left_velocity,) = _struct_12B2hBHhb7HBH5B4h.unpack(buff[0:52])

        msg.wall = bool(_x.wall)
        msg.cliff_left = bool(_x.cliff_left)
        msg.cliff_front_left = bool(_x.cliff_front_left)
        msg.cliff_front_right = bool(_x.cliff_front_right)
        msg.cliff_right = bool(_x.cliff_right)
        msg.virtual_wall = bool(_x.virtual_wall)
        msg.song_playing = bool(_x.song_playing)

        # do unit conversions
        msg.angle = radians(_x.angle)
        # msg.header.stamp = rclpy
        msg.distance = float(_x.distance) / 1000.
    
        msg.requested_velocity = float(_x.requested_velocity) / 1000.
        msg.requested_radius = float(_x.requested_radius) / 1000.
        msg.requested_right_velocity = float(_x.requested_right_velocity) / 1000.
        msg.requested_left_velocity = float(_x.requested_left_velocity) / 1000.

        return msg
    except struct.error as e:
        # TODO(allenh1): evaluate what this looks like in ROS 2
        # raise roslib.message.DeserializationError(e)
        raise Exception(e)


class CreateSensorHandler(object):    
    def __init__(self, robot):
        self.robot = robot
        self.node = Node('SensorHandlerNode')

    def request_packet(self, packet_id):
        """Reqeust a sensor packet."""
        with self.robot.sci.lock:
            self.robot.sci.flush_input()
            self.robot.sci.sensors(packet_id)
            #kwc: there appears to be a 10-20ms latency between sending the
            #sensor request and fully reading the packet.  Based on
            #observation, we are preferring the 'before' stamp rather than
            #after.
            stamp = self.node.get_clock().now()
            length = SENSOR_GROUP_PACKET_LENGTHS[packet_id]
            return self.robot.sci.read(length), stamp

    def get_all(self, sensor_state):
        buff, timestamp = self.request_packet(6)
        # self.node.get_logger().info("buffer lenght: '%d'" % len(buff))
        # self.node.get_logger().info("buffer: '%s'" % buff)
        if buff:
            deserialize(sensor_state, buff, timestamp)
