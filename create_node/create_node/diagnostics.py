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
#    contributors may bediag used to endorse or promote products derived
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

#Melonee Wise mwise@willowgarage.com

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class TurtlebotDiagnostics():
    def __init__(self, node):
        self.node = node
        self.charging_state =  {0:"Not Charging",
                                1:"Reconditioning Charging",
                                2:"Full Charging",
                                3:"Trickle Charging",
                                4:"Waiting",
                                5:"Charging Fault Condition"}
        self.charging_source = {0:"None",
                                1:"Internal Charger",
                                2:"Base Dock"}
        self.digital_outputs = {0:"OFF",
                                1:"ON"}
        self.oi_mode = {1:"Passive",
                        2:"Safe",
                        3:"Full"}
        self.diag_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.last_diagnostics_time = self.node.get_clock().now()


    def node_status(self, msg, status):
        curr_time = self.node.get_clock().now()
        diag = DiagnosticArray()
        diag.header.stamp = Time.to_msg(curr_time)
        stat = DiagnosticStatus()
        if status == "error":
            stat = DiagnosticStatus(name="TurtleBot Node", level=DiagnosticStatus.ERROR, message=msg)
        if status == "warn":
            stat = DiagnosticStatus(name="TurtleBot Node", level=DiagnosticStatus.WARN, message=msg)
        diag.status.append(stat)
        self.diag_pub.publish(diag)


    def publish(self, sensor_state, gyro):
        curr_time = Time.from_msg(sensor_state.header.stamp)
        # limit to 5hz
        diff_time = (curr_time - self.last_diagnostics_time).nanoseconds / 1e9
        if diff_time < 0.2:
            return
        self.last_diagnostics_time = curr_time

        diag = DiagnosticArray()
        diag.header.stamp = Time.to_msg(curr_time)
        stat = DiagnosticStatus()

        #node status
        stat = DiagnosticStatus(name="TurtleBot Node", level=DiagnosticStatus.OK, message="RUNNING")
        diag.status.append(stat)

        #mode info
        stat = DiagnosticStatus(name="Operating Mode", level=DiagnosticStatus.OK)
        try:
            stat.message = self.oi_mode[sensor_state.oi_mode]
        except KeyError as ex:
            stat.level=DiagnosticStatus.ERROR
            stat.message = "Invalid OI Mode Reported %s"%ex
            self.node.get_logger().warn(stat.message)
        diag.status.append(stat)
        
        #battery info
        stat = DiagnosticStatus(name="Battery", level=DiagnosticStatus.OK, message="OK")
        values = stat.values
        if sensor_state.charging_state == 5:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Charging Fault Condition"
            values.append(KeyValue(key="Charging State", value=self.charging_state[sensor_state.charging_state]))

        values.extend([KeyValue(key="Voltage (V)", value=str(sensor_state.voltage / 1000.0)),
                       KeyValue(key="Current (A)", value=str(sensor_state.current/1000.0)),
                       KeyValue(key="Temperature (C)",value=str(sensor_state.temperature)),
                       KeyValue(key="Charge (Ah)", value=str(sensor_state.charge/1000.0)),
                       KeyValue(key="Capacity (Ah)", value=str(sensor_state.capacity/1000.0))])
        diag.status.append(stat)
        
        #charging source
        stat = DiagnosticStatus(name="Charging Sources", level=DiagnosticStatus.OK)
        try:
            stat.message = self.charging_source[sensor_state.charging_sources_available]
        except KeyError as ex:
            stat.level=DiagnosticStatus.ERROR
            stat.message = "Invalid Charging Source %s, actual value: %i"%(ex,sensor_state.charging_sources_available)
            self.node.get_logger().warn(stat.message)
        diag.status.append(stat)
        #cliff sensors
        stat = DiagnosticStatus(name="Cliff Sensor", level=DiagnosticStatus.OK, message="OK")
        if sensor_state.cliff_left or sensor_state.cliff_front_left or sensor_state.cliff_right or sensor_state.cliff_front_right:
            stat.level = DiagnosticStatus.WARN
            if (sensor_state.current/1000.0)>0:
                stat.message = "Near Cliff: While the irobot create is charging, the create thinks it's near a cliff."
                stat.level = DiagnosticStatus.OK # We're OK here
            else:
                stat.message = "Near Cliff"
        stat.values = [KeyValue(key="Left", value=str(sensor_state.cliff_left)),
                       KeyValue(key="Left Signal", value=str(sensor_state.cliff_left_signal)),
                       KeyValue(key="Front Left", value=str(sensor_state.cliff_front_left)),
                       KeyValue(key="Front Left Signal", value=str(sensor_state.cliff_front_left_signal)),
                       KeyValue(key="Front Right", value=str(sensor_state.cliff_right)),
                       KeyValue(key="Front Right Signal", value=str(sensor_state.cliff_right_signal)),
                       KeyValue(key="Right", value=str(sensor_state.cliff_front_right)),
                       KeyValue(key="Right Signal", value=str(sensor_state.cliff_front_right_signal))]
        diag.status.append(stat)
        #Wall sensors
        stat = DiagnosticStatus(name="Wall Sensor", level=DiagnosticStatus.OK, message="OK")
        #wall always seems to be false??? 
        if sensor_state.wall:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Near Wall"
        stat.values = [KeyValue(key="Wall", value=str(sensor_state.wall)),
                       KeyValue(key="Wall Signal", value=str(sensor_state.wall_signal)),
                       KeyValue(key="Virtual Wall", value=str(sensor_state.virtual_wall))]
        diag.status.append(stat)
        #Gyro
        stat = DiagnosticStatus(name="Gyro Sensor", level = DiagnosticStatus.OK, message = "OK")
        if gyro is None:
            stat.level = DiagnosticStatus.WARN
            stat.message = "Gyro Not Enabled: To enable the gyro set the has_gyro param in the turtlebot_node."   
        elif gyro.cal_offset < 100:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Gyro Power Problem: For more information visit http://answers.ros.org/question/2091/turtlebot-bad-gyro-calibration-also."
        elif gyro.cal_offset > 575.0 or gyro.cal_offset < 425.0:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Bad Gyro Calibration Offset: The gyro average is outside the standard deviation."

        if gyro is not None:    
            stat.values = [KeyValue(key="Gyro Enabled", value=str(gyro is not None)),
                           KeyValue(key="Raw Gyro Rate", value=str(sensor_state.user_analog_input)),
                           KeyValue(key="Calibration Offset", value=str(gyro.cal_offset)),
                           KeyValue(key="Calibration Buffer", value=str(gyro.cal_buffer))]
        diag.status.append(stat)
        #Digital IO
        stat = DiagnosticStatus(name="Digital Outputs", level = DiagnosticStatus.OK, message = "OK")
        out_byte = sensor_state.user_digital_outputs
        stat.values = [KeyValue(key="Raw Byte", value=str(out_byte)),
                       KeyValue(key="Digital Out 2", value=self.digital_outputs[out_byte%2]),
                       KeyValue(key="Digital Out 1", value=self.digital_outputs[(out_byte >>1)%2]),
                       KeyValue(key="Digital Out 0", value=self.digital_outputs[(out_byte >>2)%2])]
        diag.status.append(stat)
        #publish
        self.diag_pub.publish(diag)

