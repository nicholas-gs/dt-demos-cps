# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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

# Based off https://github.com/ros2/teleop_twist_keyboard/blob/dashing/teleop_twist_keyboard.py
# Control the lane following behaviour of the Duckiebot using a keyboard

#!/usr/bin/env python3

import sys
import tty
import rclpy
import termios

from dt_interfaces_cps.msg import BoolStamped


welcome_msg = """
Start and stop lane following on the Duckiebot.
---------------------------
a : Start lane following
s : Stop lane following
CTRL-C to quit
"""


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    return termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    lane_following_state_msg = BoolStamped(data=False)
    node = rclpy.create_node('lane_following_control_node')
    node.declare_parameter(name="veh")
    veh = node.get_parameter(name="veh").get_parameter_value().string_value
    node.get_logger().info(f"Veh: {veh}")

    pub_start_lane_following = node.create_publisher(
        BoolStamped,
        f"/{veh}/gatekeeper_node/activate",
        1)

    try:
        print(welcome_msg)
        while True:
            key = getKey(settings)
            if key == 'a':
                lane_following_state_msg.data = True
                node.get_logger().info(f"Starting lane following!")
            elif key == 's':
                lane_following_state_msg.data = False
                node.get_logger().info(f"Stopping lane following!")
            elif key == '\x03':
                break
            lane_following_state_msg.header.stamp = node\
                .get_clock().now().to_msg()
            pub_start_lane_following.publish(lane_following_state_msg)
    except Exception as e:
        node.get_logger().fatal(e)
    finally:
        # Stop the lane following
        node.get_logger().info("Killing node! Stopping lane following!")
        lane_following_state_msg.header.stamp = node.get_clock().now().to_msg()
        lane_following_state_msg.data = False
        pub_start_lane_following.publish(lane_following_state_msg)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
