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
# Control the movement of the Duckiebot using a keyboard

#!/usr/bin/env python3

import sys
import tty
import rclpy
import termios

from dt_interfaces_cps.msg import WheelsCmdStamped, BoolStamped


msg = """
This node takes key presses from the keyboard and publishes them as
WheelsCmdStamped and BoolStamped messages. It works best with a US keyboard 
layout.
---------------------------
Moving around:
        w
   a    s    d
   z    x
e : increase velocity
q : decrease velocity
x : stop
z : (activate/deactivate) emergency stop
CTRL-C to quit
"""


DEFAULT_SPEED: float = 0.2
MIN_SPEED: float = 0.0
MAX_SPEED: float = 1.0
SPEED_INCREMENT = 0.1


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


def change_speed(key, current_speed: float) -> float:
    if key == 'q':
        new_speed = current_speed - SPEED_INCREMENT
        return new_speed if new_speed >= MIN_SPEED else MIN_SPEED
    elif key == 'e':
        new_speed = current_speed + SPEED_INCREMENT
        return new_speed if new_speed <= MAX_SPEED else MAX_SPEED
    else:
        return 0.0


def main():
    settings = saveTerminalSettings()

    speed = DEFAULT_SPEED

    rclpy.init()

    wheels_msg = WheelsCmdStamped()
    e_stop_msg = BoolStamped(data=False)
    node = rclpy.create_node('wheels_driver_keyboard_node')
    node.declare_parameter(name="veh")
    veh = node.get_parameter(name="veh").get_parameter_value().string_value
    node.get_logger().info(f"Veh: {veh}")
    pub_wheels_cmd = node.create_publisher(
        WheelsCmdStamped,
        f"/{veh}/wheels_driver_node/wheels_cmd",
        10)
    pub_e_stop = node.create_publisher(
        BoolStamped,
        f"/{veh}/wheels_driver_node/emergency_stop",
        1)

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in ['q', 'e']:
                speed = change_speed(key=key, current_speed=speed)
                node.get_logger().info(f"Speed: {speed}")
                continue
            if key == 'z':
                e_stop_msg.data = not e_stop_msg.data
                pub_e_stop.publish(e_stop_msg)
                node.get_logger().info(f"Emergency stop: {e_stop_msg.data}")
                continue
            if e_stop_msg.data:
                node.get_logger().warn(f"Emergency stop activated")
            if key == 'w':
                wheels_msg.vel_left = speed
                wheels_msg.vel_right = speed
            elif key == 's':
                wheels_msg.vel_left = -speed
                wheels_msg.vel_right = -speed
            elif key == 'x':
                wheels_msg.vel_left = 0.0
                wheels_msg.vel_right = 0.0
            elif key == 'a':
                wheels_msg.vel_left = -speed
                wheels_msg.vel_right = speed
            elif key == 'd':
                wheels_msg.vel_left = speed
                wheels_msg.vel_right = -speed
            elif key == '\x03':
                break
            else:
                wheels_msg.vel_left = 0.0
                wheels_msg.vel_right = 0.0

            wheels_msg.header.stamp = node.get_clock().now().to_msg()
            pub_wheels_cmd.publish(wheels_msg)
            node.get_logger().info(f"Command sent, left {wheels_msg.vel_left},\
 right {wheels_msg.vel_right}")
    except Exception as e:
        node.get_logger().fatal(e)
    finally:
        wheels_msg.header.stamp = node.get_clock().now().to_msg()
        wheels_msg.vel_left = 0.0
        wheels_msg.vel_right = 0.0
        pub_wheels_cmd.publish(wheels_msg)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
