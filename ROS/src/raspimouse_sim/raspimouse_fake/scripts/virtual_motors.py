#!/usr/bin/env python
# -*- coding: utf-8 -*-

# (C) 2017 Yuki Watanabe <yukipass19@gmail.com>
# (C) 2018 Daisuke Sato <tiryoh@gmail.com>
# (C) 2020 RT Corporation <support@rt-net.jp>
# Released under the MIT License 
# https://opensource.org/licenses/MIT

import rospy, math
from geometry_msgs.msg import Twist
import subprocess


def get_motor_freq():
    swfile = "/dev/rtmotoren0"
    lfile = "/dev/rtmotor_raw_l0"
    rfile = "/dev/rtmotor_raw_r0"
    vel = Twist()
    sound_count = 0
    while not rospy.is_shutdown():
        try:
            with open(swfile, "r") as f:
                motor_power_status = f.readline().rstrip()
            if motor_power_status == "0":
                sound_count = 0
            if motor_power_status == "1" and sound_count == 0:
                subprocess.call("aplay $(rospack find raspimouse_fake)/misc/ms_sound.wav", shell=True)
                sound_count = 1
            if motor_power_status == "1":
                with open(lfile, "r") as lf, \
                        open(rfile, "r") as rf:
                    lhz_str = lf.readline().rstrip()
                    rhz_str = rf.readline().rstrip()
                if len(lhz_str) == 0:
                    lhz = 0
                else:
                    try:
                        lhz = int(lhz_str)
                    except:
                        lhz = 0
                if len(rhz_str) == 0:
                    rhz = 0
                else:
                    try:
                        rhz = int(rhz_str)
                    except:
                        rhz = 0
                vel.linear.x = (lhz + rhz) * 9 * math.pi / 160000.0
                vel.angular.z = (rhz - lhz) * math.pi / 800.0
                print(vel)
                pub.publish(vel)

        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    rospy.init_node("virtual_motors")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    get_motor_freq()
    rospy.spin()
