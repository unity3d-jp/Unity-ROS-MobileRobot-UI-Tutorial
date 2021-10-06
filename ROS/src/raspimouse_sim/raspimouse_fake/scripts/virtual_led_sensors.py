#!/usr/bin/env python
# -*- coding: utf-8 -*-

# (C) 2016 Daisuke Sato <tiryoh@gmail.com>
# (C) 2020 RT Corporation <support@rt-net.jp>
# Released under the MIT License 
# https://opensource.org/licenses/MIT

from __future__ import print_function
import rospy, math
from sensor_msgs.msg import LaserScan


def range_to_led(range_value):
    try:
        distance = int(range_value[0] * 1000)  # distance[mm]
        if distance < 4: distance = 8 - distance
        # This formula is calculated from the measurement result of actual sensor.
        # https://rt-net.jp/mobility/archives/3361
        led_value = int(761000 / math.pow(distance, 1.66))
        if led_value > 4000: led_value = 4000
        if led_value < 15: led_value = 15
    except:
        led_value = 15
    return led_value


def write_to_file(data):
    try:
        with open("/dev/rtlightsensor0", "w") as f:
            print("%d %d %d %d" % tuple(data), file=f)
    except:
        rospy.logerr("failed to open rtlightsensor0")


def sensor1_callback(data):
    led_val[0] = range_to_led(data.ranges)


def sensor2_callback(data):
    led_val[1] = range_to_led(data.ranges)


def sensor3_callback(data):
    led_val[2] = range_to_led(data.ranges)


def sensor4_callback(data):
    led_val[3] = range_to_led(data.ranges)
    write_to_file(led_val)


def listener():
    rospy.Subscriber(rospy.get_namespace() + "rf_scan", LaserScan, sensor1_callback)
    rospy.Subscriber(rospy.get_namespace() + "rs_scan", LaserScan, sensor2_callback)
    rospy.Subscriber(rospy.get_namespace() + "ls_scan", LaserScan, sensor3_callback)
    rospy.Subscriber(rospy.get_namespace() + "lf_scan", LaserScan, sensor4_callback)


if __name__ == "__main__":
    led_val = [15, 15, 15, 15]
    rospy.init_node("virtual_led_sensors", anonymous=True)
    listener()
    rospy.spin()
