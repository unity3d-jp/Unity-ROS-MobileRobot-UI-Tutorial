#!/usr/bin/env python
# -*- coding: utf-8 -*-

# distance_sensor_simulator.py
# (C) 2016 - 2020 RT Corporation <support@rt-net.jp>
# Released under the MIT License
# https://opensource.org/licenses/MIT

import rospy
import math
from raspimouse_msgs.msg import LightSensorValues
from sensor_msgs.msg import LaserScan


def range_to_led(range_value):
    try:
        distance = int(range_value[0]*1000) # distance[mm]
        if distance < 4: distance = 8 - distance
        # This formula is calculated from the measurement result of actual sensor.
        # https://rt-net.jp/mobility/archives/3361
        led_value = int( 761000 / math.pow(distance,1.66) )
        if led_value > 4000: led_value = 4000
        if led_value < 15: led_value = 15
    except:
        led_value = 15 
    return led_value

def sensor1_callback(data):
    sensor1.ranges = data.ranges

def sensor2_callback(data):
    sensor2.ranges = data.ranges

def sensor3_callback(data):
    sensor3.ranges = data.ranges

def sensor4_callback(data):
    sensor4.ranges = data.ranges

def listener():
    rospy.Subscriber("{}raspimouse_on_gazebo/rf_scan".format(rospy.get_namespace()), LaserScan, sensor1_callback)
    rospy.Subscriber("{}raspimouse_on_gazebo/rs_scan".format(rospy.get_namespace()), LaserScan, sensor2_callback)
    rospy.Subscriber("{}raspimouse_on_gazebo/ls_scan".format(rospy.get_namespace()), LaserScan, sensor3_callback)
    rospy.Subscriber("{}raspimouse_on_gazebo/lf_scan".format(rospy.get_namespace()), LaserScan, sensor4_callback)

def talker():
    if not rospy.is_shutdown():
        try:
            d = LightSensorValues()
            d.right_forward = range_to_led(sensor1.ranges)
            d.right_side = range_to_led(sensor2.ranges)
            d.left_side = range_to_led(sensor3.ranges)
            d.left_forward = range_to_led(sensor4.ranges)
            d.sum_all = range_to_led(sensor1.ranges) + range_to_led(sensor2.ranges) \
                        + range_to_led(sensor3.ranges) + range_to_led(sensor4.ranges)
            d.sum_forward = range_to_led(sensor1.ranges) + range_to_led(sensor4.ranges)
            pub.publish(d)
        except:
            rospy.logerr("Failed to convert sensor data")
        r.sleep()

if __name__ == "__main__":
    rospy.init_node("led_sensor_publisher", anonymous=True)
    sensor1 = LaserScan()
    sensor2 = LaserScan()
    sensor3 = LaserScan()
    sensor4 = LaserScan()

    pub = rospy.Publisher('lightsensors', LightSensorValues, queue_size=10)
    r = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            listener()
            talker()
    except rospy.ROSInterruptException:
        pass