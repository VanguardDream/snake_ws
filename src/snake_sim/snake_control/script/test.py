#!/usr/bin/python3

import rospy
import rospkg

s = rospkg.get_ros_package_path('rosserial')

print(s)