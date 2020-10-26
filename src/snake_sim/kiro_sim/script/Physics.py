#!/usr/bin/python3

import rospy
import sys
import math
import time
import random

from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.msg import ModelStates

from rosgraph_msgs.msg import Clock

from geometry_msgs.msg import Vector3
from std_msgs.msg import Time
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

def gazeboPhysicsSet(time_step_value = 0.001, max_update_rate_value = 1000, gravity_x = 0, gravity_y = 0, gravity_z = -9.81):
# Set as Default Gazebo Property
    gravity_value = Vector3(x = gravity_x, y = gravity_y, z = gravity_z)
    ODE_value = ODEPhysics(auto_disable_bodies = False, sor_pgs_precon_iters = 0, sor_pgs_iters = 50, sor_pgs_w = 1.3, sor_pgs_rms_error_tol = 0, contact_surface_layer = 0.001, contact_max_correcting_vel = 100.0, cfm = 0.0, erp = 0.2, max_contacts = 20)

    rospy.wait_for_service('/gazebo/set_physics_properties')
    try:
        set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties',SetPhysicsProperties)
        service_result = set_physics_properties(max_update_rate = max_update_rate_value, time_step = time_step_value, gravity = gravity_value, ode_config = ODE_value)
        if(service_result):
            print('Physics properties is set as coded!')
        else:
            print('Service Does not Called!')
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

def resetWorld():
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()
        print('World is reseted!')
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))


if __name__ == '__main__':
    while(True):
        print("1 : reset world, 2 : set Physics params. 1? 2?")
        com_sel = int(input())

        if com_sel == 1:
            print("Reset World Service is called!")
            resetWorld()
            continue

        elif com_sel == 2:
            print("Gravity_x : ? (m/s^2)")
            gr_x = float(input())
            print("Gravity_y : ? (m/s^2)")
            gr_y = float(input())
            print("Gravity_z : ? (m/s^2)")
            gr_z = float(input())

            gazeboPhysicsSet(gravity_x=gr_x,gravity_y=gr_y,gravity_z=gr_z)

            print("Physics is set with "+str(gr_x)+" "+str(gr_y)+" "+str(gr_z))
            continue

        else:
            print("Wrong command.")
            continue
