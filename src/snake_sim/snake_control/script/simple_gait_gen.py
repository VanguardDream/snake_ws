#!/usr/bin/python3

import rospy
import sys
import math
import threading
import csv
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

pub_com_1 = rospy.Publisher('/snake/1_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_2 = rospy.Publisher('/snake/2_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_3 = rospy.Publisher('/snake/3_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_4 = rospy.Publisher('/snake/4_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_5 = rospy.Publisher('/snake/5_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_6 = rospy.Publisher('/snake/6_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_7 = rospy.Publisher('/snake/7_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_8 = rospy.Publisher('/snake/8_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_9 = rospy.Publisher('/snake/9_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_10 = rospy.Publisher('/snake/10_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_11 = rospy.Publisher('/snake/11_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_12 = rospy.Publisher('/snake/12_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_13 = rospy.Publisher('/snake/13_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_14 = rospy.Publisher('/snake/14_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_15 = rospy.Publisher('/snake/15_joint_position_controller/command', data_class=Float64, queue_size= 1)
pub_com_16 = rospy.Publisher('/snake/16_joint_position_controller/command', data_class=Float64, queue_size= 1)

thetas = []
count = 0
os_delay_sec = rospy.Duration(nsecs=800000)
delay_sec = rospy.Duration(nsecs=3000)
amp_ver = (3.1415 / 180) * 60
amp_hor = (3.1415 / 180) * 2.5
phase_ver =  (3.1415 / 180) * 25
phase_hor = (3.1415 / 180) * 35

gait_type = "vertical"
gazebo_pause = True

for i in range(16):
    thetas.append(0)

def gazeboPhysicsSet(time_step_value = 0.001, max_update_rate_value = 1000):
# Set as Default Gazebo Property
    gravity_value = Vector3(x = 0, y = 0, z = -9.81)
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


def motionCalculate(gait):
    global count

    if gait == 'vertical':
        thetas[0] = amp_ver * math.cos(count + phase_ver * 1)
        thetas[2] = amp_ver * math.cos(count + phase_ver * 3)
        thetas[4] = amp_ver * math.cos(count + phase_ver * 5)
        thetas[6] = amp_ver * math.cos(count + phase_ver * 7)
        thetas[8] = amp_ver * math.cos(count + phase_ver * 9)
        thetas[10] = amp_ver * math.cos(count + phase_ver * 11)
        thetas[12] = amp_ver * math.cos(count + phase_ver * 13)
        thetas[14] = amp_ver * math.cos(count + phase_ver * 15)

    elif gait == 'sinuous':
        thetas[0] = amp_ver * math.cos(count)
        thetas[1] = amp_hor * math.cos(0.5 * count + phase_hor * 1.5)

        thetas[2] = amp_ver * math.cos(count + phase_ver * 2)
        thetas[3] = amp_hor * math.cos(0.5 * count + phase_hor * 2.5)

        thetas[4] = amp_ver * math.cos(count + phase_ver * 4)
        thetas[5] = amp_hor * math.cos(0.5 * count + phase_hor * 3.5)

        thetas[6] = amp_ver * math.cos(count + phase_ver * 6)
        thetas[7] = amp_hor * math.cos(0.5 * count + phase_hor * 4.5)
        
        thetas[8] = amp_ver * math.cos(count + phase_ver * 8)
        thetas[9] = amp_hor * math.cos(0.5 * count + phase_hor * 5.5)

        thetas[10] = amp_ver * math.cos(count + phase_ver * 10)
        thetas[11] = amp_hor * math.cos(0.5 * count + phase_hor * 6.5)

        thetas[12] = amp_ver * math.cos(count + phase_ver * 12)
        thetas[13] = amp_hor * math.cos(0.5 * count + phase_hor * 7.5)

        thetas[14] = amp_ver * math.cos(count + phase_ver * 14)
        thetas[15] = amp_hor * math.cos(0.5 * count + phase_hor * 8.5)
    elif gait == 'sidewind':
        thetas[1] = amp_hor * math.cos(count)
        thetas[0] = amp_ver * math.cos(count + ( phase_ver/ 2) * 1)

        thetas[3] = amp_hor * math.cos(count + phase_hor * 1)
        thetas[2] = amp_ver * math.cos(count + ( phase_ver/ 2) * 3)

        thetas[5] = amp_hor * math.cos(count + phase_hor * 2)
        thetas[4] = amp_ver * math.cos(count + ( phase_ver/ 2) * 5)

        thetas[7] = amp_hor * math.cos(count + phase_hor * 3)
        thetas[6] = amp_ver * math.cos(count + ( phase_ver/ 2) * 7)

        thetas[9] = amp_hor * math.cos(count + phase_hor * 4)
        thetas[8] = amp_ver * math.cos(count + ( phase_ver/ 2) * 9)

        thetas[11] = amp_hor * math.cos(count + phase_hor * 5)
        thetas[10] = amp_ver * math.cos(count + ( phase_ver/ 2) * 11)

        thetas[13] = amp_hor * math.cos(count + phase_hor * 6)
        thetas[12] = amp_ver * math.cos(count + ( phase_ver/ 2) * 13)

        thetas[15] = amp_hor * math.cos(count + phase_hor * 7)
        thetas[14] = amp_ver * math.cos(count + ( phase_ver/ 2) * 15)
    
    count+=1

def commandZero():
    pub_com_1.publish(0.0)
    pub_com_2.publish(0.0)
    pub_com_3.publish(0.0)
    pub_com_4.publish(0.0)
    pub_com_5.publish(0.0)
    pub_com_6.publish(0.0)
    pub_com_7.publish(0.0)
    pub_com_8.publish(0.0)
    pub_com_9.publish(0.0)
    pub_com_10.publish(0.0)
    pub_com_11.publish(0.0)
    pub_com_12.publish(0.0)
    pub_com_13.publish(0.0)
    pub_com_14.publish(0.0)
    pub_com_15.publish(0.0)
    pub_com_16.publish(0.0)

def commandSend(gait):
    if (gait == 'vertical'):
        # rospy.sleep(os_delay_sec)
        pub_com_1.publish(thetas[0])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_3.publish(thetas[2])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_5.publish(thetas[4])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_7.publish(thetas[6])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_9.publish(thetas[8])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_11.publish(thetas[10])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_13.publish(thetas[12])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_15.publish(thetas[14])
        rospy.sleep(delay_sec)
    elif gait == 'sinuous':
        # rospy.sleep(os_delay_sec)
        pub_com_1.publish(thetas[0])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_2.publish(thetas[1])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_3.publish(thetas[2])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_4.publish(thetas[3])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_5.publish(thetas[4])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_6.publish(thetas[5])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_7.publish(thetas[6])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_8.publish(thetas[7])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_9.publish(thetas[8])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_10.publish(thetas[9])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_11.publish(thetas[10])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_12.publish(thetas[11])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_13.publish(thetas[12])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_14.publish(thetas[13])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_15.publish(thetas[14])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_16.publish(thetas[15])
        rospy.sleep(delay_sec)

    else:
        # rospy.sleep(os_delay_sec)
        pub_com_2.publish(thetas[1])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_1.publish(thetas[0])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_4.publish(thetas[3])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_3.publish(thetas[2])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_6.publish(thetas[5])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_5.publish(thetas[4])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_8.publish(thetas[7])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_7.publish(thetas[6])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_10.publish(thetas[9])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_9.publish(thetas[8])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_12.publish(thetas[11])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_11.publish(thetas[10])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_14.publish(thetas[13])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_13.publish(thetas[12])
        rospy.sleep(delay_sec)

        # rospy.sleep(os_delay_sec)
        pub_com_16.publish(thetas[15])
        rospy.sleep(delay_sec)

def pauseSimulation():
    global gazebo_pause
    if gazebo_pause:
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            gazebo_pause = False
            unpause_gazebo()
            print('Gazebo Simulation is Starting Now...')
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
    else:
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            pause_gazebo = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            gazebo_pause = True
            pause_gazebo()
            print('Gazebo Simulation is Stopping Now...')
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

    # rospy.wait_for_service('/gazebo/reset_simulation')
    # try:
    #     reset_sim = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
    #     reset_sim()
    #     pass
    # except rospy.ServiceException as exc:
    #     print("Service did not process request: " + str(exc))
    #     pass



if __name__ == '__main__':
    rospy.init_node('snake_gait_generator',anonymous=True)
    rate = rospy.Rate(20)
    pauseSimulation()

    gazeboPhysicsSet(max_update_rate_value=1000)

    t_prior = rospy.Time.now()

    while not rospy.is_shutdown():

        t_now = rospy.Time.now()

        if t_now - t_prior > rospy.Duration(secs=10):
            pauseSimulation()
            
            resetWorld()

            pauseSimulation()

            t_prior =  rospy.Time.now()

            continue   

        motionCalculate(gait_type)

        commandSend(gait_type)
        rospy.sleep(delay_sec)
        rate.sleep()
        # rospy.sleep(0.5)
        # rospy.spin()