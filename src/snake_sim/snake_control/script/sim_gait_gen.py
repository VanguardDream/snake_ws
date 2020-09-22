#!/usr/bin/python3

import rospy
import sys
import math
import threading


from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics


from geometry_msgs.msg import Vector3
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
os_delay_sec = rospy.Duration(nsecs=50000)
delay_sec = rospy.Duration(nsecs=3000)
phase_ver =  3.1415 / 6
phase_hor =  3.1415 / 6

gazebo_pause = True

for i in range(16):
    thetas.append(0)

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

def clearSimulation():
    pauseSimulation()
    resetWorld()
    pauseSimulation()

def setTimer(sec = 5, func = clearSimulation):
    threading.Timer(sec,func).start()

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


def motionCalculate():
    global count

    thetas[0] = 0.46565384 * math.cos(count + phase_ver * 1)
    thetas[2] = 0.46565384 * math.cos(count + phase_ver * 3)
    thetas[4] = 0.46565384 * math.cos(count + phase_ver * 5)
    thetas[6] = 0.46565384 * math.cos(count + phase_ver * 7)
    thetas[8] = 0.46565384 * math.cos(count + phase_ver * 9)
    thetas[10] = 0.46565384 * math.cos(count + phase_ver * 11)
    thetas[12] = 0.46565384 * math.cos(count + phase_ver * 13)
    thetas[14] = 0.46565384 * math.cos(count + phase_ver * 15)
     
    count+=1

def commandSend():
    rospy.sleep(os_delay_sec)
    pub_com_1.publish(thetas[0])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_2.publish(thetas[1])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_3.publish(thetas[2])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_4.publish(thetas[3])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_5.publish(thetas[4])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_6.publish(thetas[5])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_7.publish(thetas[6])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_8.publish(thetas[7])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_9.publish(thetas[8])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_10.publish(thetas[9])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_11.publish(thetas[10])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_12.publish(thetas[11])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_13.publish(thetas[12])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_14.publish(thetas[13])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_15.publish(thetas[14])
    rospy.sleep(delay_sec)

    rospy.sleep(os_delay_sec)
    pub_com_16.publish(thetas[15])
    rospy.sleep(delay_sec)

if __name__ == '__main__':
    rospy.init_node('snake_gait_generator',anonymous=True)
    rate = rospy.Rate(20)
    pauseSimulation()

    gazeboPhysicsSet(max_update_rate_value=5000)

    t_prior = rospy.Time.now()

    while not rospy.is_shutdown():

        t_now = rospy.Time.now()

        if t_now - t_prior > rospy.Duration(secs=10):
            t_prior =  t_now
            clearSimulation()     

        motionCalculate()

        commandSend()
        rospy.sleep(delay_sec)
        rate.sleep()
        # rospy.sleep(0.5)
        # rospy.spin()
        
        
