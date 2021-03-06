#!/usr/bin/python3

import rospy
import sys
import math
import threading
import csv
import time
import random
import numpy as np

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


# Variable for Optimization
gait_case = 0
findGradientDirection = True
flagLocalMinima = False
num_op_variable = 2
origin_variable = [0,0,0,0,0,-0.1] #A_V, A_H, P_V, P_H, Distance(k), Last Distance(k-1)
direction_vector = [0,0,0,0] #A_V, A_H, P_V, P_H
optimized_vector = [0,0,0,0] #A_V, A_H, P_V, P_H
step_size = 0.5
dA = 1  #Different of Amplitude
dP = 1  #Different of Phase

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

ros_secs = 0
ros_nsecs = 0

gazebo_model_pose_x = 0.0
gazebo_model_pose_y = 0.0
gazebo_model_pose_z = 0.0

csv_file = open('sim_result.csv', 'a', encoding='utf-8', newline='')
csv_file.close()
sim_data_buffer = []

thetas = []
count = 0
os_delay_sec = rospy.Duration(nsecs=5000000)
delay_sec = rospy.Duration(0.01)
phase_ver = (3.1415 / 180) * 25
phase_hor = (3.1415 / 180) * 60

amp_ver = (3.1415 / 180) * 60
amp_hor = (3.1415 / 180) * 20

gait_type = 'vertical'

gazebo_pause = True

for i in range(16):
    thetas.append(0)

def callback_clock(data):
    global ros_secs
    global ros_nsecs
    ros_secs = data.clock.secs
    ros_nsecs = data.clock.nsecs

def callback_states(data):
    global ros_secs
    global ros_nsecs

    global gazebo_model_pose_x
    global gazebo_model_pose_y
    global gazebo_model_pose_z

    global sim_data_buffer

    gazebo_model_pose_x = data.pose[1].position.x
    gazebo_model_pose_y = data.pose[1].position.y
    gazebo_model_pose_z = data.pose[1].position.z

    sim_data_buffer.append([ros_secs,ros_nsecs,gazebo_model_pose_x,gazebo_model_pose_y,gazebo_model_pose_z])
    # print("%d | %d | %f" %(ros_secs,ros_nsecs,gazebo_model_pose_x),end='\n')

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

def clearSimulation():
    global csv_file
    global sim_data_buffer
    global amp_ver
    global amp_hor
    global phase_ver
    global phase_hor
    global os_delay_sec
    global gait_type
    global gait_case
    global count
    global num_op_variable
    global findGradientDirection
    global origin_variable
    global flagLocalMinima

    commandZero()

    rospy.sleep(rospy.Duration(1.0))

    pauseSimulation()
    # resetSimulation()

    diff_x = []
    diff_y = []
    diff_z = []
    final_x = 0
    final_y = 0
    final_z = 0

    count = 0

    for i in range(len(sim_data_buffer)):
        diff_x.append(sim_data_buffer[i][2])
        diff_y.append(sim_data_buffer[i][3])
        diff_z.append(sim_data_buffer[i][4])

    if abs(min(diff_x)) > abs(max(diff_x)):
        final_x = min(diff_x)
    else:
        final_x = max(diff_x)

    if abs(min(diff_y)) > abs(max(diff_y)):
        final_y = min(diff_y)
    else:
        final_y = max(diff_y)

    if abs(min(diff_z)) > abs(max(diff_z)):
        final_z = min(diff_z)
    else:
        final_z = max(diff_z)


    # CSV File writer
    csv_line_writer = csv.writer(csv_file)

    csv_line_writer.writerow([str(time.strftime('%c', time.localtime(time.time()))), "end", gait_type, delay_sec.to_sec(), amp_ver / (3.1415 /180), phase_ver / (3.1415 /180), amp_hor / (3.1415 /180), phase_hor / (3.1415 /180), final_x, final_y, final_z, flagLocalMinima, optimized_vector[0],optimized_vector[1],optimized_vector[2],optimized_vector[3]])

    csv_file.close()
    
    resetWorld()

    # My Optimizer
    if findGradientDirection:
        Op_direction_find(final_x,final_y,final_z)
    else:
        findGradientDirection = True
        num_op_variable = 2

        amp_ver = random.randint(0,12) * 5 * (3.1415 /180)
        amp_hor = random.randint(0,12) * 5 * (3.1415 /180)
        phase_ver = random.randint(0,36)* 10 * (3.1415 /180)
        phase_hor = random.randint(0,36)* 10 * (3.1415 /180)

    num_op_variable = 2 #for 무한반복
    findGradientDirection = True
    flagLocalMinima = False

    if flagLocalMinima:
        csv_file = open('sim_result.csv', 'a', encoding='utf-8', newline='')

        csv_line_writer = csv.writer(csv_file)

        csv_line_writer.writerow([str(time.strftime('%c', time.localtime(time.time()))), "Local Minima Found Reset Gait", gait_type, delay_sec.to_sec(), amp_ver / (3.1415 /180), phase_ver / (3.1415 /180), amp_hor / (3.1415 /180), phase_hor / (3.1415 /180)])

        csv_file.close()

        flagLocalMinima = False
    

    # gait_case = gait_case + 1

    # # Optimization Code - Sinuous
    # hor_case = gait_case // 432
    # ver_case = gait_case % 432

    # tmp_amp_ver = (ver_case // 36) % 12
    # tmp_phase_ver = ver_case % 36

    # tmp_amp_hor = hor_case // 36
    # tmp_phase_hor = hor_case % 36

    # amp_ver = (tmp_amp_ver + 1) * 5 * (3.1415 /180)
    # phase_ver = tmp_phase_ver * 10 * (3.1415 /180)

    # amp_hor = (tmp_amp_hor) * 5 * (3.1415 /180)
    # phase_hor = tmp_phase_hor * 10 * (3.1415 /180)

    # # Optimization Code - Ver
    # quotient = gait_case // 36
    # remainder = gait_case % 36
    
    # amp_ver = (quotient+1) * 5 * (3.1415 /180)
    # phase_ver = remainder * 10 * (3.1415 /180)

    # # Testing Randomize
    # amp_ver = random.randint(0,90) * (3.1415 /180)
    # amp_hor = random.randint(0,90) * (3.1415 /180)
    # phase_ver = random.randint(0,360) * (3.1415 /180)
    # phase_hor = random.randint(0,360) * (3.1415 /180)
    
    # tmp_sec = random.randint(50,800)
    # os_delay_sec = rospy.Duration(nsecs = tmp_sec * 10000)

    csv_file = open('sim_result.csv', 'a', encoding='utf-8', newline='')
    csv_line_writer = csv.writer(csv_file)
    #csv_line_writer.writerows(sim_data_buffer)
    sim_data_buffer.clear()

    # csv_line_writer.writerow([str(time.strftime('%c', time.localtime(time.time()))),os_delay_sec.to_sec(),amp_ver / (3.1415 /180),phase_ver / (3.1415 /180),"start"])

    commandZero()
    pauseSimulation()
    rospy.sleep(rospy.Duration(1.0))

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
            commandZero()
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

def resetSimulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        reset_simulation()

        print('Simulation is reseted!')
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

def setTimer(sec = 5, func = clearSimulation):
    threading.Timer(sec,func).start()


def Op_direction_find(x = 0,y = 0,z  =0):

    global findGradientDirection
    global flagLocalMinima
    global direction_vector
    global num_op_variable
    global origin_variable
    global amp_ver
    global amp_hor
    global phase_ver
    global phase_hor
    global optimized_vector
    
    if(findGradientDirection and num_op_variable == 2):
        #원래 벡터 저장
        origin_variable[0] = (round(    amp_ver / (3.1415 / 180) , 1   ))
        origin_variable[1] = (round(    amp_hor / (3.1415 / 180) , 1   ))
        origin_variable[2] = (round(    phase_ver / (3.1415 / 180) , 1   ))
        origin_variable[3] = (round(    phase_hor / (3.1415 / 180) , 1  ))
        origin_variable[4] = x

        if origin_variable[5] > origin_variable[4]:
            flagLocalMinima = True

            amp_ver = random.randint(0,12) * 5 * (3.1415 /180)
            amp_hor = random.randint(0,12) * 5 * (3.1415 /180)
            phase_ver = random.randint(0,36) * 10 * (3.1415 /180)
            phase_hor = random.randint(0,36) * 10 * (3.1415 /180)

            return 0 #Terminate Optimizing and reset variable
        else:
            optimized_vector[0] = origin_variable[0]
            optimized_vector[1] = origin_variable[1]
            optimized_vector[2] = origin_variable[2]
            optimized_vector[3] = origin_variable[3]




    if num_op_variable == 2:
        amp_ver = amp_ver + dA * (3.1415 /180)
        amp_hor = origin_variable[1] * (3.1415 /180)
        phase_ver = origin_variable[2] * (3.1415 /180)
        phase_hor = origin_variable[3] * (3.1415 /180)
        num_op_variable = num_op_variable - 1

    elif num_op_variable == 1:
        amp_ver = origin_variable[0] * (3.1415 /180)
        amp_hor = amp_hor + dA * (3.1415 /180)
        phase_ver = origin_variable[2] * (3.1415 /180)
        phase_hor = origin_variable[3] * (3.1415 /180)
        num_op_variable = num_op_variable - 1

        if (origin_variable[4] - x) < 0:
            direction_vector[0] = 1
        else:
            direction_vector[0] = -1

    elif num_op_variable == 0:
        # findGradientDirection = False

        if (origin_variable[4] - x) < 0:
            direction_vector[1] = 1
        else:
            direction_vector[1] = -1

        # Set k diff_x value to k-1 diff_x
        origin_variable[5] = origin_variable[4]

        amp_ver = (origin_variable[0] + dA * step_size) * (3.1415 /180)
        amp_hor = (origin_variable[1] + dA * step_size) * (3.1415 /180)
        phase_ver = (origin_variable[2] + dP * step_size) * (3.1415 /180)
        phase_hor = (origin_variable[3] + dP * step_size) * (3.1415 /180)

        num_op_variable = 2

    else:
        pass

if __name__ == '__main__':
    sub_clock = rospy.Subscriber('/clock',Clock,callback_clock,queue_size=1000)
    sub_model_states = rospy.Subscriber('/gazebo/model_states',ModelStates,callback_states,queue_size=1000)

    csv_file = open('sim_result.csv', 'a', encoding='utf-8', newline='')
    csv_line_writer = csv.writer(csv_file)
    # csv_line_writer.writerow([str(time.strftime('%c', time.localtime(time.time()))),os_delay_sec.to_sec(),amp_ver / (3.1415 /180),phase_ver / (3.1415 /180),"start"])

    rospy.init_node('snake_gait_generator',anonymous=True)
    rate = rospy.Rate(20)
    pauseSimulation()

    gazeboPhysicsSet(max_update_rate_value=1000)

    t_prior = rospy.Time.now()

    while not rospy.is_shutdown():
        
        rospy.loginfo("1")
        
        t_now = rospy.Time.now()

        if t_now - t_prior > rospy.Duration(secs=10):
            rospy.loginfo("3")
            clearSimulation()

            t_prior =  rospy.Time.now()

            continue     

        motionCalculate(gait_type)

        commandSend(gait_type)
        rospy.loginfo("1-")
        rate.sleep()
        # rospy.sleep(0.5)
        # rospy.spin()
        
        
