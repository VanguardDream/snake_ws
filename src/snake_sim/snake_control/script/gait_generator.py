#!/usr/bin/python3

import cls_gait
import rospy
import math
import threading
import actionlib

#Gazebo msgs.
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel

#snake_control msgs.
from snake_control.msg import GaitParameter

_PI = 3.1415
input_gait = cls_gait.gait()
sim_generator = cls_gait.generator()

#Return : Gait params, Simulation Duration
def _sim_signal(Duration = 5.0, Gait_name = 'vertical', AMP_Ver = 15, AMP_Hor = 45 , Phase_Ver = 60 , Phase_hor = 80, Time_delay = 110, Time_period = 100):

    # Declare gait class and time duration.
    global input_gait
    global sim_generator

    time_duration = Duration

    # Starting gait parameters. The gait may have local minima so we need to optimize operation iteratively.
    input_gait.set_parameters(Gait_name,AMP_Ver,AMP_Hor,Phase_Ver,Phase_hor,Time_delay,Time_period)

    # Load parameter to generator
    sim_generator.gaitType = input_gait

    return input_gait.get_parameters(), time_duration

def _set_sim_timer():
    thread_sim_reset.start()
    pass

def _stop_sim_timer():
    thread_sim_reset._stop()
    thread_sim_reset.join()
    pass

def _reset_simulation():
    while True:
        print('reset done!')
        global sim_reset_rate
        sim_reset_rate.sleep()

def _calculate_gait(counter): #이 함수는 해상도 만큼의 슬롯으로 머리의 각도를 미리 계산하는 함수
    #카운터의 주기는 0.01초 (10ms) 인 것으로 가정함.
    for i in range(16):
        if i % 2 == 0:
            sim_generator.motorTheta[i] = round(sim_generator.gaitType.ampVertical * math.sin( (2 * _PI / (sim_generator.gaitType.timePeriod)) * counter + (i / 2) * (_PI/180.0) * sim_generator.gaitType.phaseVertical), 1)
        else:
            sim_generator.motorTheta[i] = round(sim_generator.gaitType.ampHorizontal * math.sin( (2 * _PI / (sim_generator.gaitType.timePeriod)) * counter + ((i - 1) / 2) * (_PI/180.0) * sim_generator.gaitType.phaseHorizontal), 1)
    return 0

def _command_gait():
    pass

def _sim_signal_cb(data):
    if(data.period != 0):
        global cal_rate
        
        _sim_signal(data.duration, data.gaitType, data.amp_v, data.amp_h, data.phase_v, data.phase_h, data.delay, data.period)
        cal_rate = rospy.Rate(1000/data.delay)

#Init ROS node.
rospy.init_node('snake_gait_generator',anonymous=True)

#Subscribe Signal (GaitParamter)
sim_signal = rospy.Subscriber('sim_signal', GaitParameter, _sim_signal_cb)

#Set Gait Parameter & ROS Rate
_sim_signal(Duration = 5.0, Gait_name = 'vertical', AMP_Ver = 30, AMP_Hor = 45 , Phase_Ver = 60 , Phase_hor = 80, Time_delay = 110, Time_period = 100)
motorRate = 1000/input_gait.timeDelay
cal_rate = rospy.Rate(motorRate)
sim_reset_rate = rospy.Rate(0.2)

counter = 0

# 알고리즘1 - 5초 혹은 정해진 시간마다 reset 시키고 결과 값을 출력하는 알고리즘
thread_sim_reset = threading.Thread(target=_reset_simulation, args=())
thread_sim_reset.setDaemon(True)

_set_sim_timer()

while not rospy.is_shutdown():
    _calculate_gait(counter)
    print(sim_generator.motorTheta)
    counter = counter + 1
    cal_rate.sleep()

    if counter > 100:
        _stop_sim_timer()

    # 알고리즘2 - 






# for i in range(101):
#     _calculate_gait(i)
#     print(sim_generator.motorTheta)
#     rospy.sleep(0.01)
