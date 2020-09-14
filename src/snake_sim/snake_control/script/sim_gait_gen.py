#!/usr/bin/python3

import rospy
import sys
import math

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

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
delay_sec = rospy.Duration(nsecs=3000)
phase_ver =  3.1415 / 6
phase_hor =  3.1415 / 6

for i in range(16):
    thetas.append(0)

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
    pub_com_1.publish(thetas[0])
    rospy.sleep(delay_sec)
    pub_com_2.publish(thetas[1])
    rospy.sleep(delay_sec)
    pub_com_3.publish(thetas[2])
    rospy.sleep(delay_sec)
    pub_com_4.publish(thetas[3])
    rospy.sleep(delay_sec)
    pub_com_5.publish(thetas[4])
    rospy.sleep(delay_sec)
    pub_com_6.publish(thetas[5])
    rospy.sleep(delay_sec)
    pub_com_7.publish(thetas[6])
    rospy.sleep(delay_sec)
    pub_com_8.publish(thetas[7])
    rospy.sleep(delay_sec)
    pub_com_9.publish(thetas[8])
    rospy.sleep(delay_sec)
    pub_com_10.publish(thetas[9])
    rospy.sleep(delay_sec)
    pub_com_11.publish(thetas[10])
    rospy.sleep(delay_sec)
    pub_com_12.publish(thetas[11])
    rospy.sleep(delay_sec)
    pub_com_13.publish(thetas[12])
    rospy.sleep(delay_sec)
    pub_com_14.publish(thetas[13])
    rospy.sleep(delay_sec)
    pub_com_15.publish(thetas[14])
    rospy.sleep(delay_sec)
    pub_com_16.publish(thetas[15])
    rospy.sleep(delay_sec)

if __name__ == '__main__':
    rospy.init_node('snake_gait_generator',anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        motionCalculate()

        commandSend()

        rate.sleep()
        # rospy.sleep(0.5)
        # rospy.spin()
        
        
