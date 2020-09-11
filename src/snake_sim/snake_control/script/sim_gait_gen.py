#!/usr/bin/python3

import rospy
import sys

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
for i in range(16):
    thetas.append(0)

# def motionCalculate():
    

def commandSend():
    pub_com_1.publish(thetas[0])
    pub_com_2.publish(thetas[1])
    pub_com_3.publish(thetas[2])
    pub_com_4.publish(thetas[3])
    pub_com_5.publish(thetas[4])
    pub_com_6.publish(thetas[5])
    pub_com_7.publish(thetas[6])
    pub_com_8.publish(thetas[7])
    pub_com_9.publish(thetas[8])
    pub_com_10.publish(thetas[9])
    pub_com_11.publish(thetas[10])
    pub_com_12.publish(thetas[11])
    pub_com_13.publish(thetas[12])
    pub_com_14.publish(thetas[13])
    pub_com_15.publish(thetas[14])
    pub_com_16.publish(thetas[15])

if __name__ == '__main__':
    rospy.init_node('snake_gait_generator',anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # motionCalculate()

        commandSend()

        rate.sleep()
        
