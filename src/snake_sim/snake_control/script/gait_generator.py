#!/usr/bin/python3

import cls_gait
import rospy

from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel

#Return : Gait params, Simulation Duration
def _sim_signal(Duration = 5.0, Gait_name = 'vertical', AMP_Ver = 15, AMP_Hor = 45 , Phase_Ver = 60 , Phase_hor = 80, Time_delay = 110):

    # Declare gait class and time duration.
    input_gait = cls_gait.gait()
    time_duration = Duration

    # Starting gait parameters. The gait may have local minima so we need to optimize operation iteratively.
    input_gait.set_parameters(Gait_name,AMP_Ver,AMP_Hor,Phase_Ver,Phase_hor,Time_delay)

    return input_gait.get_parameters(), time_duration

print(_sim_signal())

def _reset_simulation():
    
