#!/usr/bin/python3

import rospy
import rospkg

from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.msg import ModelStates

def gazeboDeleteModel(time_step_value = 0.001, max_update_rate_value = 1000):
# Set as Default Gazebo Property
    ODE_value = ODEPhysics(auto_disable_bodies = False, sor_pgs_precon_iters = 0, sor_pgs_iters = 50, sor_pgs_w = 1.3, sor_pgs_rms_error_tol = 0, contact_surface_layer = 0.001, contact_max_correcting_vel = 100.0, cfm = 0.0, erp = 0.2, max_contacts = 20)

    gazeboModel = SpawnModel(model_name = "snake", model_xml = )

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


print(s)