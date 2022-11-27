import rospy
import time
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Imu
import random
import numpy as np


class Gazebo_enviorment:
    def __init__(self):
        rospy.init_node('joint_position_node')
        self.pause_engine = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.unpause_engine = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        
        self.model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        self.model_state_req = SetModelStateRequest()
        self.model_state_req.model_state = ModelState()
        self.model_state_req.model_state.model_name = 'three_leg_robot'
        self.model_state_req.model_state.pose.position.z = 0.8
        self.model_state_req.model_state.reference_frame = 'world'

        self.joint_publisher=[rospy.Publisher('/robot_6_joint/joint1_position_controller/command',Float64,queue_size=2),\
                        rospy.Publisher('/robot_6_joint/joint2_position_controller/command',Float64,queue_size=2),\
                        rospy.Publisher('/robot_6_joint/joint3_position_controller/command',Float64,queue_size=2),\
                        rospy.Publisher('/robot_6_joint/joint4_position_controller/command',Float64,queue_size=2),\
                        rospy.Publisher('/robot_6_joint/joint5_position_controller/command',Float64,queue_size=2),\
                        rospy.Publisher('/robot_6_joint/joint6_position_controller/command',Float64,queue_size=2)]

        self.joint_name_list = ['base_leg1_joint','base_leg2_joint','base_leg3_joint','leg1_joint','leg2_joint','leg3_joint']
        
        self.starting_pos = np.array([1.0 , 1.0, 1.0,
                                     1.0, 1.0, 1.0])
        

        
    def reset_robot(self):
        
        self.move_joints([1,1,1,1,1,1])
        time.sleep(1)
        self.move_joints([1,1,1,1,1,1])
        time.sleep(2)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_engine()
        except:
            print("gazebo couldnt paused")
        
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.model_state_proxy(self.model_state_req)
        except:
            print('/gazebo/set_model_state call failed')

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_engine()
        except :
            print("physics couldnt started")
        
        
    def move_joints(self,list_of_joint_angles):
        for i in range(len(list_of_joint_angles)):
            self.joint_publisher[i].publish(list_of_joint_angles[i])
    


env=Gazebo_enviorment()
env.reset_robot()
time.sleep(5)
env.move_joints([0,0,0,0,0,0])

# for i in range(-150,150,10):
#     env.move_joints([i/100,i/100,i/100,i/100,i/100,i/100])
#     print(i)
#     time.sleep(0.2)
# for i in range(150,-150,-10):
#     env.move_joints([i/100,i/100,i/100,i/100,i/100,i/100])
#     print(i)
#     time.sleep(0.2)
