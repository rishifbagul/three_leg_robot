import rospy
import time
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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
import math


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

        self.joint_publisher=[rospy.Publisher('/robot_6_joint/joint1_position_controller/command',Float64,queue_size=1),\
                        rospy.Publisher('/robot_6_joint/joint2_position_controller/command',Float64,queue_size=1),\
                        rospy.Publisher('/robot_6_joint/joint3_position_controller/command',Float64,queue_size=1),\
                        rospy.Publisher('/robot_6_joint/joint4_position_controller/command',Float64,queue_size=1),\
                        rospy.Publisher('/robot_6_joint/joint5_position_controller/command',Float64,queue_size=1),\
                        rospy.Publisher('/robot_6_joint/joint6_position_controller/command',Float64,queue_size=1)]

        
        self.joint_state_subscriber = rospy.Subscriber('/robot_6_joint/joint_states',JointState,
                                                       self.joint_state_subscriber_callback)

        self.joint_name_list = ['base_leg1_joint','base_leg2_joint','base_leg3_joint','leg1_joint','leg2_joint','leg3_joint']
        
        self.starting_pos = [3.0 , 3.0, 3.0, 1.5, 1.5, 1.5]
        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.get_model_state_req = GetModelStateRequest()
        self.get_model_state_req.model_name = 'three_leg_robot'
        self.get_model_state_req.relative_entity_name = 'world'

        self.state_joint_angle=self.starting_pos
        self.next_state_joint_angle=self.starting_pos

        self.joint_max_angle=[math.pi,math.pi,math.pi,math.pi/2,math.pi/2,math.pi/2]
        self.joint_min_angle=[0,0,0,-math.pi/2,-math.pi/2,-math.pi/2]
        
        self.target_x=2
        self.target_y=0
        #self.last_reward_x=0
        #self.last_reward_y=0
        self.reset_robot()
        
    def reset_robot(self):
        
        print("resting Enviorment")
        # self.move_joints(self.starting_pos)
        # time.sleep(1)
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_engine()
        except :
            print("physics couldnt started")

        list_of_joint_angles=[0.7,1,1,1,1,1]
        for i in range(len(list_of_joint_angles)):
            self.joint_publisher[i].publish(list_of_joint_angles[i])
        self.next_state_joint_angle=list_of_joint_angles
        time.sleep(3)
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
        
        time.sleep(2)
        
        model_state= self.get_model_state()
        #self.reward_last_x=model_state.pose.position.x
        #self.reward_last_y=model_state.pose.position.y
        return self.get_state(model_state)
        
    def move_joints(self,list_of_joint_angles):
        try:
            self.unpause_engine()
        except :
            print("physics couldnt started")
            
        for i in range(len(list_of_joint_angles)):
            self.joint_publisher[i].publish(list_of_joint_angles[i])
        self.next_state_joint_angle=list_of_joint_angles
        
        time.sleep(0.1)
        try:
            self.pause_engine()
        except:
            print("gazebo couldnt paused")
    
    def get_state(self,model_state):
        state=self.get_state_joint_angle()
        for i in range(len(state)):
            state[i]=self.remap(state[i],self.joint_min_angle[i],self.joint_max_angle[i],0,1)
        x,y,z,w,yaw,Z,lx,ly,lz,ax,ay,az=self.get_state_pose(model_state)
        yaw=self.remap(yaw,-math.pi,math.pi,0,1)
        pose=np.array([x,y,z,w,yaw,Z,lx,ly,lz,ax,ay,az])
        return np.concatenate((state,pose))

    
    def joint_state_subscriber_callback(self,joint_state):
        self.state_joint_angle = np.array(joint_state.position)

    def get_state_joint_angle(self):
        return self.state_joint_angle
    
    def get_model_state(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        model_state = self.get_model_state_proxy(self.get_model_state_req)
        #print("model state",model_state)
        return model_state
    
    def get_state_pose(self,model_state):
        x=model_state.pose.orientation.x
        y=model_state.pose.orientation.y
        z=model_state.pose.orientation.z
        w=model_state.pose.orientation.w
        Z=model_state.pose.position.z
        lx=model_state.twist.linear.x
        ly=model_state.twist.linear.y
        lz=model_state.twist.linear.z
        ax=model_state.twist.angular.x
        ay=model_state.twist.angular.y
        az=model_state.twist.angular.z
        
        ro,pi,yaw=euler_from_quaternion([x,y,z,w])
        return x,y,z,w,yaw,Z,lx,ly,lz,ax,ay,az


    def remap(self,value,low_from,high_from,low_to,high_to):
        if value>high_from:
            value=high_from
        elif value<low_from:
            value=low_from
        return (value-low_from)*((high_to-low_to)/(high_from-low_from))+low_to
    
    def calculate_reward(self,model_state):                       #make changes when ever target is changed
        x=model_state.pose.orientation.x
        y=model_state.pose.orientation.y
        z=model_state.pose.orientation.z
        w=model_state.pose.orientation.w
        X=model_state.pose.position.x
        Y=model_state.pose.position.y
        Z=model_state.pose.position.z

        roll,pitch,yaw=euler_from_quaternion([x,y,z,w])
        if roll > 2 or roll < -2 or pitch > 2 or pitch < -2 or Z < 0.1:
            done=True
            reward=-20000
        else:
            done=False
        
        if X > self.target_x and ( Y>self.target_y-0.2 and Y<self.target_y+0.2):
            done=True
            reward=100
        
        if not done:
            reward=-100*(math.sqrt((self.target_x-X)**2 + (self.target_y-Y)**2))
        
        return reward,done
    
    def perform_one_step(self,action):
        for i in range(len(action)):
            action[i]=self.remap(action[i],-1,1,self.joint_min_angle[i],self.joint_max_angle[i])
        self.move_joints(action)
        # time.sleep(0.1)
        model_state=self.get_model_state()
        next_state=self.get_state(model_state)
        raward,done = self.calculate_reward(model_state)
        return next_state,raward,done
    
    
            
        
        




# env = Gazebo_enviorment()
# env.reset_robot()

# for i in range(10):
#     time.sleep(2)
#     action=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
#     print(env.perform_one_step(action))

# pos = np.array([math.pi,math.pi,math.pi,math.pi/2,math.pi/2,math.pi/2])
# env.move_joints(pos)

# time.sleep(2)

# pos = np.array([math.pi,math.pi,math.pi,math.pi/2,math.pi/2,math.pi/2])
# env.move_joints(pos)

# time.sleep(2)

# pos = np.array([0,0,0,-math.pi/2,-math.pi/2,-math.pi/2])
# env.move_joints(pos)

# time.sleep(2)

# pos = np.array([0,0,0,-math.pi/2,-math.pi/2,-math.pi/2])
# env.move_joints(pos)
