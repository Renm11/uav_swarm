#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped,TwistStamped
from std_msgs.msg import String#, Float32MultiArray
from mavros_msgs.srv import CommandBool,SetMode
import sys
import numpy

# 这个跟随者类订阅了：
#     1.自己的位置
#     2.领导者位置
#     3.防撞速度
#     4.编队形状
# 发布了：
#     1.控制命令（东北天坐标系下的命令，也是本程序中采用的）
#     2.本无人机信息
#     3.控制命令（这里没有使用）

class Follower:

    def __init__(self):
        # self.hover = "HOVER"
        # self.uav_type = uav_type
        # self.uav_num = uav_num
        # self.id = uav_id
        self.f = 30 #频率
        self.pose = PoseStamped()
        self.cmd_vel_enu = TwistStamped()
        # self.avoid_vel = Vector3()
        # self.formation_pattern = None
        self.Kp = 1.0 
        # self.Kp_avoid = 2.0
        # self.vel_max = 1.0
        # self.leader_pose = PoseStamped()

        self.pose_sub = rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=1)
        # self.avoid_vel_sub = rospy.Subscriber("/xtdrone/"+self.uav_type+'_'+str(self.id)+"/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)
        # self.formation_pattern_sub = rospy.Subscriber("/xtdrone/formation_pattern", Float32MultiArray, self.formation_pattern_callback, queue_size=1)

        self.vel_enu_pub = rospy.Publisher("/iris_0/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)#发布东北天下的控制命令;Twist是指的线速度和角速度
        # self.info_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/info', String, queue_size=1)
        # self.cmd_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd',String,queue_size=1)#机体坐标系下的控制命令（这里没有使用）

        self.arming_client_srv = rospy.ServiceProxy("/iris_0/mavros/cmd/arming", CommandBool)
        self.set_FCU_mode_srv = rospy.ServiceProxy("/iris_0/mavros/set_mode", SetMode)
        # self.leader_pose_sub = rospy.Subscriber("_0/mavros/local_position/pose", PoseStamped, self.leader_pose_callback, queue_size=1)

    # def formation_pattern_callback(self, msg):
    #     self.formation_pattern = numpy.array(msg.data).reshape(3, self.uav_num-1) #reshape用于改变数组的形状


    def pose_callback(self, msg):
        self.pose = msg #无人机当前位置，

    # def leader_pose_callback(self, msg):
    #     self.leader_pose = msg    

    # def avoid_vel_callback(self, msg):
    #     self.avoid_vel = msg

    def loop(self):
        rospy.init_node('follower')
        rate = rospy.Rate(100)
        # flight_mode = 'OFFBOARD'
        # flight_mode2 = "HOVER"
        # on_off_on="ARM"
        # on_off_off = "DISARM"
        # mSetMode offb_set_mode
        # offb_set_mode.request.custom_mode = "OFFBOARD"
        # set_FCU_mode_srv.call(offb_set_mode)
        # CommandBool arm_cmd
        # arm=True
        # arm_cmd.request.value = arm
        # arming_client_srv.call(arm_cmd)
        # self.armService(True) #解锁
        # self.flightModeService(custom_mode=flight_mode1) #设定模式
        while not rospy.is_shutdown():
            # self.set_FCU_mode_srv(custom_mode=flight_mode) #设定模式
            # self.arming_client_srv(True) #解锁
            if (self.arming_client_srv(True)):
			    print("Vehicle arming succeed!")
            if (self.set_FCU_mode_srv(custom_mode='OFFBOARD')):
                print("Vehicle offboard succeed!")
            else:   
			    print("Vehicle offboard failed!")
            # ROS_ERROR("sucess")
            # set_FCU_mode_srv.call(offb_set_mode)
            # arming_client_srv.call(arm_cmd)
            if (1):
                self.cmd_vel_enu.twist.linear.x = self.Kp * (0 - self.pose.pose.position.x) #直接用线速度不知道是否可以 答：可以，下面加上了判断是否超过最大线速度
                self.cmd_vel_enu.twist.linear.y = self.Kp * (5 - self.pose.pose.position.y) 
                self.cmd_vel_enu.twist.linear.z = self.Kp * (5 - self.pose.pose.position.z) 

                # self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.Kp_avoid * self.avoid_vel.x#在这里加上了躲避线速度
                # self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.Kp_avoid * self.avoid_vel.y
                # self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.Kp_avoid * self.avoid_vel.z
                # cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5 #线速度；x**y表示x的y次方
                # if cmd_vel_magnitude > 3**0.5 * self.vel_max:#线速度是否大于最大线速度的根号3倍
                #     self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
                #     self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
                #     self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max
                
                self.vel_enu_pub.publish(self.cmd_vel_enu)#发布东北天下的坐标系指令

            try:
                rate.sleep()
            except:
                continue

if __name__ == '__main__':
    follower = Follower()
    follower.loop()   