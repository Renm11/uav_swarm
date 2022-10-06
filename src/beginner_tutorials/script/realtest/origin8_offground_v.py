#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped,TwistStamped
from std_msgs.msg import String#, Float32MultiArray
from mavros_msgs.srv import CommandBool,SetMode
import sys
import numpy

class Follower:

    def __init__(self):
        self.pose = PoseStamped()
        self.cmd_vel_enu = TwistStamped()
        self.Kp = 1.0 
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=1)
        self.vel_enu_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)#发布东北天下的控制命令;Twist是指的线速度和角速度
        self.arming_client_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_FCU_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        
    def pose_callback(self, msg):
        self.pose = msg #无人机当前位置
        print("current pose x y z:",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)

    def loop(self):
        rospy.init_node('follower')
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if (self.arming_client_srv(True)):
			    print("Vehicle arming succeed!")
            if (self.set_FCU_mode_srv(custom_mode='OFFBOARD')):
                print("Vehicle offboard succeed!")
            else:   
			    print("Vehicle offboard failed!")
            if (1):
                self.cmd_vel_enu.twist.linear.x = self.Kp * (0 - self.pose.pose.position.x) #直接用线速度不知道是否可以 答：可以，下面加上了判断是否超过最大线速度
                self.cmd_vel_enu.twist.linear.y = self.Kp * (0 - self.pose.pose.position.y) 
                self.cmd_vel_enu.twist.linear.z = self.Kp * (1.3 - self.pose.pose.position.z) 

            self.vel_enu_pub.publish(self.cmd_vel_enu)#发布东北天下的坐标系指令

            try:
                rate.sleep()
            except:
                continue

if __name__ == '__main__':
    follower = Follower()
    follower.loop()   