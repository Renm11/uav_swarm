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
        self.cur_pose = PoseStamped()
        self.targetPose = PoseStamped()
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=1)
        self.pose_enu_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)#发布东北天下的控制命令;Twist是指的线速度和角速度
        self.arming_client_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_FCU_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        
    def pose_callback(self, msg):
        self.cur_pose = msg #无人机当前位置
        print("current pose x y z:",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)

    def loop(self):
        rospy.init_node('follower')
        rate = rospy.Rate(100)
        self.targetPose.pose.position.x=0
        self.targetPose.pose.position.y=0
        self.targetPose.pose.position.z=1.3
        while not rospy.is_shutdown():
            if (self.arming_client_srv(True)):
			    print("Vehicle arming succeed!")
            if (self.set_FCU_mode_srv(custom_mode='OFFBOARD')):
                print("Vehicle offboard succeed!")
            else:   
			    print("Vehicle offboard failed!")

            self.pose_enu_pub.publish(self.targetPose)#发布东北天下的坐标系指令

            try:
                rate.sleep()
            except:
                continue

if __name__ == '__main__':
    follower = Follower()
    follower.loop()   