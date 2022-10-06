#!/usr/bin/python
# -*- coding: UTF-8 -*-

# @Description:
#   这两线程都是服务端
#   1.从主机获取队形，并在本机发布在话题 /formation_pattern
#   2.从主机获取躲避服务，并发布在话题 /avoid_vel
# @Author: ren ming
# @Date: 2022-10-03


import socket
import string
import threading
import socket
from beginner_tutorials.src.socket.testSringAndArray import ListtoArray
import rospy
import numpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
# @Description: #从主机获取数据，发布目标位置
# @Author: ren ming
# @Date: 2022-10-03


class fomation():

    def __init__(self, uavId=0, uavNum=3):
        threading.Thread.__init__(self, name="Myname")
        self.id = uavId
        self.Num = uavNum
        self.targetPose = Vector3()
        self.dataFomation = numpy.array()
        self.Float32MultiArray
        self.formation_pattern_pub = rospy.Publisher('/formation_pattern', Vector3, queue_size=1)

    # 将数字字符串转换为array

    def ListtoArray(self, list_a):
        e = list_a.split(",")
        arr = list(map(int, e))
        f = numpy.array(arr)
        return f

    def Recv(self):
        address = ('127.0.0.1', 9000)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(address)
        while 1:
            data, addr = s.recvfrom(2048)
            if not data:
                break
            self.dataFomation = ListtoArray(data)
            self.targetPose = numpy.array(data).reshape(self.Num, 3)[self.id]  # 后一个数字指的是三列
            self.formation_pattern_pub.publish(self.targetPose)
            print("got data from ", addr)
            print(data)
        s.close()

    def run(self):
        self.Recv()


# @Description: 从主机获取数据，发布躲避速度
#               这其中只有端口不一样
# @Author: ren ming
# @Date: 2022-10-03


class avoid():

    def __init__(self, uavId=0, uavNum=3):
        threading.Thread.__init__(self, name="Myname")
        self.id = uavId
        self.Num = uavNum
        self.targetPose = Vector3()
        self.dataFomation = numpy.array()
        self.Float32MultiArray
        self.fomation_avoid_pub = rospy.Publisher('/fomation_avoid', Vector3, queue_size=1)

    # 将数字字符串转换为array

    def ListtoArray(self, list_a):
        e = list_a.split(",")
        arr = list(map(int, e))
        f = numpy.array(arr)
        return f

    def Recv(self):
        address = ('127.0.0.1', 9500)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(address)
        while 1:
            data, addr = s.recvfrom(2048)
            if not data:
                break
            self.dataFomation = ListtoArray(data)
            self.targetPose = numpy.array(data).reshape(self.Num, 3)[self.id]  # 后一个数字指的是三列
            self.fomation_avoid_pub.publish(self.targetPose)
            print("got data from ", addr)
            print(data)
        s.close()

    def run(self):
        self.Recv()


if __name__ == '__main__':
    print('Successful connection')
    s = FAndAvoid(0, 3)
    FAndAvoid.Recv()
