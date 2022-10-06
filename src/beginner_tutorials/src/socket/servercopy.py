#!/usr/bin/python
# -*- coding: UTF-8 -*-

#!/usr/bin/env python
import socket
import threading
import numpy
import rospy
from geometry_msgs.msg import Vector3, PoseStamped
import time
import numpy 
import sys

vehicle_num=3
pose = [Vector3()] * vehicle_num
vehicles_avoid_control = [Vector3()] * vehicle_num #多维数组

def ListtoArray(list_a):
    e = list_a.split(",")
    arr = list(map(int, e))
    f = numpy.array(arr)
    return f


#服务端,接收各个飞机的位置数据
def Recv():
    address = ('127.0.0.1', 1114)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # s.allow_reuse_address = True
    s.bind(address)
    while 1:
        data, addr = s.recvfrom(2048)
        if not data:
            break
        if addr[0] == '192.168.3.28':  #"0号ip"
            pose[0] = ListtoArray(data)
        elif addr[0] == '192.168.3.30':  #"1号ip":
            pose[1] = ListtoArray(data)
        elif addr[0] == '192.168.3.29':  # "2号ip":
            pose[2] = ListtoArray(data)
        print "got data from", addr[0], addr[1]
        print(data)
    s.close()
def avoid():
    rospy.init_node('avoid')#初始化一个节点
    time.sleep(1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        for i in range(vehicle_num):#对于每一架无人机
            position1 = pose[i]#numpy.array([pose[i].pose.position.x, pose[i].pose.position.y, pose[i].pose.position.z]) #每个无人机的位姿数据
            for j in range(1, vehicle_num-i):#遍历是否与其他无人机发生碰撞，j从1到无人机总数减1
                position2 = pose[i+j]#numpy.array([pose[i+j].pose.position.x, pose[i+j].pose.position.y, pose[i+j].pose.position.z])#这里会数组越界啊 ？？？答：并不会，因为是vehicle_num-i并不是-1  ！！！
                dir_vec = position1-position2 #减出来是三维数组，xyz轴的差值
                k = 1 - numpy.linalg.norm(dir_vec) / avoid_radius #numpy.linalg.norm 表示求范数；k表示在控制圆的多少
                if k > 0:#这时候说明已经在危险区域了
                    cos1 = dir_vec.dot(aid_vec1)/(numpy.linalg.norm(dir_vec) * numpy.linalg.norm(aid_vec1))#dot表示点积；距离的x/半径值*1
                    cos2 = dir_vec.dot(aid_vec2)/(numpy.linalg.norm(dir_vec) * numpy.linalg.norm(aid_vec2))#            距离的y/半径值*1
                    if  abs(cos1) < abs(cos2):#哪边更接近
                        avoid_control = k * numpy.cross(dir_vec, aid_vec1)/numpy.linalg.norm(numpy.cross(dir_vec, aid_vec1))#k*直线距离和aid的叉乘除以对应的范数，相当于归一化了；cross,返回两个向量的叉积,还是一个向量；向量的大小为这两个向量组成的平行四边形面积，夹角也可以算出来
                    else:
                        avoid_control = k * numpy.cross(dir_vec, aid_vec2)/numpy.linalg.norm(numpy.cross(dir_vec, aid_vec2))

                    vehicles_avoid_control[i] = Vector3(vehicles_avoid_control[i].x+avoid_control[0],vehicles_avoid_control[i].y+avoid_control[1],vehicles_avoid_control[i].z+avoid_control[2])#两个相近无人机其中一个朝正向
                    vehicles_avoid_control[i+j] = Vector3(vehicles_avoid_control[i+j].x-avoid_control[0],vehicles_avoid_control[i+j].y-avoid_control[1],vehicles_avoid_control[i+j].z-avoid_control[2])#两个相近无人机另一个朝反向

        # for i in range(vehicle_num):
        #     avoid_control_pub[i].publish(vehicles_avoid_control[i])#躲避命令发布
        vehicles_avoid_control = [Vector3()] * vehicle_num#躲避命令控制数组清空
        rate.sleep()

#客户端
def Send():
    addr = ('127.0.0.1', 11112)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    PORT = 1060
    network = '<broadcast>'
    while 1:
        data =str(vehicles_avoid_control.flatten().tolist())  #是一个一维数组字符串
        if not data:
            break
        s.sendto(data.encode('utf-8'), (network, PORT))
    s.close()


if __name__ == '__main__':
    print('Successful connection')
    recvMessage = threading.Thread(target=Recv, args=())
    sendMessage = threading.Thread(target=Send, args=())
    sendMessage1 = threading.Thread(target=avoid, args=())
    sendMessage.start()
    sendMessage1.start()
    recvMessage.start()
