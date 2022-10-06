#!/usr/bin/python
# -*- coding: UTF-8 -*-
### This code is about the distributed formation control with a consensus protocol and the task allocation by KM algorithm
### For more details, please see the paper on https://arxiv.org/abs/2005.01125

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
import sys
import numpy
from scipy.optimize import linear_sum_assignment
from mavros_msgs.msg import PositionTarget

if sys.argv[2] == '6':#formation中
    from formation_dict import formation_dict_6 as formation_dict
elif sys.argv[2] == '9':
    from formation_dict import formation_dict_9 as formation_dict
elif sys.argv[2] == '18':
    from formation_dict import formation_dict_18 as formation_dict
elif sys.argv[2]=='2':
    from formation_dict import formation_dict_3 as formation_dict
else:
    print("Only 3, 6, 9 and 18 UAVs are supported.")

#主要干的事情：当编队形状发生改变的时候，通过匈牙利算法算出每个无人机前往的位置，然后发布出去，同时自己也要避免被撞到
# 领导者订阅了：
#     1.本机位置信息
#     2.本机前左上（右手系）控制信息，控制信息由键盘控制
#     3.防撞速度指令（由于是leader，所以不需要防撞指令）
#     4.领导者编队模型命令
# 发布了：
#     1.领导者位置
#     2.编队形状
#     3.沟通拓扑结构
#     4.东北天控制命令
#     5.领导者命令

# 除去回调函数还有：
#      1.build_graph(self, orig_formation, change_formation)
#      2.find_path(self, i)
#      3.KM(self)
#      4.get_new_formation(self, changed_id, change_formation)
#      5.get_communication_topology(self, rel_posi)
#      6.loop(self): 主要循环函数

# 测试版流程：
#     启动无人机
#     自动起飞
#     开始编队，初始的时候发布一个默认的编队
#     切换编队

# 启动起飞后，在leader机器上运行：
#     将leader设置为0号机
#     发布本身位置
#     发布初始编队拓扑结构
#     监听指令
#         if 改变队形
#             获取新的沟通拓扑结构
#         else 
#             发布原本的沟通拓扑结构

# 启动起飞后，follower机器上运行
#     初始化本机编号
#     发布本机里程计（用于避障等）
#     获取编队中自己需要到达的目标点
#     监听其他机器
#         if 快要撞了
#             避障
#         else 
#             发布命令走向目标点

class Leader:

    def __init__(self, uav_type, leader_id, uav_num):
        self.id = leader_id
        self.pose = PoseStamped()
        self.cmd_vel_enu = Twist() #控制命令
        self.uav_num = uav_num
        self.avoid_vel = Vector3(0,0,0)
        self.formation_config = 'waiting'
        self.origin_formation = formation_dict["origin"]#返回的是一个多维数组
        self.new_formation = self.origin_formation #初始编队模型
        # self.adj_matrix = None
        self.communication_topology = None #拓扑结构,设置为一维数组，每三个一组，每架无人机执行获取对应的目标
        self.formation_new = numpy.zeros((uav_num, 3))
        self.list = []          ## 空列表
        
        # self.changed_id = numpy.arange(0, self.uav_num - 1)
        # self.target_height_recorded = False
        # self.cmd = String()
        # self.f = 200 #频率
        # self.Kz = 0.5
        self.target_pose=PositionTarget()#

        self.pose_sub = rospy.Subscriber(uav_type+'_'+str(self.id)+"/mavros/local_position/pose", PoseStamped , self.pose_callback, queue_size=1)#订阅者，本机位姿
        # self.cmd_vel_sub = rospy.Subscriber("/xtdrone/leader/cmd_vel_flu", Twist, self.cmd_vel_callback, queue_size=1)#控制命令有啥用
        # self.avoid_vel_sub = rospy.Subscriber("/xtdrone/"+uav_type+'_'+str(self.id)+"/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)#防撞机制命令
        self.leader_cmd_sub = rospy.Subscriber("/xtdrone/leader/cmd",String, self.cmd_callback, queue_size=1)#订阅命令，领导者编队模型命令,值为各种类型的形状

        self.pose_pub = rospy.Publisher("/xtdrone/leader/pose", PoseStamped , queue_size=1) #发布者，领导者位姿数据
        self.formation_pattern_pub = rospy.Publisher('/xtdrone/formation_pattern', Float32MultiArray, queue_size=1)#发布者，编队形状数组
        # self.communication_topology_pub = rospy.Publisher('/xtdrone/communication_topology', Int32MultiArray, queue_size=1)#沟通拓扑结构，各个无人机该选哪个位置？？？
        self.vel_enu_pub =  rospy.Publisher('/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd_vel_flu', Twist, queue_size=1)#东北天控制命令，使无人机一直处于这个位置
        # self.cmd_pub = rospy.Publisher('/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd', String, queue_size=1)#领导者命令

    def pose_callback(self, msg):
        self.pose = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_enu = msg #将当前的参数值直接赋值，机器就直接停在那了

    def cmd_callback(self, msg): #采用哪一种编队形式
        self.cmd=msg.data
        # old="origin"#使用数组
        a=len(list)
        if(list[a-1]!=self.cmd):
            get_plane_to_taregetpose(list[a-1],self.cmd,self_num)#获取每个无人机需要执行的新目标
            self.list.append(self.cmd) 
        

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg
    #下面的函数是用于变换队形的时候用于求解每个无人机应该往哪一个位置去，最后会得到一种最优的路径
    def get_plane_to_taregetpose(self,old_formation,new_fomation_type,uav_num)#传入  老数组，新数组，总数量
        targetpose=numpy.zeros((uav_num, uav_num))
        for i range(0,uav_num): #构造变换矩阵
            for j in range(0,uav_num):
                targetpose[i][j]=numpy.linalg.norm(origin_formation[i]-new_fomation_type[j])
        row_ind,col_ind=row_ind, col_ind = linear_sum_assignment(targetpose)
        for i range range(0,uav_num)
            a=col_ind[i]
            self.formation_new[i]=new_fomation_type[a]
        return formation_new #得到结果，返回每个无人机应该往哪一个无人机执行，这是一个数组

    def loop(self):
        rospy.init_node('leader')
        rate = rospy.Rate(self.f)
        while True:
            # self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.avoid_vel.x
            # self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.avoid_vel.y
            # self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.avoid_vel.z
            self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x #东北天坐标系
            self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y
            self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z
            formation_pattern = Float32MultiArray()#赋初值，这个类型的data数据是一个一维数组
            # formation_pattern.data = self.new_formation.flatten().tolist()#flatten():将数组或者矩阵压平（压成一维数组）；tolist()转化为列表
            if(cmd!="orgin"):
                
            else
                formation_pattern.data = self.new_formation.flatten().tolist()#flatten():将数组或者矩阵压平（压成一维数组）；tolist()转化为列表


            # self.formation_pattern_pub.publish(formation_pattern)#发布编队造型；初始的时候有一个初始的编队
            # if(not self.communication_topology is None):#拓扑结构非空，开始时候不执行
            #     communication_topology = Int32MultiArray()
            #     communication_topology.data = self.communication_topology.flatten().tolist()
            #     self.communication_topology_pub.publish(communication_topology)
            self.vel_enu_pub.publish(self.cmd_vel_enu)#发布东北天下的速度，以这个机器为中心，这会使这个机器停在这里
            self.pose_pub.publish(self.pose)#发布领导者位置
            # self.cmd_pub.publish(self.cmd)#发布领导者控制命令
            try:
                rate.sleep()
            except:
                continue

if __name__ == '__main__':
    leader = Leader(sys.argv[1], 0, int(sys.argv[2]))
    leader.loop()   
