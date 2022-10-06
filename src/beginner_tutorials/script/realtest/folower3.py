#!/usr/bin/python
# -*- coding: UTF-8 -*-

#!/usr/bin/env python
import socket
import threading
import rospy
import numpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray

# @Description: 
# @Author: ren ming
# @Date: 2022-10-04
#服务端
class receive(threading.Thread):

    def __init__(self, name=None, uav_id=0):
        threading.Thread.__init__(self, name=name)
        self.id = uav_id
        self.pose = PoseStamped()
        self.cmd_vel_enu = TwistStamped()
        self.Kp = 1.0
        self.commanderCmd = "cmd"
        self.formation_new = None
        self.targetPose = Vector3()  #目标位置
        self.avoid_vel = Vector3()  #躲避速度
        self.leader_pose = Vector3()  #领导者位置
        self.Kp_avoid = 2.0
        self.vel_max = 1.0
        self.address = ('127.0.0.1', 1118)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=1)
        self.formation_pattern_sub = rospy.Subscriber("/lab/formation_pattern",
                                                      Float32MultiArray,
                                                      self.formation_callback,
                                                      queue_size=1)  #订阅者，编队形状数组
        self.vel_enu_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped,
                                           queue_size=1)  #发布东北天下的控制命令;Twist是指的线速度和角速度
        self.avoid_vel_sub = rospy.Subscriber("/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)  #躲避速度
        self.leader_pose_sub = rospy.Subscriber("mavros/local_position/pose",
                                                PoseStamped,
                                                self.leader_pose_callback,
                                                queue_size=1)  #领导者位置
        self.arming_client_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_FCU_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    def pose_callback(self, msg):
        self.pose = msg  #无人机当前位置
        print("current pose x y z:", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def formation_callback(self, msg):
        self.formation_new = numpy.array(msg.data).reshape(2, 3)
        self.targetPose = self.formation_new[self.id]

    #躲避
    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    #领导者位置
    def leader_pose_callback(self, msg):
        self.leader_pose = msg

    #每次查询是否有命令
    def commander(self):
        while 1:
            data, addr = self.server.recvfrom(2048)
            if not data:
                break
            self.commanderCmd = data
            break

    def setLandMode(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            isLanding = landService(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        except rospy.ServiceException, e:
            print("service land call failed: %s. The vehicle cannot land ", e)

    def Recv(self):
        print("recv")
        self.server.bind(self.address)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        while 1:
            self.commander()
            if self.commanderCmd == "takeoff":
                if (self.arming_client_srv(True)):
                    print("Vehicle arming succeed!")
                if (self.set_FCU_mode_srv(custom_mode='OFFBOARD')):
                    print("Vehicle offboard succeed!")
                else:
                    print("Vehicle offboard failed!")
                self.cmd_vel_enu.twist.linear.x = self.Kp * (0 - self.pose.pose.position.x
                                                             )  #直接用线速度不知道是否可以 答：可以，下面加上了判断是否超过最大线速度
                self.cmd_vel_enu.twist.linear.y = self.Kp * (0 - self.pose.pose.position.y)
                self.cmd_vel_enu.twist.linear.z = self.Kp * (1.3 - self.pose.pose.position.z)
                self.vel_enu_pub.publish(self.cmd_vel_enu)  #发布东北天下的坐标系指令
                # print("takeoff success")
                # self.server.sendto("takeoffSuccess", client_addr)
            elif self.commanderCmd == "land":
                self.setLandMode()
                # print("land success")
                # self.server.sendto("landSuccess", client_addr)
            # elif self.commanderCmd=="startFomation":
            #     print("startFomation success")
            #     server.sendto("startFomationSuccess",client_addr)
            elif self.commanderCmd == "line":
                #获取line目标位置，用pid控制算法给每家无人机发送位置
                if (self.arming_client_srv(True)):
                    print("Vehicle arming succeed!")
                if (self.set_FCU_mode_srv(custom_mode='OFFBOARD')):
                    print("Vehicle offboard succeed!")
                else:
                    print("Vehicle offboard failed!")
                self.cmd_vel_enu.twist.linear.x = self.Kp * (
                    self.leader_pose + self.targetPose[0] - self.pose.pose.position.x
                )  #直接用线速度不知道是否可以 答：可以，下面加上了判断是否超过最大线速度
                self.cmd_vel_enu.twist.linear.y = self.Kp * (self.leader_pose + self.targetPose[1] -
                                                             self.pose.pose.position.y)
                self.cmd_vel_enu.twist.linear.z = self.Kp * (self.leader_pose + self.targetPose[2] -
                                                             self.pose.pose.position.z)

                self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.Kp_avoid * self.avoid_vel.x  #在这里加上了躲避线速度
                self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.Kp_avoid * self.avoid_vel.y
                self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.Kp_avoid * self.avoid_vel.z
                cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 +
                                     self.cmd_vel_enu.linear.z**2)**0.5  #线速度；x**y表示x的y次方
                if cmd_vel_magnitude > 3**0.5 * self.vel_max:  #线速度是否大于最大线速度的根号3倍
                    self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
                    self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
                    self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max
                self.vel_enu_pub.publish(self.cmd_vel_enu)  #发布东北天下的坐标系指令
                print("line success")
                # self.server.sendto("lineSuccess", client_addr)
            elif self.commanderCmd == "triangle":
                #获取triangle对应位置，用pid控制算法给每家无人机发送位置
                print("triangle success")
                # self.server.sendto("triangleSuccess", client_addr)

    def run(self):
        # while 1:
        self.Recv()


if __name__ == '__main__':
    print('Successful connection')
    s = send(name="send")
    r = receive(name="receive")
    s.start()
    r.start()
