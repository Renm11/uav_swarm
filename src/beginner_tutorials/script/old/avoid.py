import rospy
from geometry_msgs.msg import Vector3, PoseStamped
import time
import numpy 
import sys

#防撞策略
vehicle_type = sys.argv[1] #类型
vehicle_num = int(sys.argv[2])#数量
control_type = sys.argv[3]#控制模式
pose = [None]*vehicle_num#pose=[None]*3 ，pose的值为[None, None, None]
pose_sub = [None]*vehicle_num
avoid_control_pub = [None]*vehicle_num
avoid_radius = 1.5
aid_vec1 = [1, 0, 0]
aid_vec2 = [0, 1, 0]
vehicles_avoid_control = [Vector3()] * vehicle_num #多维数组

def pose_callback(msg, id):
    pose[id] = msg

rospy.init_node('avoid')#初始化一个节点
for i in range(vehicle_num):
    pose_sub[i] = rospy.Subscriber(vehicle_type+'_'+str(i)+'/mavros/local_position/pose',PoseStamped,pose_callback,i,queue_size=1)#订阅对应无人机的数据
    if control_type == "vel":
        avoid_control_pub[i] = rospy.Publisher("/xtdrone/"+vehicle_type+'_'+str(i)+"/avoid_vel", Vector3,queue_size=1)#发布避免碰撞的速度
    elif control_type == "accel":
        avoid_control_pub[i] = rospy.Publisher("/xtdrone/"+vehicle_type+'_'+str(i)+"/avoid_accel", Vector3,queue_size=1)#发布避免碰撞的加速度
    else:
        print("Only vel and accel are supported.")

time.sleep(1)
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    for i in range(vehicle_num):#对于每一架无人机
        position1 = numpy.array([pose[i].pose.position.x, pose[i].pose.position.y, pose[i].pose.position.z]) #每个无人机的位姿数据
        for j in range(1, vehicle_num-i):#遍历是否与其他无人机发生碰撞，j从1到无人机总数减1
            position2 = numpy.array([pose[i+j].pose.position.x, pose[i+j].pose.position.y, pose[i+j].pose.position.z])#这里会数组越界啊 ？？？答：并不会，因为是vehicle_num-i并不是-1  ！！！
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

    for i in range(vehicle_num):
        avoid_control_pub[i].publish(vehicles_avoid_control[i])#躲避命令发布
    vehicles_avoid_control = [Vector3()] * vehicle_num#躲避命令控制数组清空
    rate.sleep()
            
            
