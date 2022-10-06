# import rospy
# from std_msgs.msg import String

# formation='1'

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
# def leader():
#     rospy.Subscriber("chatter", String, callback)

#     target_pose_pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         target_pose_pub.publish(hello_str)
#         rospy.spin()
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         leader()
#     except rospy.ROSInterruptException:
#         pass

# import rospy
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist, Vector3, PoseStamped
# import numpy as np
# from mavros_msgs.msg import PositionTarget
# leader_odom=PoseStamped()
# formation_dict_2={"origin":np.array([[4,4,1]])}
# formation_dict_6 = {"origin":np.array([[3,0,0],[0,3,0],[3,3,0],[0,6,0],[3,6,0]]),"T":np.array([[4,0,0],[2,0,0],[2,0,-2],[2,0,-4],[2,0,-6]]) , "diamond": np.array([[2,2,-2],[2,-2,-2],[-2,-2,-2],[-2,2,-2],[0,0,-4]]), "triangle": np.array([[-3,0,-3],[3,0,-3],[-1.5,0,-1.5],[1.5,0,-1.5],[0,0,-3]])}
# setpoint_raw_local_pub = rospy.Publisher("/mavros/local_position/pose", PositionTarget,queue_size=1)
# def leader_callback(msg):
#     leader_odom=msg
# leader_odom = rospy.Subscriber("leader_odom",PoseStamped , leader_callback, queue_size=1)


# def target(pos_setpoint,yaw_sp,setpoint_raw_local_pub):
#     pos_setpoint.type_mask = 0b100111111000;  # 100 111 111 000  xyz + yaw
#     pos_setpoint.coordinate_frame = 1
#     pos_setpoint.position.x = pos_sp[0]
#     pos_setpoint.position.y = pos_sp[1]
#     pos_setpoint.position.z = pos_sp[2]
#     pos_setpoint.yaw = yaw_sp
#     setpoint_raw_local_pub.publish(pos_setpoint)
def talker():
    pub = rospy.Publisher('targetformation_type', String, queue_size=10)
    rospy.init_node('keyboard', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       hello_str = "hello world %s" % rospy.get_time()
       rospy.loginfo(hello_str)
       pub.publish(hello_str)
       rate.sleep()
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
