
import rospy
from mavros_msgs.srv import CommandBool,SetMode,CommandTOL

def setLandMode():
   rospy.wait_for_service('/iris_1/mavros/cmd/land')
   try:
       landService = rospy.ServiceProxy('/iris_1/mavros/cmd/land',CommandTOL)
       isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land "%e

def myLoop():
   x='1'
   while not rospy.is_shutdown():
        setLandMode()
       
if __name__ == '__main__':
   rospy.init_node('gapter_pilot_node', anonymous=True)
   #rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
   #velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
   myLoop()
   

