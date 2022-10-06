#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
import sys, select, tty, termios
from std_msgs.msg import String
e = """
usage:
    t:takeof
    l:land
    line:line formation
    tri:triangle formation
"""

if __name__ == '__main__':
    rospy.init_node('keyboard')
    pub = rospy.Publisher('keys',String,queue_size = 1)
    cmd=String()
    rate=rospy.Rate(10)
    old_attr = termios.tcgetattr(sys.stdin)     #备份终端属性
    tty.setcbreak(sys.stdin.fileno())     #设置属性
    print(e)
    # print('Please input keys, press Ctrl + C to quit')
    formation_configs = ['waiting', 'T', 'diamond', 'triangle']
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:     #设置超时参数为0，防止阻塞
            pub.publish(sys.stdin.read(1))
            cmd=sys.stdin.read(1)
            print(cmd)
        rate.sleep()     #使用sleep()函数消耗剩余时间
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)     #设置为原先的标准模式
