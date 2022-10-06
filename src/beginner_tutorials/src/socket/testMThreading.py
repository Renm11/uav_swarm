#!/usr/bin/python
# -*- coding: UTF-8 -*-

import threading
# import thread
import time
# import sys



class Test(object):
    def __init__(self):
        # threading.Thread.__init__(self)
        self._sName = "machao"

    def process(self):
        #args是关键字参数，需要加上名字，写成args=(self,)
        th1 = threading.Thread(target=Test.buildList, args=(self,))
        th2 = threading.Thread(target=Test.buildList2, args=(self,))
        th3 = threading.Thread(target=Test.buildList3, args=(self,))
        th1.start()
        th2.start()
        th3.start()
        # th1.join()

    def buildList(self):
        while True:
            self._sName=input("input:")
            print(self._sName)
            # time.sleep(3)
            # self._sName="one"
    def buildList2(self):
        while True:
            # print(self._sName)
            c=1
    def buildList3(self):
        while True:
            a=3
            # time.sleep(6)
            # self._sName="two"


test = Test()
test.process()

# #!/usr/bin/env python
# import socket
# import threading

# bianliang1 = "x"
# bianliang2 = "y"


# #服务端
# def Recv():
#     global bianliang1 
#     bianliang1 = "recv"
#     global bianliang2
#     bianliang2 = "recv2"


# #客户端
# def Send():
#     global bianliang1 
#     bianliang1 = "send"
#     global bianliang2
#     bianliang2 = "send2"
    
# def get():
#     print(bianliang1)
#     print(bianliang2)

# if __name__ == '__main__':
#     # print('Successful connection')
#     recvMessage = threading.Thread(target=Recv, args=())
#     sendMessage = threading.Thread(target=Send, args=())
#     sendMessage1 = threading.Thread(target=get, args=())
#     sendMessage.start()
#     sendMessage1.start()
#     recvMessage.start()
