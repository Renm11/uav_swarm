#!/usr/bin/python
# -*- coding: UTF-8 -*-

#!/usr/bin/env python
import socket
import threading

#服务端
def Recv():
    address=('192.168.3.11',10000)
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(address)
    while 1:
        data,addr=s.recvfrom(2048)
        if not data:
            break
        print("got data from",addr[0],addr[1])
        print(data)
    s.close()

#客户端
# def Send():
#     addr=('127.0.0.1',1114)
#     s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
#     while 1:
#         data="raw_input"#()
#         if not data:
#             break
#         s.sendto(data,addr)
#     s.close()

if __name__ == '__main__':
    print('Successful connection')
    recvMessage = threading.Thread(target=Recv, args=())
    # sendMessage = threading.Thread(target=Send, args=())
    # sendMessage.start()
    recvMessage.start()
