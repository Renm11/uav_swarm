# !/usr/bin/python
# -*- coding: UTF-8 -*-

import numpy 

#array to string
array_a=numpy.array([1,2213,3,1234,5,6])
print(array_a)
b=array_a.tolist()#array to list
# print(b)
# d=[str(x) for x in b] #number list to string list
# c=','.join(d) #list to str
# print(c)
# print(type(c))


def listToStr(list_a):
    d=[str(x) for x in list_a] #number list to string list
    c=','.join(d) #list to str
    return c
print(listToStr(b))

# import numpy

# x="0.278317540884,-1.25857508183,-0.802574515343"
# def ListtoArray(str_a):
#     e = str_a.split(",")
#     arr = list(map(float, e))
#     f = numpy.array(arr)
#     return f
# pose = numpy.zeros((2,3))
# pose[0]=ListtoArray(x)
# print(pose)


# print(ListtoArray(x))
# print(type(ListtoArray(x)))

# a="1.23,1.312,12.6"
# e = a.split(",")
# print(e)
# arr = list(map(float, e))
# print(arr[1])
# f = numpy.array(arr)

# print(f[1])
# pose = numpy.zeros((4,3))
# print(pose)
# pose[1]=f
# print(pose)









# # import imp
# # import string
# # from turtle import st
# # import numpy

# # # array to list

# #将一个数组转换成一个字符串
# def arraytolist(array_a):
#     b = array_a.tolist()
#     d = [str(x) for x in b]
#     c = ','.join(d)
#     return c


# # # list to array


# # #数字字符串转array,传入一个str3 = '123,123,1323'
# import numpy
# def ListtoArray(list_a):
#     e = list_a.split(",")
#     arr = list(map(float, e))
#     f = numpy.array(arr)
#     return f
# str3 = str([123,123,1323])
# print(type(str3))
# print(str3)
# print(ListtoArray(str3))

# # array_a = numpy.array([1, 2213, 3, 1234, 5, 6])
# # print(array_a,type(array_a))
# # print(str(array_a),type(str(array_a)))
# # print(arraytolist(array_a))
# # print(ListtoArray(arraytolist(array_a)))

# # str3 = str([123,123,1323])
# # print(type(str3))
# # print(ListtoArray(str3))
# # result3 = str3.split(',')
# # arr = list(map(int, result3))
# # print(arr,type(arr))
# # print(result3,type(arr))

# # import string

# # x = string()
# # print(x)

# # import numpy
# # s=numpy.zeros((4,3)) #初始值
# # s[0][1]=3
# # s[0][0]=2
# # s[0][2]=4
# # print(s)#原始数组
# # f=str(s.flatten().tolist()) #数组
# # print(f)
# # e = list_a.split(",")
# # arr = list(map(int, e))
# # f = numpy.array(arr)



# # print(s.flatten())
# # print(s.flatten().tolist())
# # print(numpy.array(s.flatten().tolist()).reshape(4,3) )
# # 输出：
# # [[2. 3. 4.]
# #  [0. 0. 0.]
# #  [0. 0. 0.]
# #  [0. 0. 0.]]
# # [2. 3. 4. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
# # [2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# # [[2. 3. 4.]
# #  [0. 0. 0.]
# #  [0. 0. 0.]
# #  [0. 0. 0.]]