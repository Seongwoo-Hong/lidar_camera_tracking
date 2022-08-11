#!/usr/bin/env python
import rospy
from std_msgs.msg import String,  Float64
from obstacle_detector.msg import Obstacles
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import cv2
import matplotlib.pyplot as plt
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import math

# pub = rospy.Publisher('tracker_output', String, queue_size=10)

def check_header(data):
    data = str(data)
    split_data = data.split()
    count_header = split_data.count('header:')
    if count_header == 1:
        header_index = split_data.index('header:')
        my_data = split_data[header_index:]
        return my_data
    elif count_header > 1:
        # header_index_list = [i for i, value in enumerate(mod) if value == 'header:']
        # first_header_index = header_index_list[0]
        # second_header_index = header_index_list[1]
        print('please header check....')

def time_check(data):
    if type(data) == list :
        sec_index = data.index('secs:')
        nsec_index = data.index('nsecs:')
        secs = float(data[sec_index + 1])
        decimal_place =len(data[nsec_index + 1])
        nsecs = float(data[nsec_index + 1])*10**((-1)* decimal_place)
        temp_time = secs + nsecs
        return temp_time
    else :
        print('input is not list')

def make_id(data, time):
    # time = time_check(data)
    obstacle_index_list = [i for i, value in enumerate(data) if value == 'center:']
    coordi_x = float(data[obstacle_index_list[0]+2])
    coordi_y = float(data[obstacle_index_list[0]+4])
    vel_x = float(data[obstacle_index_list[0]+9])
    vel_y = float(data[obstacle_index_list[0]+11])
    ped_coordi_x = coordi_x+vel_x
    ped_coordi_y = coordi_y+vel_y
    info_list = [coordi_x,coordi_y,ped_coordi_x,ped_coordi_y,time,1]
    unknown_class.append(info_list)
    
distance_offset = 0.01
unknown_class = []


def check_id_list(data):
    time = time_check(data)
    obstacle_index_list = [i for i, value in enumerate(data) if value == 'center:']
    for i in obstacle_index_list:
        count=0
        coordi_x = float(data[i+2])
        coordi_y = float(data[i+4])
        vel_x = float(data[i+9])
        vel_y = float(data[i+11])
        x_offset = [coordi_x - distance_offset, coordi_x + distance_offset]
        y_offset = [coordi_y - distance_offset, coordi_y + distance_offset]
        for j in range(len(unknown_class)):
            if unknown_class[j][2]> x_offset[0] and unknown_class[j][2]< x_offset[1] and unknown_class[j][3] > y_offset[0] and unknown_class[j][3] < y_offset[1]:
                unknown_class[j][0], unknown_class[j][1] = coordi_x, coordi_y
                unknown_class[j][2], unknown_class[j][3] = vel_x, vel_y
                unknown_class[j][4] = time
            else:
                count+=1
        if count != len(unknown_class)-1:
            make_id(data[i:i+12],time)

def check_obstacle_numer(data):
    if type(data) == list :
        obstacle_number = data.count('center:')
        return obstacle_number
    else :
        print('input is not list')

def update_id_threshhold_with_distance(data):
    
    for i in range(len(unknown_class)):
        unknown_class[i][-1] == 0
    time = time_check(data)
    obstacle_index_list = [i for i, value in enumerate(data) if value == 'center:']
    for j in range(len(unknown_class)):
        count = 0
        temporary_id = []
        temporary_offset = []
        for i in obstacle_index_list:
            coordi_x, coordi_y = float(data[i+2]), float(data[i+4])
            vel_x, vel_y = float(data[i+9]), float(data[i+11])
            x_offset, y_offset = abs(unknown_class[j][2] - coordi_x), abs(unknown_class[j][3] - coordi_y) ### unknown_class[j][2] = pred_x, unknown_class[j][3] = pred_y
        if x_offset< distance_offset and y_offset< distance_offset:
            count +=1
            temporary_id.append([i,coordi_x,coordi_y,vel_x,vel_y,time,1])
            temporary_offset.append(math.sqrt(math.pow(x_offset,2) + math.pow(y_offset,2)))
        if count == 1:
            unknown_class[j] = temporary_id[0][1:]
            obstacle_index_list.remove(temporary_id[0][0])
        elif count >= 2:
            good_target_index = np.argmin(temporary_offset)
            unknown_class[j] = temporary_id[good_target_index][1:]
            obstacle_index_list.remove(temporary_id[good_target_index][0])
    if len(obstacle_index_list) !=0:
        for i in obstacle_index_list:
            make_id(data[i:i+12],time)
    for i in range(len(unknown_class)):
        if unknown_class[i][-1] == 0:
            x_vel = unknown_class[i][2]-unknown_class[i][0]
            y_vel = unknown_class[i][3]-unknown_class[i][1]
            unknown_class[i][0], unknown_class[i][1] = unknown_class[i][2],unknown_class[i][3]
            unknown_class[i][2], unknown_class[i][3] = unknown_class[i][2]+x_vel, unknown_class[i][3]+y_vel
            unknown_class[i][-1] = 1
    

def callback(data):
    
    # While True : ##### for subscribe data record
        # print(type(data))
        # print('')
        # print('')
        # print('listener start')
        # test_data = str(data)
        # test_data = test_data.split()
        # print(type(test_data))
        # f = open("obstacle_info2.txt", 'a')
        # f.write('')
        # f.write('')
        # f.write('listener_start')
        # f.write('')
        # f.write(test_data)
        # f.write('')
        # f.close()
        # rospy.loginfo(
    pub = rospy.Publisher('tracker_output', String, queue_size=1000)
    input_data = check_header(data)
    obstacle_number = check_obstacle_numer(input_data)
    # check_id_list(input_data)
    update_id_threshhold_with_distance(input_data)
    pub.publish(str(unknown_class))
    # rate = rospy.Rate(10)
    
    # rate.sleep()
    
def listener():
    # global pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # pub = rospy.Publisher('tracker_output', String, queue_size=1000)
    rospy.init_node('listener_lidar', anonymous=True)

    rospy.Subscriber("/obstacles", Obstacles, callback)
    # pub.publish('output data')
    # spin() simply keeps python from exiting until this node is stopped
    # print(unknown_class)
    # pub.publish(str(unknown_class))
    rospy.spin()

if __name__ == '__main__':
    listener()