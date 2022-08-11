#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import cv2
import matplotlib.pyplot as plt
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import math

def image_callback(ros_image):
    global xmin,xmax,ymin,ymax,human_angle, time_list,temp_time,coordi_x_list,coordi_y_list
    print('image xmin : %f, xmax : %f, ymin : %f, ymax : %f'%(xmin,xmax,ymin,ymax))
    if len(time_list)==2:
        cv_bridge = CvBridge()
        try:
            depth_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        except CvBridgeError:#, e:
            # print e
            print('error')
        print(type(ros_image))
        depth_array = np.array(depth_image, dtype=np.uint16)
        # depth_array = np.array(depth_image, dtype=np.float32)
        np.save("depth_img.txt", depth_image)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap = depth_image
        print("depth_colormap : ", depth_colormap.shape)
        # print(type(depth_colormap))
        xmin = int(xmin)
        xmax = int(xmax)
        ymin = int(ymin)
        ymax = int(ymax)
        crop_person = depth_colormap[ymin:ymax, xmin:xmax]
        print(crop_person.shape)
        flatten_crop_person = crop_person.flatten()
        # print('flatten')
        # np.where(flatten_test < 400, 3000, flatten_test)
        flatten_crop_person[flatten_crop_person<400] = 4000
        flatten_crop_person[flatten_crop_person>4000] = 4000
        # print(type(flatten_test))
        print(flatten_crop_person.shape)
        # print(np.histogram(flatten_crop_person, bins=350))
        print('human_angle : ', human_angle)
        print('human_distance : ',min(flatten_crop_person))
        # plt.hist(flatten_test) 
        # plt.title("histogram") 
        # plt.show()
        human_distance = min(flatten_crop_person)
        coord_x = int(human_distance * math.sin(human_angle))
        coord_y = int(human_distance * math.cos(human_angle))
        print('coordinate_x : ',coord_x)
        print('coordinate_y : ',coord_y)
        # coordi_x_list = []
        # coordi_y_list = []
        print('time_list : ',time_list)
        # if len(time_list) == 0:
        #     pass
        # elif len(time_list) ==1:
        #     coordi_x_list.append(coord_x)
        #     coordi_y_list.append(coord_y)
        #     temp_time = time_list[0]
        # if len(time_list) ==2:
        print('')
        print(time_list)
        print(temp_time)
        print('')
        if time_list[1] != temp_time:
            print('time_list : ', time_list)
            coordi_x_list.append(coord_x)
            coordi_y_list.append(coord_y)
            temp_time = time_list[1]
            vel_x = (coordi_x_list[0] - coordi_x_list[1])/(time_list[1]-time_list[0])
            vel_y = (coordi_y_list[0] - coordi_y_list[1])/(time_list[1]-time_list[0])
            if len(coordi_x_list)==3:
                del coordi_x_list[0]
                del coordi_y_list[0]
            vel_human = [vel_x, vel_y]
            print('vel_human : ',vel_human)
            temp_time=time_list[1]
        else:
            pass
    else :
        pass
    
    # cv2.imwrite("depth_img.png", depth_colormap)
    # fig = plt.figure(figsize=(9, 6))
    # ax = fig.add_subplot(111, projection='3d')
    # x = np.linspace(0,639,640, dtype=int)
    # y = np.linspace(0,479,480, dtype=int)
    # xx, yy = np.meshgrid(x, y)
    # zz = depth_image.flatten()
    # mycmap = plt.get_cmap('gist_earth')
    # surf = ax.plot_surface(xx, yy, depth_image, cmap=mycmap)
    # ax.set_zlim(0, 5000)
    # plt.show()
    print('')
    time.sleep(10)


def callback(data):
    print(type(data))
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print('here!!!!!!!')
    # print(data)
    data2 = str(data).split()
    # print(type(data2))
    # print(len(data2))
    # print(data2)
    if '"person"' in data2:
        end_index = data2.index('"person"')
        start_index = end_index - 13
        print('here is if roof')
        test_data = data2[start_index:end_index + 1]
        # print(test_data)
        xmin_index = test_data.index('xmin:')
        xmax_index = test_data.index('xmax:')
        ymin_index = test_data.index('ymin:')
        ymax_index = test_data.index('ymax:')
        global xmin 
        xmin = float(test_data[xmin_index + 1])
        global xmax 
        xmax = float(test_data[xmax_index + 1])
        global ymin 
        ymin = float(test_data[ymin_index + 1])
        global ymax 
        ymax = float(test_data[ymax_index + 1])
        # print(f'xmin : {xmin}, xmax : {xmax}, ymin : {ymin}, ymax : {ymax}')
        print('in fun xmin : %f, xmax : %f, ymin : %f, ymax : %f'%(xmin,xmax,ymin,ymax))
        center = (xmin+xmax)/2
        global human_angle
        human_angle = (center * 43.5 /320) -43.5
        global time_list
        if len(time_list)<=1:
            time_list.append(time.time())
        elif len(time_list)==2:
            time_list.append(time.time())
            print('afdhadfhad',len(time_list))
            del time_list[0]
        elif len(time_list)==3:
            del time_list[0]
        print('original time_list : ',time_list)
    # time.sleep(300)
    print('end!!!')
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, image_callback, queue_size=1)
    # rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, image_callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    global xmin ,xmax, ymin, ymax, time_list,coordi_x_list,coordi_y_list
    time_list = []
    coordi_x_list = []
    coordi_y_list = []
    temp_time = 0
    listener()
    print('in main xmin : %f, xmax : %f, ymin : %f, ymax : %f'%(xmin,xmax,ymin,ymax))