#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, BoundingBox, BoundingBoxes
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import convertPointCloud2ToPointCloud


import numpy as np
import cv2
import time
import ros_numpy


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


from pylab import *

    # print(xyz_array.shape)
def convert_bbox(BBox):
    print(BBox)
    print(type(BBox))


# def convert_depth_image(ros_image):
#     cv_bridge = CvBridge()
#     try:
#         depth_image = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
#     except CvBridgeError, e:
#         print e
#     depth_array = np.array(depth_image, dtype=np.float32)
#     print('place_test_1')

#     print("depth_image : ", depth_image.shape)

    
#     print('place_test_2')
#     np.save("depth_img.txt", depth_image)

#     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
#     print("depth_colormap : ", depth_colormap.shape)
    

#     # print(depth_image.shape)

#     cv2.imwrite("depth_img.png", depth_colormap)


#     fig = plt.figure(figsize=(9, 6))
#     ax = fig.add_subplot(111, projection='3d')

    
      

#     x = np.linspace(0,1279,1280, dtype=int)
#     y = np.linspace(0,719,720, dtype=int)
#     xx, yy = np.meshgrid(x, y)
#     # print(type(depth_image))

#     # zz = depth_image.flatten('F')
#     zz = depth_image.flatten()

#     mycmap = plt.get_cmap('gist_earth')
#     surf = ax.plot_surface(xx, yy, depth_image, cmap=mycmap)

#     print("done.")
#     ax.set_zlim(0, 5000)


#     plt.show()


#     time.sleep(300)


def pixel2depth():
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", Image,callback=convert_bbox, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    pixel2depth()