#!/usr/bin/env python2
# -*- coding:utf-8 -*-

import os
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2

DATA_PATH = "/home/mith/桌面/Desktop/数据集/kitti/"


if  __name__ == "__main__":
    frame = 0
    rospy.init_node('kitti_node',anonymous=True)
    #cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #img = cv2.imread(os.path.join(DATA_PATH, 'image_02/data/%06d.png'%frame))
        #cam_pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))

        point_cloud = np.fromfile(os.path.join(DATA_PATH, 'aaa.bin'),dtype=np.float32).reshape(-1,4)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:,:3]))
        rospy.loginfo("kitti published")
        print(frame)
        rate.sleep()
        frame += 1
        frame %= 1
