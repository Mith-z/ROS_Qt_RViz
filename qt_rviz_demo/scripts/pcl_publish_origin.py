#!/usr/bin/env python2
# -*- coding:utf-8 -*-

import os
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
import glob
from std_msgs.msg import Header
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2

#DATA_PATH = "/home/mith/catkin_qt/src/qt_rviz_demo/scripts/result"
DATA_PATH = "./result"

if  __name__ == "__main__":
    frame = 0
    rospy.init_node('kitti_node',anonymous=True)
    #cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud_origin', PointCloud2, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        #img = cv2.imread(os.path.join(DATA_PATH, 'image_02/data/%06d.png'%frame))
        #cam_pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))
        pcd_dir = './pointcloud/*.bin'
        pcd_files = glob.glob(pcd_dir)

        # 从bin文件中读取点云数据
        #point_cloud = np.fromfile(os.path.join(DATA_PATH, '000001.bin'),dtype=np.float32).reshape(-1,4)
        point_cloud = np.fromfile(pcd_files[frame],dtype=np.float32).reshape(-1,4)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:,:3]))
        rospy.loginfo("kitti published")
        print(frame)
        rate.sleep()
        frame += 1
        if frame == len(pcd_files):
            frame = 0;
