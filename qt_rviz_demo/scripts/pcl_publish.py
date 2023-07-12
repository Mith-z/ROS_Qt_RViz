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
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2

#DATA_PATH = "/home/mith/catkin_qt/src/qt_rviz_demo/scripts/result"
DATA_PATH = "./result"

def func():
    frame = 0
    rospy.init_node('kitti_node',anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        pcd_dir = '/home/mith/catkin_qt/src/qt_rviz_demo/scripts/result/*.bin'
        pcd_files = glob.glob(pcd_dir)

        img_dir =  '/home/mith/catkin_qt/src/qt_rviz_demo/scripts/result/*.png'
        img_files = glob.glob(img_dir)

        # 从bin文件中读取点云数据
        #point_cloud = np.fromfile(os.path.join(DATA_PATH, '000001.bin'),dtype=np.float32).reshape(-1,4)
        point_cloud = np.fromfile(pcd_files[frame],dtype=np.float64).reshape(-1,4)

#        img = cv2.imread(img_files[frame])
#        cam_pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))

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


if  __name__ == "__main__":
    frame = 0
    rospy.init_node('kitti_node',anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        pcd_dir = './result/*.bin'
        pcd_files = glob.glob(pcd_dir)

        img_dir =  './image/*.png'
        img_files = glob.glob(img_dir)
#        pcd_dir = '/home/mith/catkin_qt/src/qt_rviz_demo/scripts/result/*.bin'
#        pcd_files = glob.glob(pcd_dir)

#        img_dir =  '/home/mith/catkin_qt/src/qt_rviz_demo/scripts/image/*.png'
#        img_files = glob.glob(img_dir)

        # 从bin文件中读取点云数据
        #point_cloud = np.fromfile(os.path.join(DATA_PATH, '000001.bin'),dtype=np.float32).reshape(-1,4)
        point_cloud = np.fromfile(pcd_files[frame],dtype=np.float64).reshape(-1,4)

        img = cv2.imread(img_files[frame])
        cam_pub.publish(bridge.cv2_to_imgmsg(img,"bgr8"))

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
