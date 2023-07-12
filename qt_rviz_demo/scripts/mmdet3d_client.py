#!/usr/bin/env python2
# -*- coding:utf-8 -*-
import requests
import numpy as np
import glob
from os import path as osp
from os import makedirs

def detection(point_cloud, score_thr, out_dir, file_name):
# 发送POST请求到服务端
    url = 'http://10.236.11.118:5000/predict'
    data = {'point_cloud': point_cloud, 'score_thr': score_thr}
    response = requests.post(url, json=data)
    if response.status_code == 200:
        result = response.json()

        # scores = np.array(result['scores'])
        # boxes = np.array(result['boxes'])
        # labels = np.array(result['pred_classes'])
        # points = np.array(result['points'])

        points_result = np.array(result['points_result'], dtype=np.float32)
        remains = points_result[..., 3:]
        matrix = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        points_result = np.matmul(points_result[..., :3], matrix)
        points_result = np.concatenate((points_result[..., :3], remains), axis=1)
        # 输出服务端的预测结果
        print(points_result.shape)
        points_result.tofile(out_dir + '/' + file_name + '.bin')

    else:
        print("请求失败")
        print("状态码：", response.status_code)


if __name__ == '__main__':
    pcd_dir = './pointcloud/*.bin'
    pcd_files = glob.glob(pcd_dir)
    out_dir = './result'
    # 设置置信度，将置信度小于阈值的目标不视为障碍物
    score_thr = [0.7]
    result_path = out_dir
    if not osp.exists(result_path):
        makedirs(result_path)
    for pcd in pcd_files:
        file_name = osp.split(pcd)[-1].split('.')[0]

        # 创建保存文件的路径
        #result_path = osp.join(out_dir, file_name)
        # if not osp.exists(result_path):
        #     makedirs(result_path)

        # 从bin文件中读取点云数据
        point_cloud = np.fromfile(pcd, dtype=np.float32).reshape(-1, 4).tolist()
        detection(point_cloud, score_thr, result_path, file_name)

    print('检测完成')

