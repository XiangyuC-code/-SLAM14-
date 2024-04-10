import cv2
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import os
os.chdir(os.path.dirname(__file__))

#文件路径
left_file = "./left.png"
right_file = "./right.png"

def main():
    #内参
    fx = 718.856
    fy = 718.856
    cx = 607.1928
    cy = 185.2157
    #基线
    b = 0.573

    left = cv2.imread(left_file)
    right = cv2.imread(right_file)

    sgbm = cv2.StereoSGBM_create(0, 96, 9, 8*9*9, 32*9*9, 1, 63, 10, 100, 32)
    disparity_sgbm = sgbm.compute(left, right)
    disparity = np.float32(disparity_sgbm) / 16.0

    pointcloud = []

    for v in range(left.shape[0]):
        for u in range(left.shape[1]):
            if disparity[v, u] <= 0 or disparity[v, u] >= 96: continue

            point = [0, 0, 0, left[v, u, 0]/255.0]

            x = (u - cx) / fx
            y = (v - cy) / fy
            depth = fx * b / disparity[v, u]
            point[0] = x * depth
            point[1] = y * depth
            point[2] = depth

            pointcloud.append(point)

    cv2.imshow("disparity", disparity/96.0)
    cv2.waitKey(0)
    showPointCloud(pointcloud)

def showPointCloud(pointcloud):
    if len(pointcloud) == 0:
        print("Point cloud is empty!")
        return

    pointcloud = np.array(pointcloud)
    color = np.expand_dims(pointcloud[:, 3], axis=1)
    color = np.tile(color, 3)

    pointcloud = pointcloud[:, :3] 

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)
    pcd.colors = o3d.utility.Vector3dVector(color)
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()
