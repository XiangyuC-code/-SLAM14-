import cv2 
import numpy as np
import open3d as o3d
import os
os.chdir(os.path.dirname(__file__))




def main():
    colorImgs, depthImgs = [], []
    poses = []

    with open("./pose.txt", 'r') as f:
        data = f.readlines()
    f.close()

    # c++和python读取同一个pgm文件得到的值不同，试了很多种读取方法都没法保持一致，因此将c++读取到的值保存到depth.txt里面，python直接读取该文件的内容
    with open("./depth.txt", "r") as w:
        deps = w.readlines()
    depthImgs = []

    for i in range(5):
        colorImgs.append(cv2.imread("./color/{}.png".format(i+1)))
        depthImg = deps[i*480:(i+1)*480]
        tmp = []
        for line in depthImg:
            line = line.split(" ")[:-1]
            tmp.append(np.array(line, dtype=np.uint16))
        depthImgs.append(tmp)

        d = data[i].split(' ')
        d[-1] = d[-1][:-1]
        poses.append(np.array(d, dtype=np.float32))
    
    cx = 325.5 
    cy = 253.5 
    fx = 518.0
    fy = 519.0
    depthScale = 1000.0

    pointcloud = []

    for i in range(5):
        print("转换图像中: %d" % (i+1))
        color = colorImgs[i]
        depth = depthImgs[i]
        T = pose2T(poses[i])
        for v in range(color.shape[0]):
            for u in range(color.shape[1]):
                d = depth[v][u]
                if d==0: continue

                point = np.ones([4, 1])
                point[2] = d/depthScale
                point[0] = (u - cx) * point[2] / fx
                point[1] = (v - cy) * point[2] / fy
                pointWorld = np.matmul(T, point)

                p = [0] * 6 
                p[:3] = pointWorld[:3]
                p[5] = color[v, u, 0] / 255.0
                p[4] = color[v, u, 1] / 255.0
                p[3] = color[v, u, 2] / 255.0

                pointcloud.append(p)

    print("点云共有%d个点" % (len(pointcloud)))
    showPointCloud(pointcloud)

def pose2T(pose):
    T = np.zeros([4, 4])
    q1 = pose[6]
    q2 = pose[3]
    q3 = pose[4]
    q4 = pose[5]

    T[:3, :3] = [[2*(q1**2+q2**2)-1, 2*(q2*q3 - q1*q4), 2*(q2*q4 + q1*q3)], 
                 [2*(q2*q3 + q1*q4), 2*(q1**2+q3**2)-1, 2*(q3*q4 - q1*q2)], 
                 [2*(q2*q4 - q1*q3), 2*(q3*q4 + q1*q2), 2*(q1**2+q4**2)-1]] 
    T[:3, 3] = [pose[0], pose[1], pose[2]]
    T[3, 3] = 1
    return T

def showPointCloud(pointcloud):
    #展示出来的点云图需要手动调整视角
    #源代码中使用了pangolin::OpenGlRenderState调整了相机的位置
    if len(pointcloud) == 0:
        print("Point cloud is empty!")
        return

    pointcloud = np.array(pointcloud)
    color = pointcloud[:, 3:]

    pointcloud = pointcloud[:, :3] 

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)
    pcd.colors = o3d.utility.Vector3dVector(color)
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()