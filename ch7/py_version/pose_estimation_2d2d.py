import cv2
import numpy as np
import os
os.chdir(os.path.dirname(__file__))

img_path1 = "1.png"
img_path2 = "2.png"

def main():
    # 读取图像
    img1 = cv2.imread(img_path1, cv2.IMREAD_COLOR)
    img2 = cv2.imread(img_path2, cv2.IMREAD_COLOR)
    if img1 is None or img2 is None:
        print("Can not load {} and {}".format(img_path1, img_path2))
    
    keypoints1, keypoints2, matches = find_feature_matches(img1, img2)
    print("Find %d matches"%(len(matches)))

    # 估计两张图像间的运动
    R, t, E= pose_estimation_2d2d(keypoints1, keypoints2, matches)

    # 验证E=t^R*scale
    t_x = np.array([0, -t[2,0], t[1,0], t[2,0], 0, -t[0,0], -t[1,0], t[0,0], 0]).reshape([3,3])
    E_x = np.matmul(t_x, R)
    scale = E[0,0] / E_x[0,0]
    E_x *= scale
    print("t^R=\n", E_x)

    # 验证对极约束
    d_avg = 0
    K = np.array([520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1]).reshape([3,3])
    for m in matches:
        pt1 = np.array(pixel2cam(keypoints1[m.queryIdx].pt, K)+[1])
        pt2 = np.array(pixel2cam(keypoints2[m.trainIdx].pt, K)+[1])
        d_avg += pt2.reshape([1,3]) @ E @ pt1.reshape([3,1])
    
    print("average epipolar constraint = ", d_avg[0, 0]/len(matches))

def pixel2cam(p, K):
    return [(p[0]-K[0,2]) / K[0,0], (p[1]-K[1,2]) / K[1,1]] 

def pose_estimation_2d2d(keypoints1, keypoints2, matches):
    K = np.array([520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1]).reshape(3,3)

    points1, points2 = [], []
    for i in range(len(matches)):
        points1.append(keypoints1[matches[i].queryIdx].pt)
        points2.append(keypoints2[matches[i].trainIdx].pt)
    points1 = np.array(points1)
    points2 = np.array(points2)

    # 计算基础矩阵
    fundamental_matrix, _ = cv2.findFundamentalMat(points1, points2, cv2.FM_8POINT) 
    print("fundamental_matrix is", fundamental_matrix)

    # 计算本质矩阵
    principal_point = [325.1, 249.7]
    focal_length = 521
    essential_matrix, _ = cv2.findEssentialMat(points1, points2, focal_length, principal_point)
    print("essential_matrix is", essential_matrix)

    # 从本质矩阵中恢复旋转和平移信息
    _, R, t, _ = cv2.recoverPose(essential_matrix, points1, points2, focal=focal_length, pp=principal_point)
    print("R is\n", R)
    print("t is\n", t)
    return R, t, essential_matrix

def find_feature_matches(img1, img2):
    # 初始化
    orb = cv2.ORB_create()
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    # 检测角点位置
    keypoints1 = orb.detect(img1)
    keypoints2 = orb.detect(img2)

    # 根据角点位置计算BRIEF描述子
    keypoints1, descriptor1 = orb.compute(img1, keypoints1)
    keypoints2, descriptor2 = orb.compute(img2, keypoints2)

    # 对两幅图中的BRIEF描述子进行匹配， 使用Hamming距离
    matches = matcher.match(descriptor1, descriptor2)

    # 匹配点对筛选
    min_dist = matches[0].distance
    max_dist = matches[0].distance
    for i in range(1, len(matches)):
        if matches[i].distance < min_dist:
            min_dist = matches[i].distance

        if matches[i].distance > max_dist:
            max_dist = matches[i].distance 

    print("-- Max dist : %f" %(max_dist))
    print("-- Min dist : %f" %(min_dist))

    good_matches = []
    for i in range(len(matches)):
        if matches[i].distance <= max(2*min_dist, 30):
            good_matches.append(matches[i])
    
    return keypoints1, keypoints2, good_matches

if __name__ == "__main__":
    main()