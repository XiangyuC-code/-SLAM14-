import cv2
import os
os.chdir(os.path.dirname(__file__))

img_path1 = "1.png"
img_path2 = "2.png"

# 读取图像
img1 = cv2.imread(img_path1, cv2.IMREAD_COLOR)
img2 = cv2.imread(img_path2, cv2.IMREAD_COLOR)
if img1 is None or img2 is None:
    print("Can not load {} and {}".format(img_path1, img_path2))

# 初始化
orb = cv2.ORB_create()
matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

# 检测Oriented Fast角点位置
keypoint1 = orb.detect(img1)
keypoint2 = orb.detect(img2)

# 根据角点位置计算BRIEF描述子
keypoint1, descriptor1 = orb.compute(img1, keypoint1)
keypoint2, descriptor2 = orb.compute(img2, keypoint2)

out_img1 = cv2.drawKeypoints(img1, keypoint1, None)
cv2.imshow("ORB features", out_img1)

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

# 绘制匹配结果
img_match = cv2.drawMatches(img1, keypoint1, img2, keypoint2, matches, outImg=None)
img_goodmatch = cv2.drawMatches(img1, keypoint1, img2, keypoint2, good_matches, outImg=None)
cv2.imshow("all matches", img_match)
cv2.imshow("good matches", img_goodmatch)
cv2.waitKey(0)