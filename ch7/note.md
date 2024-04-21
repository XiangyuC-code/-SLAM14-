# cv::ORB::create()
创建ORB对象：
```cpp
auto orb = cv::ORB::create();
```
函数原型：
```cpp
static Ptr<ORB> cv::ORB::create (
    int     nfeatures = 500,    //The maximum number of features to retain.
    float   scaleFactor = 1.2f,
                                    //金字塔抽取比，大于1。scaleFactor==2表示经典金字塔，每一层的像素都比上一层少4倍，
                                    //但如此大的尺度因子会显著降低特征匹配得分。
                                    //另一方面，过于接近1的比例因素将意味着要覆盖一定的比例范围，
                                    //你将需要更多的金字塔级别，因此速度将受到影响。
    int     nlevels = 8,        //金字塔等级的数量。
                                    //最小级别的线性大小等于input_image_linear_size/pow(scaleFactor, nlevels).
                                    //输入图像线性大小/功率(缩放因子，非线性)。
    int     edgeThreshold = 31,  //这是未检测到特征的边框大小。它应该大致匹配patchSize参数.
    int     firstLevel = 0,     //It should be 0 in the current implementation.
    int     WTA_K = 2,          //产生oriented BRIEF描述子的每个元素的点数。
    int     scoreType = ORB::HARRIS_SCORE,
                                    //默认的HARRIS_SCORE意味着HARRIS算法用于对特征进行排序
                                    //(分数被写入KeyPoint::score，并用于保留最佳特征)；
                                    //FAST_SCORE是参数的另一个值，它会产生稍微不太稳定的关键点，
                                    //但是计算起来要快一点。
    int     patchSize = 31,     //oriented BRIEF描述符使用的补丁大小。
                                    //在较小的金字塔层上，被特征覆盖的感知图像区域会更大。
    int     fastThreshold = 20
   )
```

使用ORB算法提取关键点和描述子：
```cpp
cv::Mat img = imread("img.png", 1);
vector<cv::KeyPoint> keypoint;
cv::Mat descriptor;
orb->detect(img, keypoint);  //检测关键点
orb->compute(img, keypoint, descriptor);  //计算描述子

//展示关键点
cv::Mat outimg;
cv::drawKeypoints(img, keypoint, outimg);
imshow("ORB Feature", outimg);
```

函数原型：
```cpp
void xfeatures2d::SURT::detect( InputArray image,   //输入图像，类型为cv::Mat
                                std::vector<KeyPoint>& keypoints,   //检测到的关键点
                                InputArray mask=noArray() );   //InputArray类型的mask，默认为空，指定在何处查找关键点的掩码（可选）。它必须是8位整数感兴趣区域中具有非零值的矩阵


void xfeatures2d::SURT::compute( InputArray image,   //输入图像，类型为cv::Mat
                                 std::vector<KeyPoint>& keypoints,  //检测到的关键点
                                 OutputArray descriptors );  //计算得到的描述子，类型为cv::Mat

void drawKeypoints( InputArray image,   //输入图像，类型为cv::Mat
                    const std::vector<KeyPoint>& keypoints,   //检测到的关键点
                    InputOutputArray outImage,   //输出的图像，类型为cv::Mat
                    const Scalar& color=Scalar::all(-1),  //cv::Scalar类型的color，绘制关键点的颜色，默认为Scalar::all(-1)随机颜色，每个点都是这个颜色，那么随机时，每个点都是随机的
                    int flags=DrawMatchesFlags::DEFAULT );  
```

计算得到的keypoints有很多属性，常用的是pt和angle，分别表示计算得到的关键点的位置和方向：
```cpp
keypoint[0].pt.x
keypoint[0].pt.y
keypoint[0].angle
```

# cv::DescriptorMatcher::create()
匹配描述符。创建对象:
```cpp
auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
```

函数原型:
```cpp
cv::Ptr<cv::DescriptorMatcher> descriptorMatcher = cv::DescriptorMatcher::create(const string& descriptorMatcherType)
//  descriptorMatcherType – Descriptor matcher type.   
//  Now the following matcher types are supported:   
//      BruteForce (it uses L2 )   
//      BruteForce-L1   
//      BruteForce-Hamming   
//      BruteForce-Hamming(2)   
//      FlannBased   
```

匹配描述子：
```cpp
vector<cv::DMatch> matches;
cv::matcher->match(descriptor1, descriptor2, matches);

cv::Mat img_match;
cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);  //绘制两幅图间的匹配关系，结果保存在img_matche中
imshow("matches", img_match);
```

匹配结果存储在matches中，matches的数据格式是元素为DMatch的vector，保存了两个匹配的描述子的索引，以及相应的距离：
```cpp
matches[0].queryIdx  //查询描述子的索引，与descriptor1的元素相对应
matches[0].trainIdx  //与queryIdx匹配的描述子的索引，与descriptor2的元素相对应
matches[0].distance  //query和train两个描述子的距离
```

# cv::findFundamentalMat()

求基础矩阵F，函数原型：
```cpp
Mat findFundamentalMat(InputArray points1, InputArray points2, int method=FM_RANSAC, double param1=3., double param2=0.99, OutputArray mask=noArray() )

// 参数
// points1, 第一幅图像点的数组
// points2, 第二幅图像点的数组, points1和points2为点的像素坐标，形式为[u, v, 1]^T
// method = FM_RANSAC, RANSAC 算法
// param1 = 3., 点到对极线的最大距离，超过这个值的点将被舍弃
// param2 = 0.99, 矩阵正确的可信度
// mask = noArray() 内点向量索引

```

# cv::findEssentialMat()
求本质矩阵E，函数原型:
```cpp
// 形式1
Mat findEssentialMat( InputArray points1, InputArray points2,
                      InputArray cameraMatrix, int method = RANSAC,
                      double prob = 0.999, double threshold = 1.0,
                      OutputArray mask = noArray() );

// 参数
// points1, 第一幅图像点的数组
// points2, 第二幅图像点的数组, points1和points2为点的像素坐标，形式为[u, v, 1]^T
// cameraMatrix, 相机内参矩阵
// method = RANSAC, RANSAC 算法
// prob 估计矩阵正确的可信度（概率），仅用于RANSAC或LMedS方法的参数；
// threshold 用于RANSAC的参数。 它是从点到极线的最大距离（以像素为单位）；超出此距离时，该点被视为异常值，不用于计算最终的基本矩阵。 根据点定位精度、图像分辨率和图像噪声的不同，可将其设置为1~3；
// mask = noArray() 内点向量索引

// 形式2
Mat findEssentialMat( InputArray points1, InputArray points2,
                      double focal=1.0, Point2d pp = Point2d(0,0),
                      int method = RANSAC, double prob = 0.999, 
                      double threshold = 1.0, OutputArray mask = noArray() );

// focal和pp分别表示相机焦距和光心坐标，可以由内参矩阵计算得到。
// focal = (cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(1, 1)) / 2
// pp.x = cameraMatrix.at<double>(0, 2),  pp.y = cameraMatrix.at<double>(1, 2)
```
# cv::recoverPose()
计算相机由位姿1到位姿2的旋转和平移矩阵，函数原型:
```cpp
// 形式1
int cv::recoverPose	( InputArray E, InputArray points1,
                      InputArray points2, InputArray cameraMatrix,
                      OutputArray R, OutputArray t,
                      InputOutputArray mask = noArray() );

// E为本质矩阵， R为输出的旋转矩阵， t为输出的位移矩阵。 其他参数与findEssentialMat含义相同。

// 形式2
int cv::recoverPose	( InputArray E, InputArray points1,
                      InputArray points2, OutputArray R, 
                      OutputArray t, double focal=1.0, 
                      Point2d pp = Point2d(0,0), InputOutputArray mask = noArray() );

```

参考:<br>
https://www.cnblogs.com/qq21497936/p/13184583.html