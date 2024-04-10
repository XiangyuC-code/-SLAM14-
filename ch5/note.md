# cv::StereoSGBM::create()

```cpp 
static Ptr<StereoSGBM> create(
        int minDisparity = 0, int numDisparities = 16,  
        int blockSize = 3,  int P1 = 0, int P2 = 0, int disp12MaxDiff = 0,
        int preFilterCap = 0, int uniquenessRatio = 0,  int speckleWindowSize = 0, 
        int speckleRange = 0,int mode = StereoSGBM::MODE_SGBM);
``` 
```
minDisparity 最小的可能的视差值。正常情况下，它为零，但有时校正算法会移动图像，因此需要相应调整此参数。
 
numDisparity 最大视差减去最小视差。该值始终大于零。在实现中，此参数必须可以被16整除。
 
blockSize   匹配的块大小。它必须是奇数>=1。通常情况下，它应该在3-11范围
 
P1, P2：控制视差变化平滑性的参数。P1、P2的值越大，视差越平滑。P1是相邻像素点视差增/减 1 时的惩罚系数；P2是相邻像素点视差变化值大于1时的惩罚系数。P2必须大于P1。
 
disp12MaxDiff是左右图视差检查所允许的最大的不同值
 
preFilterCap：预处理滤波器的截断值，该算法首先计算每个像素的x导数，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值。结果值将传递给Birchfield-Tomasi像素代价函数。
 
uniquenessRatio：视差唯一性百分比。通常，5-15范围内的值就足够了。
 
speckleWindowSize  检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查。否则，将其设置在50-200范围内。
 
speckleRange：视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零，否则将参数设置为正值，它将隐式地乘以16。通常，1或2就足够了。
 
mode 将其设置为StereoSGBM:：MODE_HH以运行全刻度双通道动态编程算法。默认情况下，它设置为false。
```

# cv::Ptr<>
cv::Ptr<>与share_ptr<>类似，也是一种智能指针，括号内表示指向数据的数据类型。

# src.convertTo(dst, type, scale, shift)
opencv的数据类型转换函数，src的数据类型为cv::Mat。将src的数据类型装换为type类型，并将转换后的数据乘scale加shift，最后的结果保存在dst中。
$$dst(i) = src(i) \times scale + shift$$
该函数常用于图像类型的转换，如将CV_8U的数据转换成CV_32F，并设置$scale=1.0/255.0$，方便图像后续处理。

# vector<xxx, Eigen::aligned_allocator<xxx>>
Eigen::aligened_allocator<xxx>告诉vector元素xxx的内存大小。当xxx是Eigen中定义的元素时，需要添加该命令，因为Eigen的内存管理方式和c++11中的内存管理方式不同；否则会报错

# boost::format
类似于python中的format,用于处理字符串。
```cpp
#include <boost/format.hpp>

boost::format fmt("./%s/%d.%s");
cv::imread((fmt % "color" % 1 % "png").str());
```
将实际内容按照fmt定义的格式填入即可。


# 访问cv::Mat元素的方法
## 1.at()
mat.at<type>(int row, int col)

```cpp
cv::Mat m = (cv::Mat_<int>(3, 2) << 1,2,3,4,5,6);
cout<<m.at<int>(2, 1)<<endl; //返回第3行第2列元素，值为6
cout<<m<<endl;
/*
[1, 2,
 3, 4,
 5, 6]
*/
```
<br>
mat.at<type>(Point pt)

```cpp
cv::Mat m = (cv::Mat_<int>(3, 2) << 1,2,3,4,5,6);
cout<<m.at<int>(cv::Point(1, 2))<<endl; //返回第3行第2列元素，值为6. Point(c, r)是先列后行
cout<<m<<endl;
/*
[1, 2,
 3, 4,
 5, 6]
*/
```
at返回的都是元素的引用。

## 2.ptr()
mat.ptr<type>(int r), 返回第r行的行首指针
```cpp
cv::Mat m = (cv::Mat_<int>(3, 2) << 1,2,3,4,5,6);
int* ptr = m.ptr<int>(1);  //ptr指向第2行行首
cout<<ptr[0]<<", "<<ptr[1]<<endl;  //输出3, 4
```
<br>
mat.ptr<type>(int r, int c), 返回第r行第c列元素的指针

```cpp
cv::Mat m = (cv::Mat_<int>(3, 2) << 1,2,3,4,5,6);
cout<<*m.ptr<int>(0, 1)<<endl;  //输出2
```

## 3.step()
step或者step[0]表示矩阵每一行元素的字节数；<br>
step[1]表示矩阵每一个元素的字节数(包含通道，如矩阵为3通道，则step[1]为3个通道所有的字节数);<br>
step1(0):矩阵中一行有几个通道数;<br>
step1(1):一个元素有几个通道数(channel());<br>
```cpp
cv::Mat img(3, 4, CV_16UC4, cv::Scalar_<uchar>(1, 2, 3, 4));
cout << img << endl;
cout << "step:" << img.step << endl;
cout << "step[0]:" << img.step[0] << endl;
cout << "step[1]:" << img.step[1] << endl;
cout << "step1(0):" << img.step1(0) << endl;
cout << "step1(1):" << img.step1(1) << endl;

/*
[1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4;
1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4;
1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4]
step[0]:32
step[1]:8
step(0):16
step(1):4
*/
```
使用mat.data与step访问矩阵元素
```cpp
cv::Mat img(3, 4, CV_16UC4, cv::Scalar(1, 2, 3, 4));

for(int i=0; i<img.rows; ++i){
    for(int j=0; j<img.cols; ++j){
        for(int c=0; c<img.channels(); ++c){ 
            cout<<(ushort)img.data[img.step[0]*i+img.step[1]*j+c*sizeof(ushort)]<<",";
            //或者cout<<(ushort)*(img.data+img.step[0]*i+img.step[1]*j+c*2)<<",";
        }
    }
    cout<<endl;
}


/*
[1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4,
1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4,
1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4]
*/
```
使用mat.data访问矩阵元素的时候，注意是按照字节数对元素进行索引的。如数据类型为CV_16UC4，则一个元素占2字节，第一行第5个元素的地址为mat.data+5*sizeof(ushort)。<br>
上文代码中使用at方法和ptr方法替换step方法：
```cpp
cout<<img.at<ushort>(i, j*img.channels()+c)<<",";

ushort* ptr = img.ptr<ushort>(i);
cout<<ptr[j*img.channels()+c]<<",";
```

参考：<br>
https://blog.csdn.net/qq_41451702/article/details/122043813
https://blog.csdn.net/weixin_70026476/article/details/127351340
https://blog.csdn.net/weixin_43520503/article/details/133432798?spm=1001.2014.3001.5501
https://blog.csdn.net/libaineu2004/article/details/132068672