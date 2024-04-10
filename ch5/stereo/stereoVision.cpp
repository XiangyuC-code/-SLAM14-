#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

// 文件路径
string left_file = "/home/c/slambook2/ch5/stereo/build/left.png";
string right_file = "/home/c/slambook2/ch5/stereo/build/right.png";

// 在pangolin中画图，已写好，无需调整
void showPointCloud(
    const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc, char **argv) {

    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // 基线
    double b = 0.573;

    
    cv::Mat left = cv::imread(left_file);
    cv::Mat right = cv::imread(right_file);
    //StereoSGBM和StereoBM是两种立体匹配算法，用于计算视差图像；参数设置采取了比较经典的参数设置
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8*9*9, 32*9*9, 1, 63, 10, 100, 32);
    
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(left, right, disparity_sgbm);
    //计算左右眼图像的视差，并进行矩阵转换。这里将disparity_sgbm转换为32位浮点的disparity，并且除以16
    //这里除以16是因为disparity_sbgm中的每一个像素值用16bit表示，第四位是小数，除以16去除小数部分得到真实视差值
    //用于计算的视差值并不需要归一化，显示时需要归一化，因此第60行代码除以了96用以归一化视差值
    //参考：https://blog.csdn.net/weixin_70026476/article/details/127351340
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0/16.0f);
    
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;
    
    for (int v=0; v<left.rows; ++v){
        for (int u=0; u<left.cols; ++u){
            if(disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;
            
            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0);
            
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));
            point[0] = x* depth;
            point[1] = y* depth;
            point[2] = depth;
            
            pointcloud.push_back(point);
        }
    }
    
    //归一化视差值
    cv::imshow("disparity", disparity/96.0);
    cv::waitKey(0);
    
    showPointCloud(pointcloud);
    return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
