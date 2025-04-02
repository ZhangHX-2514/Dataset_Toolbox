#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <cfloat>

#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
// #include <opencv2/core/eigen.hpp>
#include <Eigen/Dense> 
#include <image_transport/image_transport.h> 
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>

// #include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include "time.h"
#include <boost/thread.hpp>     
#include <mutex> 
#include <chrono> 
#include <cmath>

using namespace std;

typedef long long int sll;

std::vector<dvs_msgs::Event> event_buffer;
std::vector<sensor_msgs::Imu> imu_buffer;
std::vector<sensor_msgs::Imu> imu_buffer_;
bool first_event_received = true;

//input params
int height_;
int weight_;
float Focus_;
float pixel_size_;

// panorama params
int panorama_height_;
int panorama_width_;
cv::Mat count_pano = cv::Mat::zeros(panorama_height_, panorama_width_, CV_32F); // 初始化为浮点型矩阵，值为 0

double pi = M_PI;
sll t0_p;
// float image_count;
// bool first_got_event_= true;

// Image size
cv::Size imageSize(panorama_width_, panorama_height_);

//main class
class EventVisualizer {
    protected:
        ros::NodeHandle n_;
        //publish topic
        ros::Publisher image_pub;
        ros::Publisher event_pub;
        ros::Publisher pano_pub;

        //subscribe topic
        ros::Subscriber event_sub;
        ros::Subscriber imu_sub;

        std::mutex mtx;

        Eigen::Matrix3d R;
        cv::Mat K, D;

        

    public:
        //construct function
        EventVisualizer(ros::NodeHandle n) : n_(n) {
            // Initialize R
            R << 1., 0., 0.,
                 0., -0.93033817, 0.36670272,
                 0., -0.36670272, -0.93033817;

            // Initialize K
            K = (cv::Mat_<double>(3, 3) << 433.398607901605, 0.0, 235.267698381848,
            0.0, 433.485784159950, 320.716442889263,
            0.0, 0.0, 1.0);

            // Initialize D
            D = (cv::Mat_<double>(1, 5) << -0.0051, 0.1810, -6.3519e-04, 8.4848e-04, -0.4321); //TODO 更新畸变系数

            //sub
            this->event_sub = this->n_.subscribe("/dvs/events", 1, &EventVisualizer::event_cb, this);
            this->imu_sub = this->n_.subscribe("/dvs/imu", 7, &EventVisualizer::imu_cb, this);
            //pub
            this->image_pub = n_.advertise<sensor_msgs::Image>("/count_image", 1);
            this->pano_pub = n_.advertise<sensor_msgs::Image>("/count_pano", 1);
            this->event_pub = n_.advertise<dvs_msgs::EventArray>("/event_new", 1);

        
        }

        //visualize
        void show_count_image(std::vector<std::vector<int>>&count_image,int& max_count, ros::Time timestamp);
        void show_count_pano(cv::Mat& count_image, double& max_count, ros::Time timestamp);

        //main process function
        void data_process();

        // void precomputeBearingVectors();

        void events(const std::vector<dvs_msgs::Event>& event_buffer, int size);

        //event cb
        void event_cb(const dvs_msgs::EventArray::ConstPtr& msg) {
            if (first_event_received == false) {

                mtx.lock();
                imu_buffer_ = imu_buffer;
                if (imu_buffer.size() != 0) {
                    imu_buffer.clear();
                }
                mtx.unlock();

                if (imu_buffer_.size() == 0) {
                    return;
                }

                for (int i = 0; i < msg->events.size(); ++i) {

                    event_buffer.emplace_back(msg->events[i]);
                }

                //core function
                data_process();
            } else {
                first_event_received = false; //Discard the first set of data to complete data matching

                if (imu_buffer.size() != 0) {
                    imu_buffer.clear();
                }

                std::cout << "Data aligned!" << std::endl;
                std::cout << "Start processing data..." << std::endl;
            }
        }

        //imu cb
        void imu_cb(const sensor_msgs::ImuConstPtr& imu) {
            if (first_event_received == false) {
                imu_buffer.emplace_back(*imu);
            }
        }
};


//Show compensated count image 
void EventVisualizer::show_count_image(std::vector<std::vector<int>>&count_image, int& max_count, ros::Time timestamp){
    using namespace cv;
    cv::Mat image(height_,weight_,CV_8UC1);
    int scale = (int)(255/max_count) + 1;
    for(int i = 0;i < height_;++i){
            for(int j = 0; j < weight_;++j){
                    image.at<uchar>(i,j) = count_image[i][j]*scale;
            }
    }

    std_msgs::Header header;
    header.stamp = timestamp;

    //Change to sensor_message
    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
    image_pub.publish(*msg2); 
}


void EventVisualizer::show_count_pano(cv::Mat& count_image, double& max_count, ros::Time timestamp) {
    // 创建灰度图像
    cv::Mat image(panorama_height_, panorama_width_, CV_8UC1);

    // 计算缩放比例，将 count_image 的值映射到 [0, 255]
    int scale = (int)(255 / max_count) + 1;
    count_image.convertTo(image, CV_8UC1, scale); // 将浮点型矩阵转换为 8 位无符号整型矩阵

    // 创建带时间戳的 ROS 消息头
    std_msgs::Header header;
    header.stamp = timestamp;
    // std::cout<<image.type()<<endl;

    // 将 OpenCV 图像转换为 ROS 图像消息
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

    // 发布图像消息
    pano_pub.publish(msg);
}

void EventVisualizer::events(const std::vector<dvs_msgs::Event>& event_buffer, int size) {
    // 创建一个 EventArray 消息
    dvs_msgs::EventArray event_array_msg;

    // 设置头信息
    event_array_msg.header.stamp = ros::Time::now();
    event_array_msg.header.frame_id = std::to_string(size); // 设置适当的 frame_id

    // 设置图像的宽高
    event_array_msg.height = 480;  // 示例高度
    event_array_msg.width = 640;   // 示例宽度

    // 将事件数据从 event_buffer 填充到 EventArray 消息
    event_array_msg.events = event_buffer;

    // 发布 EventArray 消息
    event_pub.publish(event_array_msg);
}



// void EventVisualizer::precomputeBearingVectors()
// {
//     int sensor_width = cam.fullResolution().width;
//     int sensor_height = cam.fullResolution().height;

//     for(int y=0; y < sensor_height; y++)
//     {
//         for(int x=0; x < sensor_width; x++)
//         {
//             cv::Point2d rectified_point = cam.rectifyPoint(cv::Point2d(x,y));
//             cv::Point3d bearing_vec = cam.projectPixelTo3dRay(rectified_point);
//             precomputed_bearing_vectors.emplace_back(bearing_vec);
//         }
//     }
// }


void EventVisualizer::data_process() {

    if (imu_buffer_[imu_buffer_.size() - 1].header.stamp.toNSec() > event_buffer[0].ts.toNSec()) {

        // Contruct panorama image
        Eigen::Vector2d center((double)panorama_width_ / 2.0, (double)480);

        float t_diff = 0.0; //time diff
        int size = event_buffer.size();

        // if(first_got_event_){
            // t0_p = event_buffer[0].ts.toNSec();
            // image_count=0.0;
            // first_got_event_=false;
        // }
        
        for (int i = 0; i < event_buffer.size(); i+=5) {

            // 1. Calculate the rotation matrix and vector in camera axis
            t_diff = double(event_buffer[i].ts.toNSec() - t0_p) / 1000000000.0;

            double cos_term = cos(2 * pi * t_diff);
            double sin_term = sin(2 * pi * t_diff);
            Eigen::Matrix3d R_eigen;
            R_eigen << cos_term, R(2, 1) * sin_term, R(2, 2) * sin_term,
                       0, R(1, 1), R(1, 2),
                       -sin_term, R(2, 1) * cos_term, R(2, 2) * cos_term;

            // undistort the event point
            std::vector<cv::Point2f> distorted_points = {cv::Point2f(event_buffer[i].x, event_buffer[i].y)};
            std::vector<cv::Point2f> undistorted_points;

            cv::undistortPoints(distorted_points, undistorted_points, K, D);
            Eigen::Vector3d e_ray_cam(undistorted_points[0].x, undistorted_points[0].y, 1.0);

            // Eigen::Vector3d e_ray_cam;
            // e_ray_cam.x() = (event_buffer[i].x - K.at<double>(0, 2)) / K.at<double>(0, 0);
            // e_ray_cam.y() = (event_buffer[i].y - K.at<double>(1, 2)) / K.at<double>(1, 1);
            // e_ray_cam.z() = 1.0;

            // 2. Rotate according to pose(t)
            Eigen::Vector3d e_ray_w = R_eigen * e_ray_cam;

            // 3. equirectangular projection
            Eigen::Vector2d px_mosaic;

            // Calculate the pixel position in the panorama image
            const double phi = std::atan2(e_ray_w[0], e_ray_w[2]);
            const double theta = std::asin(e_ray_w[1] / e_ray_w.norm());

            const double rho = e_ray_w.norm();
            const double Ydivrho = e_ray_w[1] / rho;

            px_mosaic = center + Eigen::Vector2d(-phi * panorama_width_ / (2.0 * pi), -theta * panorama_height_ / (pi*57.99/180));
            // cout<<px_mosaic<<endl;
            const int xx = px_mosaic[0],
                      yy = px_mosaic[1];
            const float dx = px_mosaic[0] - xx,
                        dy = px_mosaic[1] - yy;

            // 4. Update the count image using bilinear voting
            if (1 <= xx && xx < panorama_width_ - 1 && 1 <= yy && yy < panorama_height_ - 1) {
                if (t_diff  < 1.) {
                    count_pano.at<float>(yy, xx) += (1.f - dx) * (1.f - dy);
                    count_pano.at<float>(yy, xx + 1) += dx * (1.f - dy);
                    count_pano.at<float>(yy + 1, xx) += (1.f - dx) * dy;
                    count_pano.at<float>(yy + 1, xx + 1) += dx * dy;
                } else {
                    // cout<<2<<endl;
                    double min_val, max_val;
                    cv::Point min_loc, max_loc;
                    // cv::minMaxLoc(count_pano, &min_val, &max_val, &min_loc, &max_loc);
                    // cout<<min_val<<' '<<max_val<<endl;
                    // 将大于20的值截断为20（类似ReLU的截断效果）
                    max_val = 20.0;
                    cv::threshold(count_pano, count_pano, max_val, max_val, cv::THRESH_TRUNC);
                    
                    show_count_pano(count_pano, max_val, ros::Time().fromNSec(t0_p));
                    // show_count_pano(count_pano, max_val, event_buffer[0].ts);
                    count_pano = cv::Mat::zeros(panorama_height_, panorama_width_, CV_32F); // 重置为 0
                    t0_p = event_buffer[i].ts.toNSec();
                }
            }
        }
        //release buffer
        event_buffer.clear();
        imu_buffer_.clear();
    } else {
        //keep safe 
        if (event_buffer.size() != 0) {

            event_buffer.clear();
        }
        if (imu_buffer_.size() != 0) {

            imu_buffer_.clear();
        }
    }
}



//main
int main(int argc, char** argv) {
    ros::init(argc, argv, "datasync_node");

    ros::NodeHandle nh;
    EventVisualizer visualizer(nh);
    ros::NodeHandle nh_priv("~");

    nh_priv.param<int>("weight_param", weight_, 346);
    nh_priv.param<int>("height_param", height_, 260);
    nh_priv.param<int>("panorama_weight", panorama_width_, 3163);
    nh_priv.param<int>("panorama_height", panorama_height_, 480);
    nh_priv.param<float>("focus", Focus_, 6550);
    nh_priv.param<float>("pixel_size", pixel_size_, 18.5);

    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}