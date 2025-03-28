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
#include <opencv2/core/eigen.hpp>
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
int panorama_weight_;
float panorama_angle_;  // left angle
cv::Mat count_image(panorama_height_,std::vector<int>(panorama_weight_));//count image

//Rotation matrix
Eigen::Matrix3d R;
        R << 1., 0., 0.
             0., -0.93033817, 0.36670272
             0., -0.36670272, -0.93033817;
double c1 = R[1, 1];
double c2 = R[1, 2];
double c3 = R[2, 1];
double c4 = R[2, 2];
double pi = M_PI;
sll t0;

// Intrinsics Matrix    //TODO 更新相机内参
Eigen::Matrix3d K;
    K << 800, 0, 320,
         0, 800, 240,
         0, 0, 1;
double fx = K[0, 0];
double fy = K[1, 1];
double cx = K[0, 2];
double cy = K[1, 2];

//main class
class EventVisualizer{
        protected:
                 ros::NodeHandle n_;
                 //publish topic
                 ros::Publisher  image_pub;
                 ros::Publisher  event_pub;

                 //subscribe topic
                 ros::Subscriber event_sub;
                 ros::Subscriber imu_sub;

                 std::mutex mtx;
        public:
                 //construct function
                 EventVisualizer (ros::NodeHandle n):
                        n_(n){
                                //sub
                                this->event_sub = this->n_.subscribe("/dvs/events", 1, &EventVisualizer::event_cb, this);
                                this->imu_sub = this->n_.subscribe("/dvs/imu", 7, &EventVisualizer::imu_cb, this);
                                //pub
                                this->image_pub = n_.advertise<sensor_msgs::Image> ("/count_image", 1);
                                this->event_pub = n_.advertise<dvs_msgs::EventArray>("/event_new", 1);
                        }

                 //visualize
                 void show_count_image(std::vector<std::vector<int>>& count_image, int& max_count, ros::Time timestamp);

                 //main process function
                 void data_process();

                 void events(const std::vector<dvs_msgs::Event>& event_buffer,int size);
                 
                 //event cb
                 void event_cb(const dvs_msgs::EventArray::ConstPtr& msg){
                        if (first_event_received == false) {

                                mtx.lock();
                                imu_buffer_ = imu_buffer;
                                if(imu_buffer.size() != 0){
                                    imu_buffer.clear();
                                }
                                mtx.unlock();

                                if(imu_buffer_.size() == 0){
                                       return;
                                }
                    
                                for (int i = 0; i < msg->events.size(); ++i) {
                                         event_buffer.emplace_back(msg->events[i]);
                                        
                                }

                                //core function
                                data_process();
                        }else{
                                first_event_received = false;//Discard the first set of data to complete data matching
                                
                                if(imu_buffer.size() != 0){
                                      imu_buffer.clear();  
                                }

                                std::cout<<"Data aligned!"<<std::endl;
                                std::cout<<"Start processing data..."<<std::endl;
                        }
                }

                //imu cb
                 void imu_cb(const sensor_msgs::ImuConstPtr& imu){
                         if(first_event_received == false){
                                 imu_buffer.emplace_back(*imu);        
                        }
                }
}; 


//Show compensated count image 
void EventVisualizer::show_count_image(std::vector<std::vector<int>>& count_image, int& max_count, ros::Time timestamp) {
    using namespace cv;
    cv::Mat image(height_, weight_, CV_8UC1);
    int scale = (int)(255 / max_count) + 1;
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < weight_; ++j) {
            image.at<uchar>(i, j) = count_image[i][j] * scale;
        }
    }

    // Create a header with the specified timestamp
    std_msgs::Header header;
    header.stamp = timestamp;  // Set the timestamp

    // Convert the OpenCV image to a sensor_msgs::Image message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

    // Publish the image message
    image_pub.publish(msg);
    }

void EventVisualizer::events(const std::vector<dvs_msgs::Event>& event_buffer,int size){
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
void EventVisualizer::data_process(){
        
         if(imu_buffer_[imu_buffer_.size() - 1].header.stamp.toNSec() > event_buffer[0].ts.toNSec()){
                //  auto T0 = std::chrono::high_resolution_clock::now();
                 float angular_velocity_x=0.0, angular_velocity_y=0.0,angular_velocity_z=0.0;
                 float average_angular_rate_x, average_angular_rate_y,average_angular_rate_z;

                 
                 int cnt=0;//imu counter

                 for(int i=0;i<imu_buffer_.size();++i){
                         if(imu_buffer_[i].header.stamp.toNSec() >= (event_buffer[0].ts.toNSec()-3000000)){
                                 angular_velocity_x+=imu_buffer_[i].angular_velocity.x;
                                 angular_velocity_y+=imu_buffer_[i].angular_velocity.y;
                                 angular_velocity_z+=imu_buffer_[i].angular_velocity.z;
                                 cnt++;
                        }  
                }
                 //Calculate the average imu angular rates
                 average_angular_rate_x = angular_velocity_x/float(cnt);
                 average_angular_rate_y = angular_velocity_y/float(cnt);
                 average_angular_rate_z = angular_velocity_z/float(cnt);
                 float average_angular_rate = std::sqrt((average_angular_rate_x*average_angular_rate_x) + (average_angular_rate_y*average_angular_rate_y) + (average_angular_rate_z*average_angular_rate_z));
                //  auto T1 = std::chrono::high_resolution_clock::now();

                // Contruct panorama image
                
                Eigen::Vector2d center((double)imageSize.width / 2.0, (double)imageSize.height / 2.0);
                 
                //  sll t0=event_buffer[0].ts.toNSec();//the first event
                 float time_diff = 0.0;//time diff
                 int count =0; //event counter
                 int size=event_buffer.size();
                //  std::vector<std::vector<int>>count_image(height_,std::vector<int>(weight_));//count image
                //  std::vector<std::vector<float>>time_image(height_,std::vector<float>(weight_));//time image
                for(int i=0;i<event_buffer.size();++i){

                        // 1. Calculate the rotation matrix and vector in camera axis
                        time_diff = double(event_buffer[i].ts.toNSec()-t0)/1000000000.0;

                        double cos_term = cos(2 * pi * time_diff);
                        double sin_term = sin(2 * pi * time_diff);
                        Eigen::Matrix3d R_eigen;
                        R_eigen << cos_term, c3*sin_term, c4*sin_term,
                                   0, c1, c2,
                                   sin_term, c3*cos_term, c4*cos_term;

                        Eigen::Vector3d e_ray_cam;
                        e_ray_cam.x = (event_buffer[i].x - cx) / fx;
                        e_ray_cam.y = (event_buffer[i].y - cy) / fy;
                        e_ray_cam.z = 1.0;

                        // 2. Rotate according to pose(t)
                        Eigen::Vector3d e_ray_w = R_eigen * e_ray_cam;

                        // 3. equirectangular projection
                        Eigen::Vector2d px_mosaic;
                        
                        // Calculate the pixel position in the panorama image
                        const double phi = std::atan2(e_ray_w[0], e_ray_w[2]);
                        const double theta = std::asin(e_ray_w[1] / e_ray_w.norm());

                        const double rho = e_ray_w.norm();
                        const double Ydivrho = e_ray_w[1] / rho;

                        Eigen::Vector2d px_mosaic = center + Eigen::Vector2d(phi * fx, theta * fy);
                        const int xx = px_mosaic[0],
                                yy = px_mosaic[1];
                        const float dx = px_mosaic[0] - xx,
                                dy = px_mosaic[1] - yy;
                        
                        // 4. Update the count image using bilinear voting
                        if (1 <= xx && xx < weight_ - 1 && 1 <= yy && yy < height_ - 1) {
                            if (ev->ts - t0 < t_next)   // unfinish the panorama image
                            {
                                count_image.at<float>(yy, xx) += (1.f-dx)*(1.f-dy);
                                count_image.at<float>(yy  ,xx+1) += dx*(1.f-dy);
                                count_image.at<float>(yy+1,xx  ) += (1.f-dx)*dy;
                                count_image.at<float>(yy+1,xx+1) += dx*dy;
                                event_buffer[count].x = event_buffer[i].x;
                                event_buffer[count].y = event_buffer[i].y;
                                count++;

                            }
                            else
                            {
                                // Publish the panorama image and reset the count image
                                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", count_image).toImageMsg();
                                image_pub.publish(msg);
                                count = 0;
                                count_image = cv::Mat::zeros(height_, weight_, CV_32F);
                                t0 = ev->ts;
                            }
                        }

                }

                // event_buffer.erase(event_buffer.begin() + count, event_buffer.end());

                // auto T2 = std::chrono::high_resolution_clock::now();

                //  int max_count = 0;
                //  float max_time = 0.0;
                //  float total_time = 0.0;
                //  float average_time = 0.0;
                //  int trigger_pixels = 0;

                //  for(int i = 0; i<height_; ++i){
                //         for(int j = 0; j < weight_; ++j){
                //                 if(count_image[i][j] != 0){
                //                         time_image[i][j] /= count_image[i][j];
                //                         max_count = std::max(max_count,count_image[i][j]);
                //                 }
                //         }
                //  }

                // events(event_buffer,size);
                // show_count_image(count_image, max_count,event_buffer[0].ts);  
                // auto T3 = std::chrono::high_resolution_clock::now();
                //  auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(T1-T0);
                //  auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(T2-T1);
                //  auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(T3-T2);
                //  auto duration4 = std::chrono::duration_cast<std::chrono::microseconds>(T3-T0);
                //  ROS_INFO_STREAM("T1-T0: " << duration1.count());
                //  ROS_INFO_STREAM("T2-T1: " << duration2.count());
                //  ROS_INFO_STREAM("T3-T2: " << duration3.count());
                //  ROS_INFO_STREAM("T3-T0: " << duration4.count());

                //  ROS_INFO_STREAM("size: " << event_buffer.size());

                 //release buffer
                 event_buffer.clear();
                 imu_buffer_.clear();
        }else{
                 //keep safe 
                if(event_buffer.size() != 0){

                        event_buffer.clear();
                } 
                if(imu_buffer_.size() != 0){

                        imu_buffer_.clear();
                }
        }
}






//main
  int main(int argc, char** argv)
   {
   ros::init(argc, argv, "datasync_node");
 
   ros::NodeHandle nh;
   EventVisualizer visualizer(nh);
   ros::NodeHandle nh_priv("~");

   nh_priv.param<int>("weight_param", weight_, 346);
   nh_priv.param<int>("height_param", height_, 260);
   nh_priv.param<int>("panorama_weight", panorama_weight_, 706);
   nh_priv.param<int>("panorama_height", panorama_height_, 260);
   nh_priv.param<float>("focus", Focus_, 6550);
   nh_priv.param<float>("pixel_size",pixel_size_ , 18.5);


   ros::AsyncSpinner spinner(3); // Use 3 threads
   spinner.start();
   ros::waitForShutdown();

   return 0;
 }
