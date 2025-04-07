#include "panorama.h"
#include <glog/logging.h>
#include <filesystem>
#include <ctime>

#include <iostream>
using namespace std;  


namespace panorama {

Panorama::Panorama(ros::NodeHandle& nh) 
    : nh_(nh), 
      pnh_("~"), 
      got_camera_info_(false) 
{
    const std::string events_topic = pnh_.param<std::string>("events_topic", "/dvs/events");
    const std::string camera_info_topic = pnh_.param<std::string>("camera_info_topic", "/dvs/camera_info");

    LOG(INFO) << "Event topic: " << events_topic;
    LOG(INFO) << "Camera info topic: " << camera_info_topic;

    camera_info_sub_ = nh_.subscribe(camera_info_topic, 0, &Panorama::cameraInfoCallback, this);
    event_sub_ = nh_.subscribe(events_topic, 0, &Panorama::eventsCallback, this);
    got_camera_info_ = false;
    is_t0_p_set_ = false; 

    front_end_params_.warp_opt.event_sample_rate = pnh_.param<int>("frontend_event_sample_rate", 1);
    front_end_params_.image_opt.panorama_height = pnh_.param<int>("panorama_height", 1);
    front_end_params_.image_opt.panorama_width = pnh_.param<int>("panorama_width", 1);

    std::vector<double> default_R_matrix = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // 默认单位矩阵

    if (!pnh_.getParam("R_matrix", front_end_params_.image_opt.R_matrix)) {
        front_end_params_.image_opt.R_matrix = default_R_matrix;
        ROS_WARN("No R_matrix provided, using identity matrix");
    } else if (front_end_params_.image_opt.R_matrix.size() != 9) {
        front_end_params_.image_opt.R_matrix = default_R_matrix;
        ROS_ERROR("Invalid R_matrix size, using identity matrix");
    }

    back_end_params_.image_opt.panorama_height = pnh_.param<int>("panorama_height", 1);
    back_end_params_.image_opt.panorama_width = pnh_.param<int>("panorama_width", 1);


    LOG(INFO) << "frontend_event_sample_rate: " << front_end_params_.warp_opt.event_sample_rate;


    int panorama_height_ = front_end_params_.image_opt.panorama_height;
    int panorama_width_ = front_end_params_.image_opt.panorama_width;
    





    // New a angular velocity estimator (the front-end runs in the main thread)
    ang_vel_estimator_ = new AngVelEstimator(&nh_);

    // New a pose graph optimizer
    pose_graph_optimizer_ = new PoseGraphOptimizer(&nh_);

    // Initialize the back-end thread and launch
    // pose_graph_optim_ = new std::thread(&PoseGraphOptimizer::Run, pose_graph_optimizer_);

    // Set the pointer to each other
    ang_vel_estimator_->setBackend(pose_graph_optimizer_);
    pose_graph_optimizer_->setFrontend(ang_vel_estimator_);
}

Panorama::~Panorama() {
    // 若后端有独立线程，需在此安全终止
    // if (pose_graph_optimizer_) {
    //     pose_graph_optimizer_->requestStop();
    // }
}

// 预计算每个像素的3D射线方向（共享给前后端）
void Panorama::precomputeBearingVectors() {

    const int width = cam.fullResolution().width;
    const int height = cam.fullResolution().height;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            cv::Point2d rectified_point = cam.rectifyPoint(cv::Point2d(x, y));
            cv::Point3d bearing_vec = cam.projectPixelTo3dRay(rectified_point);
            precomputed_bearing_vectors.emplace_back(bearing_vec);

            // cout << precomputed_bearing_vectors.size() << endl;
        }
    }
}

// 相机内参回调（仅调用一次）
void Panorama::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info) {
    if (!got_camera_info_) {
        ROS_INFO("Loading camera information");
        cam.fromCameraInfo(camera_info);
        got_camera_info_ = true;


        ROS_INFO("Camera info got");
        camera_info_sub_.shutdown();

        // Initialze the front-end
        precomputeBearingVectors();
        ang_vel_estimator_->initialize(&cam, front_end_params_, precomputed_bearing_vectors);//相机参数，前端参数，预计算投影

        // Intialize the back-end
        int camera_width = cam.fullResolution().width;
        int camera_height = cam.fullResolution().height;
        pose_graph_optimizer_->initialize(camera_width, camera_height,
                                          back_end_params_,
                                          &(ang_vel_estimator_->events_),
                                          &precomputed_bearing_vectors);

        ROS_INFO("System initialized with camera info");
    }
}

// 事件数据回调
void Panorama::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
    if (!got_camera_info_) {
        ROS_WARN_THROTTLE(1, "Waiting for camera info...");
        return;
    }

    if (!is_t0_p_set_ && !msg->events.empty()) {
        t0_p = msg->events[0].ts;  // 使用第一个事件的时间戳
        is_t0_p_set_ = true;
        
        // 传递给前端和后端
        ang_vel_estimator_->setInitialTimestamp(t0_p);
        pose_graph_optimizer_->setInitialTimestamp(t0_p);
        
        ROS_INFO_STREAM("Initial timestamp set to: " << t0_p);
    }


    for (auto ev = msg->events.begin(); ev < msg->events.end(); 
        ev += front_end_params_.warp_opt.event_sample_rate) 
    {
        // 前端处理单个事件，返回处理后的数据（如旋转增量）
        ProcessedEventData result = ang_vel_estimator_->processEvent(*ev);
        
        // 将处理结果传给后端
        pose_graph_optimizer_->WarpPano(result);
    }
}

} // namespace panorama