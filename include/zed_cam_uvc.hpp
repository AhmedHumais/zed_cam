#pragma once

#include <thread>
#include <string>
#include <atomic>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include "calibration.hpp"

using namespace cv;
using namespace std;

class ZedUVC {

public:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    atomic<bool> quit_;

    ZedUVC(const ros::NodeHandle& nh,
          const ros::NodeHandle& private_nh);
    virtual ~ZedUVC();
    bool init_cap();
    void start_stream();
    void fillCamInfo(Mat &cameraMatrix_left, Mat &cameraMatrix_right, cv::Size2i image_size, sensor_msgs::CameraInfoPtr leftCamInfoMsg,
                     sensor_msgs::CameraInfoPtr rightCamInfoMsg);
    
private:
    int cam_idx_;
    bool is_color_, pub_raw_;
    int sn_;
    double frame_rate_;
    std::vector<int> resolution_;
    string left_topic_name, right_topic_name;

    cv::Mat map_left_x, map_left_y;
    cv::Mat map_right_x, map_right_y;
    cv::Mat cameraMatrix_left, cameraMatrix_right;
    sensor_msgs::CameraInfoPtr leftInfoMsg, rightInfoMsg;
    cv::VideoCapture cap_;
    std::unique_ptr<std::thread> grab_thread_;

    void grabLoop();
};
