#include "zed_cam_uvc.hpp"


ZedUVC::ZedUVC(const ros::NodeHandle& nh, 
            const ros::NodeHandle& private_nh) : nh_(nh), pnh_(private_nh){

    pnh_.param<std::string>("left_cam_topic", left_topic_name, "/cam0/image_raw");
    pnh_.param<std::string>("right_cam_topic", right_topic_name, "/cam1/image_raw");
    pnh_.param<bool>("publish_raw", pub_raw_, false);
    pnh_.param<double>("frame_rate", frame_rate_, 100);
    pnh_.param<std::vector<int>>("resolution", resolution_, std::vector<int>{672, 376});
    pnh_.param<int>("cam_index", cam_idx_, 0);
    pnh_.param<bool>("is_color", is_color_, false);
    pnh_.param<int>("serial_no", sn_, 10026849);
    quit_ = false;

}
ZedUVC::~ZedUVC(){
    if(grab_thread_){
        grab_thread_->join();
    }
}

bool ZedUVC::init_cap(){
    if(!pub_raw_){
        std::string calibration_file;
        cv::Size2i image_size = cv::Size2i(resolution_[0], resolution_[1]);
        // ZED Calibration
        // Download camera calibration file
        if (downloadCalibrationFile(sn_, calibration_file)) return false;
        cout << "Calibration file found. Loading..." << endl;

        initCalibration(calibration_file, image_size, map_left_x, map_left_y, map_right_x, map_right_y, cameraMatrix_left, cameraMatrix_right);
    }
    cap_.open(cam_idx_);
    if (!cap_.isOpened()){
        std::cerr << "[ERROR] Couldn't open the camera \n";
        std::cerr << "Exiting... \n";
        return false;
    }
    cap_.grab();
    // Set the video resolution (2*Width * Height)
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, resolution_[0]*2);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, resolution_[1]);
    cap_.set(cv::CAP_PROP_FPS, frame_rate_);
    cap_.grab();

    return true;
}

void ZedUVC::start_stream(){
    grab_thread_ = std::unique_ptr<std::thread>(
                    new std::thread(&ZedUVC::grabLoop, this));
    sleep(3);
}

void ZedUVC::grabLoop(){
    image_transport::ImageTransport it(pnh_);
    image_transport::Publisher img_pub_left = it.advertise(left_topic_name, 10);
    image_transport::Publisher img_pub_right = it.advertise(right_topic_name, 10);
    ros::Time grab_time;
    cv_bridge::CvImage img_msg;
    int seq = 0;
    Mat frame, left_raw, left_rect, right_raw, right_rect;
    
    if(!is_color_)
        img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    else
        img_msg.encoding = sensor_msgs::image_encodings::RGB8;
    
    while(ros::ok() && !quit_)
    {
        if(cap_.grab()){
            grab_time = ros::Time::now();
            if(cap_.retrieve(frame)){
                left_raw = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
                right_raw = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
                if (!is_color_){
                    cvtColor(left_raw, left_raw, COLOR_BGR2GRAY);
                    cvtColor(right_raw, right_raw, COLOR_BGR2GRAY);
                }
                else{
                    cvtColor(left_raw, left_raw, COLOR_BGR2RGB);
                    cvtColor(right_raw, right_raw, COLOR_BGR2RGB);
                }
                if(!pub_raw_){
                    remap(left_raw, left_rect, map_left_x, map_left_y, INTER_LINEAR);
                    remap(right_raw, right_rect, map_right_x, map_right_y, INTER_LINEAR);
                }
                img_msg.header.seq = seq++;
                img_msg.header.stamp = grab_time;
                img_msg.image = pub_raw_? left_raw : left_rect;
                img_msg.header.frame_id = "cam_left";

                img_pub_left.publish(img_msg.toImageMsg());

                img_msg.image = pub_raw_? right_raw : right_rect;
                img_msg.header.frame_id = "cam_right";

                img_pub_right.publish(img_msg.toImageMsg());
            }
            else{
                std::cerr << "[WARN] Frame missed. \n";
            }
        }
        else{
            std::cerr << "[ERROR] Camera Disconnected. Exiting... \n";
            quit_ = true;
        }
    }
    cap_.release();

}