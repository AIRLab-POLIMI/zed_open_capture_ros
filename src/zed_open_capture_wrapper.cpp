#include "zed_open_capture_ros/zed_open_capture_wrapper.hpp"

ZedOpenCaptureWrapper::ZedOpenCaptureWrapper() : cap_(nullptr), nh("~") {
    int resolution, fps;
    nh.param("resolution", resolution, 1);
    video_params_.res = (sl_oc::video::RESOLUTION) resolution;
    nh.param("frame_rate", fps, 30);
    video_params_.fps = (sl_oc::video::FPS) fps;
    nh.param("config_file_location", config_file_location_, std::string(""));
    nh.param("left_frame_id", left_frame_id_, std::string("left_camera_frame"));
    nh.param("right_frame_id", right_frame_id_, std::string("right_camera_frame"));
    std::string camera_namespace;
    nh.param("camera_namespace", camera_namespace, std::string("zed"));
    nh.param("use_zed_config", use_zed_config_, true);
    nh.param("device_id", device_id_, -1);
    nh.param("serial", serial_, 0);
    capFramerate();
    ROS_INFO_STREAM("Parameters loaded");

    ros::NodeHandle nhh;

    image_transport::ImageTransport it(nhh);
    left_image_pub_ = it.advertise(camera_namespace + "/left/image", 1);
    right_image_pub_ = it.advertise(camera_namespace + "/right/image", 1);

    left_cam_info_pub_ = nhh.advertise<sensor_msgs::CameraInfo>(camera_namespace + "/left/camera_info", 1);
    right_cam_info_pub_ = nhh.advertise<sensor_msgs::CameraInfo>(camera_namespace + "/right/camera_info", 1);
    ROS_INFO_STREAM("Topic initialised");
}

bool ZedOpenCaptureWrapper::initializeVideoCapture() {
    cap_ = new sl_oc::video::VideoCapture(video_params_);
    if (serial_ == 0) {
        if (!cap_->initializeVideo(device_id_)) {
            ROS_ERROR_STREAM("Failed to open camera video capture");
            return false;
        }
    } else {
        bool success = false;
        for (uint8_t id = 0; id < 64; id++) {
            if (cap_->initializeVideo(id)) {
                if (cap_->getSerialNumber() == serial_) {
                    success = true;
                    ROS_INFO_STREAM("Camera with SN"<<serial_<<" found");
                    break;
                }
                else
                {
                    delete cap_;
                    cap_ = new sl_oc::video::VideoCapture(video_params_);
                }
            }
        }
        if (!success) {
            ROS_ERROR_STREAM("Failed to open camera video capture with the specified serial");
            return false;
        }
    }

    ROS_INFO_STREAM("Video capture initialised");

    if (use_zed_config_ && !config_file_location_.empty()) {
        if (serial_ == 0) {
            getZedCameraInfo(config_file_location_, cap_->getSerialNumber());
        } else {
            getZedCameraInfo(config_file_location_, serial_);
        }

        ROS_INFO_STREAM("Configuration loaded");
    }
    return true;
}

void ZedOpenCaptureWrapper::capFramerate() {
    switch (video_params_.res) {
        case sl_oc::video::RESOLUTION::HD2K:
            if (video_params_.fps > sl_oc::video::FPS::FPS_15) {
                video_params_.fps = sl_oc::video::FPS::FPS_15;
                ROS_WARN_STREAM("FPS capped to 15 to maintain resolution");
            }
            break;
        case sl_oc::video::RESOLUTION::HD1080:
            if (video_params_.fps > sl_oc::video::FPS::FPS_30) {
                video_params_.fps = sl_oc::video::FPS::FPS_30;
                ROS_WARN_STREAM("FPS capped to 30 to maintain resolution");
            }
            break;
        case sl_oc::video::RESOLUTION::HD720:
            if (video_params_.fps > sl_oc::video::FPS::FPS_60) {
                video_params_.fps = sl_oc::video::FPS::FPS_60;
                ROS_WARN_STREAM("FPS capped to 60 to maintain resolution");
            }
            break;
        case sl_oc::video::RESOLUTION::VGA:
            if (video_params_.fps > sl_oc::video::FPS::FPS_100) {
                video_params_.fps = sl_oc::video::FPS::FPS_100;
                ROS_WARN_STREAM("FPS capped to 100 to maintain resolution");
            }
            break;
        case sl_oc::video::RESOLUTION::LAST:
            throw std::runtime_error("Failed to get resolution");
    }
}

void ZedOpenCaptureWrapper::getZedCameraInfo(const std::string &config_file, int serial) {
    boost::property_tree::ptree pt;
    std::string serial_str = std::to_string(serial);
    std::string conf_file = config_file +
                            "/SN" + serial_str.substr(serial_str.size() - 4) +
                            ".conf";
    ROS_INFO_STREAM("Loaded "<<conf_file);
    boost::property_tree::ini_parser::read_ini(conf_file, pt);
    std::string left_str = "LEFT_CAM_";
    std::string right_str = "RIGHT_CAM_";
    std::string reso_str;

    unsigned int width_, height_;

    switch (video_params_.res) {
        case sl_oc::video::RESOLUTION::HD2K:
            reso_str = "2K";
            width_ = 4416;
            height_ = 1242;
            break;
        case sl_oc::video::RESOLUTION::HD1080:
            reso_str = "FHD";
            width_ = 3840;
            height_ = 1080;
            break;
        case sl_oc::video::RESOLUTION::HD720:
            reso_str = "HD";
            width_ = 2560;
            height_ = 720;
            break;
        case sl_oc::video::RESOLUTION::VGA:
            reso_str = "VGA";
            width_ = 1344;
            height_ = 376;
            break;
        case sl_oc::video::RESOLUTION::LAST:
            throw std::runtime_error("Failed to get resolution");
    }

    // left value
    auto l_cx = pt.get<double>(left_str + reso_str + ".cx");
    auto l_cy = pt.get<double>(left_str + reso_str + ".cy");
    auto l_fx = pt.get<double>(left_str + reso_str + ".fx");
    auto l_fy = pt.get<double>(left_str + reso_str + ".fy");
    auto l_k1 = pt.get<double>(left_str + reso_str + ".k1");
    auto l_k2 = pt.get<double>(left_str + reso_str + ".k2");
    // right value
    auto r_cx = pt.get<double>(right_str + reso_str + ".cx");
    auto r_cy = pt.get<double>(right_str + reso_str + ".cy");
    auto r_fx = pt.get<double>(right_str + reso_str + ".fx");
    auto r_fy = pt.get<double>(right_str + reso_str + ".fy");
    auto r_k1 = pt.get<double>(right_str + reso_str + ".k1");
    auto r_k2 = pt.get<double>(right_str + reso_str + ".k2");

    // get baseline and convert mm to m
    boost::optional<double> baselineCheck;

    // some config files have "Baseline" instead of "BaseLine", check accordingly...
    baselineCheck = pt.get_optional<double>("STEREO.BaseLine");
    if (!baselineCheck) {
        baselineCheck = pt.get<double>("STEREO.Baseline");

        if (!baselineCheck) {
            throw std::runtime_error("baseline parameter not found");
        }
    }

    double baseline = baselineCheck.get() * 0.001;

    // get Rx and Rz
    auto rx = pt.get<double>("STEREO.RX_" + reso_str);
    auto rz = pt.get<double>("STEREO.RZ_" + reso_str);
    auto ry = pt.get<double>("STEREO.CV_" + reso_str);

    // assume zeros, maybe not right
    double p1 = 0, p2 = 0, k3 = 0;

    left_camera_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_camera_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // distortion parameters
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    left_camera_info_.D.resize(5);
    left_camera_info_.D[0] = l_k1;
    left_camera_info_.D[1] = l_k2;
    left_camera_info_.D[2] = k3;
    left_camera_info_.D[3] = p1;
    left_camera_info_.D[4] = p2;

    right_camera_info_.D.resize(5);
    right_camera_info_.D[0] = r_k1;
    right_camera_info_.D[1] = r_k2;
    right_camera_info_.D[2] = k3;
    right_camera_info_.D[3] = p1;
    right_camera_info_.D[4] = p2;

    // Intrinsic camera matrix
    // 	[fx  0 cx]
    // K =  [ 0 fy cy]
    //	[ 0  0  1]
    left_camera_info_.K.fill(0.0);
    left_camera_info_.K[0] = l_fx;
    left_camera_info_.K[2] = l_cx;
    left_camera_info_.K[4] = l_fy;
    left_camera_info_.K[5] = l_cy;
    left_camera_info_.K[8] = 1.0;

    right_camera_info_.K.fill(0.0);
    right_camera_info_.K[0] = r_fx;
    right_camera_info_.K[2] = r_cx;
    right_camera_info_.K[4] = r_fy;
    right_camera_info_.K[5] = r_cy;
    right_camera_info_.K[8] = 1.0;

    // rectification matrix
    // Rl = R_rect, R_r = R * R_rect
    // since R is identity, Rl = Rr;
    left_camera_info_.R.fill(0.0);
    right_camera_info_.R.fill(0.0);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    cv::Mat rmat(3, 3, CV_64F);
    cv::Rodrigues(rvec, rmat);
    int id = 0;
    cv::MatIterator_<double> it, end;
    for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++) {
        left_camera_info_.R[id] = *it;
        right_camera_info_.R[id] = *it;
    }

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    left_camera_info_.P.fill(0.0);
    left_camera_info_.P[0] = l_fx;
    left_camera_info_.P[2] = l_cx;
    left_camera_info_.P[5] = l_fy;
    left_camera_info_.P[6] = l_cy;
    left_camera_info_.P[10] = 1.0;

    right_camera_info_.P.fill(0.0);
    right_camera_info_.P[0] = r_fx;
    right_camera_info_.P[2] = r_cx;
    right_camera_info_.P[3] = (-1 * l_fx * baseline);
    right_camera_info_.P[5] = r_fy;
    right_camera_info_.P[6] = r_cy;
    right_camera_info_.P[10] = 1.0;

    left_camera_info_.width = right_camera_info_.width = width_;
    left_camera_info_.height = right_camera_info_.height = height_;


    left_camera_info_.header.frame_id = left_frame_id_;
    right_camera_info_.header.frame_id = right_frame_id_;
}

void ZedOpenCaptureWrapper::spin() {
    if (cap_ == nullptr) {
        ROS_ERROR_STREAM("Video capture not initialised");
        return;
    }
    ros::Rate loop_rate((int) video_params_.fps * 1.5);
    while (nh.ok()) {
        const sl_oc::video::Frame frame = cap_->getLastFrame();
        cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR;
        cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

        cv::Mat left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
        cv::Mat right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

        std_msgs::Header header;
        auto now = ros::Time::now();
        header.stamp = now;

        sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(header, "bgr8", left_raw).toImageMsg();
        left_image_pub_.publish(msg_left);
        left_camera_info_.header.stamp = now;
        left_cam_info_pub_.publish(left_camera_info_);

        sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(header, "bgr8", right_raw).toImageMsg();
        right_image_pub_.publish(msg_right);
        right_camera_info_.header.stamp = now;
        right_cam_info_pub_.publish(right_camera_info_);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
