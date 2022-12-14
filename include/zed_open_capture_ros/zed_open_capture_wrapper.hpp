#ifndef SRC_ZED_OPEN_CAPTURE_WRAPPER_HPP
#define SRC_ZED_OPEN_CAPTURE_WRAPPER_HPP

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "boost/property_tree/ini_parser.hpp"
#include "sensor_msgs/distortion_models.h"
#include "zed_open_capture_ros/videocapture.hpp"


class ZedOpenCaptureWrapper {
public:
    ros::NodeHandle nh;

    ZedOpenCaptureWrapper();
    bool initializeVideoCapture();
    void spin();

private:
    void getZedCameraInfo(const std::string& config_file, int serial);
    void capFramerate();

    sl_oc::video::VideoCapture *cap_;
    sl_oc::video::VideoParams video_params_;
    int device_id_, serial_;
    bool use_zed_config_;
    std::string config_file_location_;
    std::string left_frame_id_;
    std::string right_frame_id_;
    sensor_msgs::CameraInfo left_camera_info_;
    sensor_msgs::CameraInfo right_camera_info_;
    ros::Publisher left_cam_info_pub_;
    ros::Publisher right_cam_info_pub_;
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
};

#endif //SRC_ZED_OPEN_CAPTURE_WRAPPER_HPP
