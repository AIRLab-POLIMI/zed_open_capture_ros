#include "zed_open_capture_ros/zed_open_capture_wrapper.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_open_capture_node");
    ZedOpenCaptureWrapper z;
    if(!z.initializeVideoCapture()) {
        return EXIT_FAILURE;
    }
    z.spin();
    return EXIT_SUCCESS;
}