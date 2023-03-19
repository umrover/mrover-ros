// THIS IS LIKE ANY OTHER ROS NODE

#include <ros/init.h>
#include <sl/Camera.hpp>

#include <tag_detection.hpp>

int main(int argc, char** argv) {
    try {
        // TODO: make try to repeatedly open the camera

        ros::init(argc, argv, "zed_wrapper");

        sl::Camera zed;
        sl::InitParameters init_parameters;

        sl::ERROR_CODE erorr = zed.open(init_parameters);
        if (erorr != sl::ERROR_CODE::SUCCESS) {
            throw std::runtime_error("ZED failed to open");
        }

        // TODO: retrieveMeasure to get point cloud and image
        // TODO: fuse the two
        // TODO: call shared library function "detectTags"
        // TODO: publish tag positions from that call to tf tree

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}