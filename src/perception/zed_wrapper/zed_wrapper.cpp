#include <ros/init.h>
#include <sl/Camera.hpp>

#include <tag_detection.hpp>

PointCloudPtr g_pointCloud = std::make_shared<PointCloud>();
sl::Mat g_imageMat;
sl::Mat g_pointCloudMat;
sl::Mat g_pointCloudNormalMat;
Detections g_detections;

sensor_msgs::PointCloud2 g_pointCloudMsg;

int main(int argc, char** argv) {
    try {
        // TODO: make try to repeatedly open the camera

        ros::init(argc, argv, "zed_wrapper");

        sl::Camera zed;
        sl::InitParameters init_parameters;
        init_parameters.camera_resolution = sl::RESOLUTION::HD720;
        init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
        init_parameters.coordinate_units = sl::UNIT::METER;
        init_parameters.sdk_verbose = true;
        init_parameters.camera_fps = 60;
        init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

        //        sl::Resolution image_size{1280, 720};
        sl::Resolution image_size{640, 480};
        g_pointCloud->resize(image_size.area());

        if (zed.open(init_parameters) != sl::ERROR_CODE::SUCCESS) {
            throw std::runtime_error("ZED failed to open");
        }

        ros::NodeHandle node;
        ros::Publisher pointCloudPublisher = node.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);

        while (ros::ok()) {
            sl::RuntimeParameters runtime_parameters;

            if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) {
                zed.retrieveImage(g_imageMat, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
                zed.retrieveMeasure(g_pointCloudMat, sl::MEASURE::XYZ, sl::MEM::CPU, image_size);
                zed.retrieveMeasure(g_pointCloudNormalMat, sl::MEASURE::NORMALS, sl::MEM::CPU, image_size);

                auto imagePtr = g_imageMat.getPtr<sl::uchar4>();
                auto* pointCloudPtr = g_pointCloudMat.getPtr<sl::float4>();
                auto* pointCloudNormalPtr = g_pointCloudNormalMat.getPtr<sl::float4>();

                size_t index = 0;
                for (auto& point: *g_pointCloud) {
                    point.x = pointCloudPtr[index].x;
                    point.y = pointCloudPtr[index].y;
                    point.z = pointCloudPtr[index].z;
                    point.normal_x = pointCloudNormalPtr[index].x;
                    point.normal_y = pointCloudNormalPtr[index].y;
                    point.normal_z = pointCloudNormalPtr[index].z;
                    // RGB -> BGR
                    point.r = imagePtr[index].b;
                    point.g = imagePtr[index].g;
                    point.b = imagePtr[index].r;
                    index += 1;
                }

                pcl::toROSMsg(*g_pointCloud, g_pointCloudMsg);
                g_pointCloudMsg.header.frame_id = "map";
                g_pointCloudMsg.header.stamp = ros::Time::now();
                pointCloudPublisher.publish(g_pointCloudMsg);

                detectTags(g_pointCloud, g_detections);


            } else {
                throw std::runtime_error("ZED failed to grab");
            }

            ros::spinOnce();
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