#include <pcl/filters/voxel_grid.h>

#include <ros/init.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>

ros::Publisher stitchedCloudPub;
pcl::PCLPointCloud2::Ptr stitchedCloud;
std::optional<tf2_ros::Buffer> tfBuffer;
std::optional<tf2_ros::TransformListener> tfListener;

auto point_cloud_callback(sensor_msgs::PointCloud2::ConstPtr const& cloudInCamRos) -> void {;
    sensor_msgs::PointCloud2 cloudInMapRos;
    pcl_ros::transformPointCloud("map", *cloudInCamRos, cloudInMapRos, *tfBuffer);

    auto cloudInMap = boost::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(cloudInMapRos, *cloudInMap);

    if (stitchedCloud) {
        *stitchedCloud += *cloudInMap;
    } else {
        stitchedCloud = cloudInMap;
    }

    auto cloudInMapStithced = boost::make_shared<pcl::PCLPointCloud2>();

    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud(stitchedCloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloudInMapStithced);

    sensor_msgs::PointCloud2 stitchedCloudInMapRos;
    fromPCL(*cloudInMapStithced, stitchedCloudInMapRos);
    stitchedCloudInMapRos.header.frame_id = "map";
    stitchedCloudInMapRos.header.stamp = ros::Time::now();
    stitchedCloudPub.publish(stitchedCloudInMapRos);
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "point_cloud_stitcher");
    ros::NodeHandle nh;

    tfBuffer.emplace();
    tfListener.emplace(*tfBuffer);

    auto cloudSubscriber = nh.subscribe("camera/left/points", 1, point_cloud_callback);
    stitchedCloudPub = nh.advertise<sensor_msgs::PointCloud2>("stitched_point_cloud", 1);

    ros::spin();

    return EXIT_SUCCESS;
}
