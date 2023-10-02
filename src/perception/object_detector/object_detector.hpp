#include "pch.hpp"

namespace mrover {

    class ObjectDetectorNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh;

        LoopProfiler mProfiler{"Object Detector"};

        void onInit() override;

    public:
        ObjectDetectorNodelet() = default;

        ~ObjectDetectorNodelet() override = default;

        void imageCallback(sensor_msgs::ImageConstPtr const& msg);

        bool enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    };

} // namespace mrover
