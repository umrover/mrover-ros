#include "pch.hpp"

namespace mrover {

    class TagDetector : public nodelet::Nodelet {
    protected:
        ros::NodeHandle mNh, mPnh;

        // image subscriber
        ros::Subscriber mImgSub;
        ros::Publisher mImgPub;

        // publishing flags
        bool mEnableDetections = true;
        bool mPublishImages{}; // If set, we publish the images with the tags drawn on top

        //Raw Tag Data from CV::ARUCO
        std::vector<std::vector<cv::Point2f>> mImmediateCorners;
        std::vector<int> mImmediateIds;

        // Message header information
        std::optional<size_t> mPrevDetectedCount; // Log spam prevention
        dynamic_reconfigure::Server<DetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<DetectorParamsConfig>::CallbackType mCallbackType;
        int dictionaryNumber;

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;
        ros::ServiceServer mServiceEnableDetections;

    public: 
        auto onInit() -> void override;

        virtual void specificOnInit() = 0;

        auto configCallback(DetectorParamsConfig& config, uint32_t level) -> void;

        auto enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) -> bool;

    };
}