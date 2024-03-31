#pragma once

#include "pch.hpp"

namespace mrover {

    using Server = actionlib::SimpleActionServer<LanderAlignAction>;

    enum struct RTRSTATE {
        turn1 = 0,
        drive = 1,
        turn2 = 2,
        done = 3,
    };

    constexpr char RTRSTRINGS[4][6] = {
            "turn1",
            "drive",
            "turn2",
            "done ",
    };

    class LanderAlignNodelet final : public nodelet::Nodelet {
    private:
		//ROS VARS
        ros::NodeHandle mNh, mPnh;

		//RANSAC VARS
        double mBestOffset{};

        double mPlaneOffsetScalar{};

        std::optional<Eigen::Vector3d> mPlaneLocationInZEDVector;
        std::optional<Eigen::Vector3d> mPlaneLocationInWorldVector;

        std::optional<Eigen::Vector3d> mOffsetLocationInZEDVector;
        std::optional<Eigen::Vector3d> mOffsetLocationInWorldVector;

        std::optional<Eigen::Vector3d> mNormalInZEDVector;
        std::optional<Eigen::Vector3d> mNormalInWorldVector;

        SE3d mOffsetLocationInWorldSE3d;
        SE3d mPlaneLocationInWorldSE3d;

        double mZThreshold{};
        double mXThreshold{};
        int mLeastSamplingDistribution = 10;

        ros::Publisher mDebugPCPub;

        std::vector<Point const*> mFilteredPoints;

        std::vector<Eigen::Vector3d> mPathPoints;

		//RTR RTR VARS
        RTRSTATE mLoopState;

        ros::Publisher mTwistPub;

        //PID CONSTANTS
        double const mAngleP = 1;
        double const mAngleFloor = 0.05;
        double const mLinearP = 0.3;
        
        ros::Publisher mDebugVectorPub;

        //Action Server Variables
        sensor_msgs::PointCloud2ConstPtr mCloud;

        std::optional<Server> mActionServer;

        //TF Member Variables
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrameId;
        std::string mMapFrameId;

        //Ransac Params

        auto onInit() -> void override;
        
        void filterNormals(sensor_msgs::PointCloud2ConstPtr const& cloud);

        void ransac(double distanceThreshold, int minInliers, int epochs);

        void sendTwist();

        void uploadPC(int numInliers, double distanceThreshold);

        void calcMotion(double desiredVelocity, double desiredOmega);

        static auto calcAngleWithWorldX(Eigen::Vector3d xHeading) -> double;
    
    public:
        void ActionServerCallBack(LanderAlignGoalConstPtr const goal);
    };

} // namespace mrover
