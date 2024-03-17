#pragma once

#include "mrover/LanderAlignAction.h"
#include "mrover/LanderAlignActionGoal.h"
#include "pch.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <optional>
#include <ostream>
#include <tuple>

namespace mrover {

    typedef actionlib::SimpleActionServer<mrover::LanderAlignAction> Server;

    enum RTRSTATE {
        turn1 = 0,
        drive = 1,
        turn2 = 2,
        done = 3
    };

    constexpr char RTRSTRINGS[4][6] = {
                                "turn1",
                                "drive",
                                "turn2",
                                "done "
    };
    

    class LanderAlignNodelet : public nodelet::Nodelet {


        //PID CONSTANTS
        double const mAngleP = 1;
        double const mLinearP = 0.3;
        ros::NodeHandle mNh, mPnh;

        Server mActionServer = Server(mNh, "LanderAlignAction", [&](const mrover::LanderAlignActionGoalConstPtr& goal){ActionServerCallBack(goal);}, false);


        ros::Publisher mDebugVectorPub;

        ros::Publisher mDebugPCPub;

        ros::Publisher mTwistPub;

        ros::Subscriber mVectorSub;

        RTRSTATE mLoopState;

        //**
        double mBestOffset;

        double mPlaneOffsetScalar;

        std::optional<Eigen::Vector3d> mPlaneLocationInZEDVector;
        std::optional<Eigen::Vector3d> mPlaneLocationInWorldVector;

        std::optional<Eigen::Vector3d> mOffsetLocationInZEDVector;
        std::optional<Eigen::Vector3d> mOffsetLocationInWorldVector;

        std::optional<Eigen::Vector3d> mNormalInZEDVector;
        std::optional<Eigen::Vector3d> mNormalInWorldVector;

        manif::SE3d mOffsetLocationInWorldSE3d;
        manif::SE3d mPlaneLocationInWorldSE3d;
        //**

        //TF Member Variables
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrameId;
        std::string mMapFrameId;

		//Ransac Params
        double mZThreshold;
        double mXThreshold;
        int mLeastSamplingDistribution = 10;
        std::vector<Point const*> mFilteredPoints;

        auto onInit() -> void override;

        void LanderCallback(sensor_msgs::PointCloud2Ptr const& cloud);
        
        void ActionServerCallBack(LanderAlignActionGoalConstPtr const& actionRequest);

        void filterNormals(sensor_msgs::PointCloud2Ptr const& cloud);

        void ransac(double distanceThreshold, int minInliers, int epochs, double offsetFactor);

        void sendTwist();

		void uploadPC(int numInliers, double distanceThreshold);

        void calcMotion(double desiredVelocity, double desiredOmega);

        static auto calcAngleWithWorldX(Eigen::Vector3d xHeading) -> double;

        class PID {
        private:
            float const Angle_P;
            float const Linear_P;


        public:
            PID(float angle_P, float linear_P);

            // auto calculate(Eigen::Vector3d input, Eigen::Vector3d target) -> float;

            [[nodiscard]] auto rotate_speed(double theta) const -> double;


            auto find_angle(Eigen::Vector3d const& current, Eigen::Vector3d const& target) -> float;


            auto find_distance(Eigen::Vector3d const& current, Eigen::Vector3d const& target) -> double;

            auto drive_speed(float) -> float;
        };
    };

} // namespace mrover
