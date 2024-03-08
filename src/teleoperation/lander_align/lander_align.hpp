#pragma once

#include "pch.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <optional>
#include <ostream>
#include <tuple>

namespace mrover {

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
        double const mLinearP = 0.1;
        ros::NodeHandle mNh, mPnh;

        ros::Publisher mDebugVectorPub;

        ros::Publisher mDebugPCPub;


        ros::Publisher mTwistPub;


        ros::Subscriber mVectorSub;

        RTRSTATE mLoopState;

        //**
        double mBestOffset;

        std::optional<Eigen::Vector3d> mBestLocationInZED;
        std::optional<Eigen::Vector3d> mBestLocationInWorld;

        std::optional<Eigen::Vector3d> mBestOffsetInZED;
        std::optional<Eigen::Vector3d> mBestOffsetInWorld;

        std::optional<Eigen::Vector3d> mBestNormalInZED;
        std::optional<Eigen::Vector3d> mBestNormalInWorld;
        //**

        //TF Member Variables
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrameId;
        std::string mMapFrameId;

		//Ransac Params
        float mZThreshold;
        int mLeastSamplingDistribution = 10;
        std::vector<Point const*> mFilteredPoints;

        auto onInit() -> void override;

        // deprecated/not needed anymore
        // auto downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr;

        void LanderCallback(sensor_msgs::PointCloud2Ptr const& cloud);

        void filterNormals(sensor_msgs::PointCloud2Ptr const& cloud);

        void ransac(double distanceThreshold, int minInliers, int epochs);

        void sendTwist();

		void uploadPC(int numInliers, double distanceThreshold);

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
