#pragma once

#include "pch.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <tuple>

namespace mrover {

    enum RTRSTATE {
        turn1,
        drive,
        turn2,
        done
    };

    class LanderAlignNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        ros::Publisher mDebugVectorPub;

        ros::Publisher mTwistPub;


        ros::Subscriber mVectorSub;

        RTRSTATE mLoopState;

        //**
        float mBestOffset;

        Eigen::Vector3f mBestCenter;
        //**

        //TF Member Variables
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrameId;
        std::string mMapFrameId;

        float mThreshold;

        std::vector<Point const*> mFilteredPoints;

        auto onInit() -> void override;

        // deprecated/not needed anymore
        // auto downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr;

        void LanderCallback(sensor_msgs::PointCloud2Ptr const& cloud);

        void filterNormals(sensor_msgs::PointCloud2Ptr const& cloud);

        auto ransac(std::vector<Point const*> const& points, float distanceThreshold, int minInliers, int epochs) -> std::optional<Eigen::Vector3f>;

        void sendTwist(Eigen::Vector3f const& planeNormal, Eigen::Vector3f const& mBestCenter, Eigen::Vector3f const& offset);

        class PID {
        private:
            float const Angle_P;
            float const Linear_P;


        public:
            PID(float angle_P, float linear_P);

            // auto calculate(Eigen::Vector3f input, Eigen::Vector3f target) -> float;

            [[nodiscard]] auto rotate_speed(float theta) const -> float;


            auto find_angle(Eigen::Vector3f current, Eigen::Vector3f target) -> float;


            auto find_distance(Eigen::Vector3f current, Eigen::Vector3f target) -> float;

            auto drive_speed(float) -> float;
        };
    };

} // namespace mrover