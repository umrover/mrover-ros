//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */
#pragma once

#include <map>
#include <array>
#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>

// ROS
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Custom Callback Queue
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>

namespace gazebo {

    class DiffDrivePlugin6W : public ModelPlugin {

    public:
        DiffDrivePlugin6W();

        ~DiffDrivePlugin6W() override;

    protected:
        void Load(physics::ModelPtr mModel, sdf::ElementPtr mSdf) override;

        void Reset() override;

        virtual void Update();

    private:
        void publish_odometry();

        void GetPositionCmd();

        physics::LinkPtr mBodyLink;
        physics::WorldPtr mWorld;
        std::array<physics::JointPtr, 6> mJoints;

        double mWheelSeparation{};
        double mWheelDiameter{};
        double mTorque{};
        std::array<double, 2> mWheelSpeed{};

        // Simulation time of the last update
        common::Time mPreviousUpdateTime;

        bool mEnableMotors{};
        double odomPose[3]{};
        double odomVel[3]{};

        // ROS STUFF
        std::unique_ptr<ros::NodeHandle> mNode;
        ros::Publisher mPublisher;
        ros::Subscriber mSubscriber;
        std::unique_ptr<tf::TransformBroadcaster> mTfBroadcaster{};
        nav_msgs::Odometry mOdometry;
        std::string mTfPrefix;

        std::mutex mLock;

        std::string mNamespace;
        std::string mVelocityCommandTopic;
        std::string mBodyLinkName;

        // Custom Callback Queue
        ros::CallbackQueue mQueue;
        boost::thread mCallbackQueueThread;

        void QueueThread();

        // DiffDrive stuff
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twistCommand);

        double mForwardVelocity{};
        double mPitch{};
        bool mIsAlive{};

        // Pointer to the update event connection
        event::ConnectionPtr mUpdateConnection;
    };

} // namespace gazebo
