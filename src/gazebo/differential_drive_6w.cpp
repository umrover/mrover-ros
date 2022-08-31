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

/**
 * Based on diffdrive_plugin by Nathan Koenig, Andrew Howard and Daniel Hewlett
 * Edited for MRover to publish odom topic
 */

#include "differential_drive_6w.hpp"

#include <algorithm>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo {

    constexpr auto VELOCITY_COMMAND_TOPIC = "cmd_vel";

    enum {
        FRONT_LEFT,
        FRONT_RIGHT,
        MID_LEFT,
        MID_RIGHT,
        REAR_LEFT,
        REAR_RIGHT,
        NUM_WHEELS
    };

    // Constructor
    DiffDrivePlugin6W::DiffDrivePlugin6W() = default;

    // Destructor
    DiffDrivePlugin6W::~DiffDrivePlugin6W() {
        mUpdateConnection.reset();
        mNode->shutdown();
        mCallbackQueueThread.join();
    }

    // Load the controller
    void DiffDrivePlugin6W::Load(physics::ModelPtr mModel, sdf::ElementPtr mSdf) {
        mWorld = mModel->GetWorld();

        // default parameters
        mNamespace.clear();
        mVelocityCommandTopic = VELOCITY_COMMAND_TOPIC;
        mWheelSeparation = 0.34;
        mWheelDiameter = 0.15;
        mTorque = 10.0;

        // load parameters
        if (mSdf->HasElement("robotNamespace"))
            mNamespace = mSdf->GetElement("robotNamespace")->GetValue()->GetAsString();

        if (mSdf->HasElement("topicName"))
            mVelocityCommandTopic = mSdf->GetElement("topicName")->GetValue()->GetAsString();

        if (mSdf->HasElement("bodyName")) {
            mBodyLinkName = mSdf->GetElement("bodyName")->GetValue()->GetAsString();
            mBodyLink = mModel->GetLink(mBodyLinkName);
        } else {
            mBodyLink = mModel->GetLink();
            mBodyLinkName = mBodyLink->GetName();
        }

        // assert that the body by mBodyLinkName exists
        if (!mBodyLink) {
            ROS_FATAL("DiffDrivePlugin6W plugin error: bodyName: %s does not exist\n", mBodyLinkName.c_str());
            return;
        }

        if (mSdf->HasElement("frontLeftJoint"))
            mJoints[FRONT_LEFT] = mModel->GetJoint(mSdf->GetElement("frontLeftJoint")->GetValue()->GetAsString());
        if (mSdf->HasElement("frontRightJoint"))
            mJoints[FRONT_RIGHT] = mModel->GetJoint(mSdf->GetElement("frontRightJoint")->GetValue()->GetAsString());
        if (mSdf->HasElement("midLeftJoint"))
            mJoints[MID_LEFT] = mModel->GetJoint(mSdf->GetElement("midLeftJoint")->GetValue()->GetAsString());
        if (mSdf->HasElement("midRightJoint"))
            mJoints[MID_RIGHT] = mModel->GetJoint(mSdf->GetElement("midRightJoint")->GetValue()->GetAsString());
        if (mSdf->HasElement("rearLeftJoint"))
            mJoints[REAR_LEFT] = mModel->GetJoint(mSdf->GetElement("rearLeftJoint")->GetValue()->GetAsString());
        if (mSdf->HasElement("rearRightJoint"))
            mJoints[REAR_RIGHT] = mModel->GetJoint(mSdf->GetElement("rearRightJoint")->GetValue()->GetAsString());

        if (!mJoints[FRONT_LEFT]) ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get front left joint");
        if (!mJoints[FRONT_RIGHT]) ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get front right joint");
        if (!mJoints[MID_LEFT]) ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get mid left joint");
        if (!mJoints[MID_RIGHT]) ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get mid right joint");
        if (!mJoints[REAR_LEFT]) ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get rear left joint");
        if (!mJoints[REAR_RIGHT]) ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get rear right joint");

        if (mSdf->HasElement("wheelSeparation"))
            mSdf->GetElement("wheelSeparation")->GetValue()->Get(mWheelSeparation);

        if (mSdf->HasElement("wheelDiameter"))
            mSdf->GetElement("wheelDiameter")->GetValue()->Get(mWheelDiameter);

        if (mSdf->HasElement("torque"))
            mSdf->GetElement("torque")->GetValue()->Get(mTorque);

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        mNode = std::make_unique<ros::NodeHandle>(mNamespace);

        mTfPrefix = tf::getPrefixParam(*mNode);
        mTfBroadcaster = std::make_unique<tf::TransformBroadcaster>();

        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
        ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
                mVelocityCommandTopic, 1,

                boost::bind(&DiffDrivePlugin6W::cmdVelCallback, this, _1),
                ros::VoidPtr(), &mQueue);
        mSubscriber = mNode->subscribe(so);
        mPublisher = mNode->advertise<nav_msgs::Odometry>("odom", 1);

        // TODO(amg): disabled due to weirdness where the thread would die and callbacks stop
        //mCallbackQueueThread = boost::thread(boost::bind(&DiffDrivePlugin6W::QueueThread, this));

        Reset();

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        mUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DiffDrivePlugin6W::Update, this));
    }

    // Initialize the controller
    void DiffDrivePlugin6W::Reset() {
        mEnableMotors = true;

        for (double& ws: mWheelSpeed) {
            ws = 0;
        }

        mPreviousUpdateTime = mWorld->SimTime();

        mForwardVelocity = 0;
        mPitch = 0;
        mIsAlive = true;

        // Reset odometric pose
        odomPose[0] = 0.0;
        odomPose[1] = 0.0;
        odomPose[2] = 0.0;

        odomVel[0] = 0.0;
        odomVel[1] = 0.0;
        odomVel[2] = 0.0;
    }

    // Update the controller
    void DiffDrivePlugin6W::Update() {

        // TODO(amg): This is probably bad...
        static const double timeout = 0.01;
        mQueue.callAvailable(ros::WallDuration(timeout));

        // TODO: Step should be in a parameter of this function
        double d1, d2;
        double dr, da;
        common::Time stepTime;

        GetPositionCmd();

        //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
        stepTime = mWorld->SimTime() - mPreviousUpdateTime;
        mPreviousUpdateTime = mWorld->SimTime();

        // Distance travelled by front wheels
        d1 = stepTime.Double() * mWheelDiameter / 2 * mJoints[MID_LEFT]->GetVelocity(0);
        d2 = stepTime.Double() * mWheelDiameter / 2 * mJoints[MID_RIGHT]->GetVelocity(0);

        dr = (d1 + d2) / 2;
        da = (d1 - d2) / mWheelSeparation;

        // Compute odometric pose
        odomPose[0] += dr * cos(odomPose[2]);
        odomPose[1] += dr * sin(odomPose[2]);
        odomPose[2] += da;

        // Compute odometric instantaneous velocity
        odomVel[0] = dr / stepTime.Double();
        odomVel[1] = 0.0;
        odomVel[2] = da / stepTime.Double();

        if (mEnableMotors) {
            mJoints[FRONT_LEFT]->SetVelocity(0, mWheelSpeed[0] / (mWheelDiameter / 2.0));
            mJoints[MID_LEFT]->SetVelocity(0, mWheelSpeed[0] / (mWheelDiameter / 2.0));
            mJoints[REAR_LEFT]->SetVelocity(0, mWheelSpeed[0] / (mWheelDiameter / 2.0));

            mJoints[FRONT_RIGHT]->SetVelocity(0, mWheelSpeed[1] / (mWheelDiameter / 2.0));
            mJoints[MID_RIGHT]->SetVelocity(0, mWheelSpeed[1] / (mWheelDiameter / 2.0));
            mJoints[REAR_RIGHT]->SetVelocity(0, mWheelSpeed[1] / (mWheelDiameter / 2.0));

            mJoints[FRONT_LEFT]->SetEffortLimit(0, mTorque);
            mJoints[MID_LEFT]->SetEffortLimit(0, mTorque);
            mJoints[REAR_LEFT]->SetEffortLimit(0, mTorque);

            mJoints[FRONT_RIGHT]->SetEffortLimit(0, mTorque);
            mJoints[MID_RIGHT]->SetEffortLimit(0, mTorque);
            mJoints[REAR_RIGHT]->SetEffortLimit(0, mTorque);
        }

        publish_odometry();
    }

    // NEW: Now uses the target velocities from the ROS message, not the Iface
    void DiffDrivePlugin6W::GetPositionCmd() {
        std::lock_guard guard(mLock);

        double vr, va;

        vr = mForwardVelocity;
        va = -mPitch;

        // Changed motors to be always on, which is probably what we want anyway
        mEnableMotors = true;

        mWheelSpeed[0] = vr + va * mWheelSeparation / 2;
        mWheelSpeed[1] = vr - va * mWheelSeparation / 2;
    }

    // NEW: Store the velocities from the ROS message
    void DiffDrivePlugin6W::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twistCommand) {
        std::lock_guard guard(mLock);

        mForwardVelocity = twistCommand->linear.x;
        mPitch = twistCommand->angular.z;
    }

    // NEW: custom callback queue thread
    void DiffDrivePlugin6W::QueueThread() {
        static const double timeout = 0.01;

        while (mIsAlive && mNode->ok()) {
            mQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    // NEW: Update this to publish odometry topic
    void DiffDrivePlugin6W::publish_odometry() {
        // get current time
        ros::Time currentTime((mWorld->SimTime()).sec, (mWorld->SimTime()).nsec);

        // getting data for base_footprint to odom transform
        ignition::math::Pose3d pose = mBodyLink->WorldPose();
        ignition::math::Vector3d velocity = mBodyLink->WorldLinearVel();
        ignition::math::Vector3d angularVelocity = mBodyLink->WorldAngularVel();

        //        tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        //        tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        //        tf::Transform base_footprint_to_odom(qt, vt);

        //        transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
        //                                                                   currentTime,
        //                                                                   "odom",
        //                                                                   "base_link"));

        // publish odom topic
        mOdometry.pose.pose.position.x = pose.Pos().X();
        mOdometry.pose.pose.position.y = pose.Pos().Y();

        mOdometry.pose.pose.orientation.x = pose.Rot().X();
        mOdometry.pose.pose.orientation.y = pose.Rot().Y();
        mOdometry.pose.pose.orientation.z = pose.Rot().Z();
        mOdometry.pose.pose.orientation.w = pose.Rot().W();

        mOdometry.twist.twist.linear.x = velocity.X();
        mOdometry.twist.twist.linear.y = velocity.Y();
        mOdometry.twist.twist.angular.z = angularVelocity.Z();

        mOdometry.header.frame_id = tf::resolve(mTfPrefix, "odom");
        mOdometry.child_frame_id = "base_link";
        mOdometry.header.stamp = currentTime;

        mPublisher.publish(mOdometry);
    }

    GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin6W)

} // namespace gazebo
