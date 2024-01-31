#include "arm_translator.hpp"

namespace mrover {

    ArmTranslator::ArmTranslator(ros::NodeHandle& nh) {
        assert(mJointDEPitchIndex == mJointDE0Index);
        assert(mJointDERollIndex == mJointDE1Index);
        assert(mArmHWNames.size() == mRawArmNames.size());
        for (size_t i = 0; i < mRawArmNames.size(); ++i) {
            if (i != mJointDEPitchIndex && i != mJointDERollIndex) {
                assert(mArmHWNames.at(i) == mRawArmNames.at(i));
            }
        }

        mJointDEPitchOffset = Radians{getFloatFromRosParam(nh, "joint_de/pitch_offset")};
        mJointDERollOffset = Radians{getFloatFromRosParam(nh, "joint_de/roll_offset")};

        mMinRadPerSecDE0 = RadiansPerSecond{getFloatFromRosParam(nh, "brushless_motors/controllers/joint_de_0/min_velocity")};
        mMaxRadPerSecDE0 = RadiansPerSecond{getFloatFromRosParam(nh, "brushless_motors/controllers/joint_de_0/max_velocity")};
        mMinRadPerSecDE1 = RadiansPerSecond{getFloatFromRosParam(nh, "brushless_motors/controllers/joint_de_1/min_velocity")};
        mMaxRadPerSecDE1 = RadiansPerSecond{getFloatFromRosParam(nh, "brushless_motors/controllers/joint_de_1/max_velocity")};

        mJointDEPitchPosSub = nh.subscribe<std_msgs::Float32>("joint_de_pitch_raw_position_data", 1, &ArmTranslator::processPitchRawPositionData, this);
        mJointDERollPosSub = nh.subscribe<std_msgs::Float32>("joint_de_roll_raw_position_data", 1, &ArmTranslator::processRollRawPositionData, this);

        mThrottleSub = nh.subscribe<Throttle>("arm_throttle_cmd", 1, &ArmTranslator::processThrottleCmd, this);
        mVelocitySub = nh.subscribe<Velocity>("arm_velocity_cmd", 1, &ArmTranslator::processVelocityCmd, this);
        mPositionSub = nh.subscribe<Position>("arm_position_cmd", 1, &ArmTranslator::processPositionCmd, this);
        mArmHWJointDataSub = nh.subscribe<sensor_msgs::JointState>("arm_hw_joint_data", 1, &ArmTranslator::processArmHWJointData, this);

        mThrottlePub = std::make_unique<ros::Publisher>(nh.advertise<Throttle>("arm_hw_throttle_cmd", 1));
        mVelocityPub = std::make_unique<ros::Publisher>(nh.advertise<Velocity>("arm_hw_velocity_cmd", 1));
        mPositionPub = std::make_unique<ros::Publisher>(nh.advertise<Position>("arm_hw_position_cmd", 1));
        mJointDataPub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::JointState>("arm_joint_data", 1));
    }

    void ArmTranslator::clampValues(float& val1, float& val2, float minValue1, float maxValue1, float minValue2, float maxValue2) {
        // val1 = (val1 - (-80)) / (maxValue1 - minValue1) * ();
        // val2 if (val1 < minValue1) {
        //     float const ratio = minValue1 / val1;
        //     val1 *= ratio;
        //     val2 *= ratio;
        // }
        // if (maxValue1 < val1) {
        //     float const ratio = maxValue1 / val1;
        //     val1 *= ratio;
        //     val2 *= ratio;
        // }
        // if (val2 < minValue2) {
        //     float const ratio = minValue2 / val2;
        //     val1 *= ratio;
        //     val2 *= ratio;
        // }
        // if (maxValue2 < val2) {
        //     float const ratio = maxValue2 / val2;
        //     val1 *= ratio;
        //     val2 *= ratio;
        // }
    }

    void ArmTranslator::mapValue(float& val, float inputMinValue, float inputMaxValue, float outputMinValue, float outputMaxValue) {
        val = (val - inputMinValue) / (inputMaxValue - inputMinValue) * (outputMaxValue - outputMinValue) + outputMinValue;
    }

    void ArmTranslator::processThrottleCmd(Throttle::ConstPtr const& msg) {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->throttles.size()) {
            ROS_ERROR("Throttle requests for arm is ignored!");
            return;
        }

        Throttle throttle = *msg;
        ROS_INFO("pitch throttle: %f    roll throttle: %f", msg->throttles.at(mJointDEPitchIndex), msg->throttles.at(mJointDERollIndex));

        auto [joint_de_0_throttle, joint_de_1_throttle] = transformPitchRollToMotorOutputs(
                msg->throttles.at(mJointDEPitchIndex),
                msg->throttles.at(mJointDERollIndex));

        ROS_INFO("pre-mapped values: de_0 %f   de_1 %f", joint_de_0_throttle, joint_de_1_throttle);

        mapValue(
                joint_de_0_throttle,
                -80.0f,
                80.0f,
                -1.0f,
                1.0f);

        mapValue(
                joint_de_1_throttle,
                -80.0f,
                80.0f,
                -1.0f,
                1.0f);

        throttle.names.at(mJointDEPitchIndex) = "joint_de_0";
        throttle.names.at(mJointDERollIndex) = "joint_de_1";
        throttle.throttles.at(mJointDEPitchIndex) = joint_de_0_throttle;
        throttle.throttles.at(mJointDERollIndex) = joint_de_1_throttle;

        ROS_INFO("post-mapped values: de_0 %f   de_1 %f", joint_de_0_throttle, joint_de_1_throttle);

        mThrottlePub->publish(throttle);
    }

    bool ArmTranslator::jointDEIsCalibrated() {
        return mJointDE0PosOffset.has_value() && mJointDE1PosOffset.has_value();
    }

    void ArmTranslator::updatePositionOffsets() {
        if (!mCurrentRawJointDEPitch.has_value() || !mCurrentRawJointDERoll.has_value() || !mCurrentRawJointDE0Position.has_value() || !mCurrentRawJointDE1Position.has_value()) {
            return;
        }

        std::pair<float, float> expected_motor_outputs = transformPitchRollToMotorOutputs(mCurrentRawJointDEPitch->get(), mCurrentRawJointDERoll->get());
        if (mCurrentRawJointDE0Position.has_value()) {
            mJointDE0PosOffset = *mCurrentRawJointDE0Position - Radians{expected_motor_outputs.first};
        }
        if (mCurrentRawJointDE1Position.has_value()) {
            mJointDE1PosOffset = *mCurrentRawJointDE1Position - Radians{expected_motor_outputs.second};
        }
    }

    void ArmTranslator::processPitchRawPositionData(std_msgs::Float32::ConstPtr const& msg) {
        mCurrentRawJointDEPitch = Radians{msg->data};
        updatePositionOffsets();
    }

    void ArmTranslator::processRollRawPositionData(std_msgs::Float32::ConstPtr const& msg) {
        mCurrentRawJointDERoll = Radians{msg->data};
        updatePositionOffsets();
    }

    void ArmTranslator::processVelocityCmd(Velocity::ConstPtr const& msg) {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->velocities.size()) {
            ROS_ERROR("Velocity requests for arm is ignored!");
            return;
        }

        Velocity velocity = *msg;

        auto [joint_de_0_vel, joint_de_1_vel] = transformPitchRollToMotorOutputs(
                msg->velocities.at(mJointDEPitchIndex),
                msg->velocities.at(mJointDERollIndex));

        mapValue(
                joint_de_0_vel,
                -800.0,
                800.0,
                mMinRadPerSecDE1.get(),
                mMaxRadPerSecDE1.get());

        mapValue(
                joint_de_1_vel,
                -800.0,
                800.0,
                mMinRadPerSecDE1.get(),
                mMaxRadPerSecDE1.get());

        ROS_INFO("max velocity: %f", joint_de_0_vel);
        velocity.names.at(mJointDEPitchIndex) = "joint_de_0";
        velocity.names.at(mJointDERollIndex) = "joint_de_1";
        velocity.velocities.at(mJointDEPitchIndex) = joint_de_0_vel;
        velocity.velocities.at(mJointDERollIndex) = joint_de_1_vel;

        mVelocityPub->publish(velocity);
    }


    void ArmTranslator::processPositionCmd(Position::ConstPtr const& msg) {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->positions.size()) {
            ROS_ERROR("Position requests for arm is ignored!");
            return;
        }

        if (!jointDEIsCalibrated()) {
            ROS_ERROR("Position requests for arm is ignored because jointDEIsNotCalibrated!");
            return;
        }

        Position position = *msg;

        auto [joint_de_0_raw_pos, joint_de_1_raw_pos] = transformPitchRollToMotorOutputs(
                msg->positions.at(mJointDEPitchIndex),
                msg->positions.at(mJointDERollIndex));

        float joint_de_0_pos = joint_de_0_raw_pos + mJointDE0PosOffset->get();
        float joint_de_1_pos = joint_de_1_raw_pos + mJointDE1PosOffset->get();

        position.names.at(mJointDEPitchIndex) = "joint_de_0";
        position.names.at(mJointDERollIndex) = "joint_de_1";
        position.positions.at(mJointDEPitchIndex) = joint_de_0_pos;
        position.positions.at(mJointDERollIndex) = joint_de_1_pos;

        mPositionPub->publish(position);
    }

    void ArmTranslator::processArmHWJointData(sensor_msgs::JointState::ConstPtr const& msg) {
        if (mArmHWNames != msg->name || mArmHWNames.size() != msg->position.size() || mArmHWNames.size() != msg->velocity.size() || mArmHWNames.size() != msg->effort.size()) {
            ROS_ERROR("Forwarding joint data for arm is ignored!");
            return;
        }

        sensor_msgs::JointState jointState = *msg;

        auto [jointDEPitchVel, jointDERollVel] = transformMotorOutputsToPitchRoll(
                static_cast<float>(msg->velocity.at(mJointDE0Index)),
                static_cast<float>(msg->velocity.at(mJointDE1Index)));

        auto [jointDEPitchPos, jointDERollPos] = transformMotorOutputsToPitchRoll(
                static_cast<float>(msg->position.at(mJointDE0Index)),
                static_cast<float>(msg->position.at(mJointDE1Index)));

        auto [jointDEPitchEff, jointDERollEff] = transformMotorOutputsToPitchRoll(
                static_cast<float>(msg->effort.at(mJointDE0Index)),
                static_cast<float>(msg->effort.at(mJointDE1Index)));

        mCurrentRawJointDE0Position = Radians{msg->position.at(mJointDE0Index)};
        mCurrentRawJointDE1Position = Radians{msg->position.at(mJointDE1Index)};

        jointState.name.at(mJointDE0Index) = "joint_de_0";
        jointState.name.at(mJointDE1Index) = "joint_de_1";
        jointState.velocity.at(mJointDE0Index) = jointDEPitchVel;
        jointState.velocity.at(mJointDE1Index) = jointDERollVel;
        jointState.position.at(mJointDE0Index) = jointDEPitchVel;
        jointState.position.at(mJointDE1Index) = jointDERollVel;
        jointState.effort.at(mJointDE0Index) = jointDEPitchEff;
        jointState.effort.at(mJointDE1Index) = jointDERollEff;

        mJointDataPub->publish(jointState);
    }


} // namespace mrover
