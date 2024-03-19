#include "arm_translator.hpp"

#include "joint_de_translation.hpp"
#include "linear_joint_translation.hpp"

#include <params_utils.hpp>
#include <ros/duration.h>
#include <units/units.hpp>

namespace mrover {

    ArmTranslator::ArmTranslator(ros::NodeHandle& nh) {
        assert(mJointDEPitchIndex == mJointDE0Index);
        assert(mJointDERollIndex == mJointDE1Index);
        assert(mArmHWNames.size() == mRawArmNames.size());

        for (std::size_t i = 0; i < mRawArmNames.size(); ++i) {
            if (i != mJointDEPitchIndex && i != mJointDERollIndex) {
                assert(mArmHWNames.at(i) == mRawArmNames.at(i));
            }
            {
                auto rawName = static_cast<std::string>(mRawArmNames[i]);
                auto [_, was_inserted] = mAdjustServersByRawArmNames.try_emplace(rawName, nh.advertiseService(std::format("{}_adjust", rawName), &ArmTranslator::adjustServiceCallback, this));
                assert(was_inserted);
            }
            {
                auto hwName = static_cast<std::string>(mArmHWNames[i]);
                auto [_, was_inserted] = mAdjustClientsByArmHWNames.try_emplace(hwName, nh.serviceClient<AdjustMotor>(std::format("{}_adjust", hwName)));
                mAdjustMotorsPub[hwName] = nh.advertise<MotorsAdjust>(std::format("{}_adjust_cmd", hwName), 1);
                assert(was_inserted);
            }
        }

        mJointDEPitchOffset = requireParamAsUnit<Radians>(nh, "joint_de/pitch_offset");
        mJointDERollOffset = requireParamAsUnit<Radians>(nh, "joint_de/roll_offset");

        mMinRadPerSecDE0 = requireParamAsUnit<RadiansPerSecond>(nh, "brushless_motors/controllers/joint_de_0/min_velocity");
        mMaxRadPerSecDE0 = requireParamAsUnit<RadiansPerSecond>(nh, "brushless_motors/controllers/joint_de_0/max_velocity");
        mMinRadPerSecDE1 = requireParamAsUnit<RadiansPerSecond>(nh, "brushless_motors/controllers/joint_de_1/min_velocity");
        mMaxRadPerSecDE1 = requireParamAsUnit<RadiansPerSecond>(nh, "brushless_motors/controllers/joint_de_1/max_velocity");

        // mJointDEPitchPosSub = nh.subscribe<std_msgs::Float32>("joint_de_pitch_raw_position_data", 1, &ArmTranslator::processPitchRawPositionData, this);
        // mJointDERollPosSub = nh.subscribe<std_msgs::Float32>("joint_de_roll_raw_position_data", 1, &ArmTranslator::processRollRawPositionData, this);

        mJointALinMult = requireParamAsUnit<RadiansPerMeter>(nh, "brushless_motors/controllers/joint_a/rad_to_meters_ratio");

        mThrottleSub = nh.subscribe<Throttle>("arm_throttle_cmd", 1, &ArmTranslator::processThrottleCmd, this);
        mVelocitySub = nh.subscribe<Velocity>("arm_velocity_cmd", 1, &ArmTranslator::processVelocityCmd, this);
        mPositionSub = nh.subscribe<Position>("arm_position_cmd", 1, &ArmTranslator::processPositionCmd, this);
        mArmHWJointDataSub = nh.subscribe<sensor_msgs::JointState>("arm_hw_joint_data", 1, &ArmTranslator::processArmHWJointData, this);

        mThrottlePub = std::make_unique<ros::Publisher>(nh.advertise<Throttle>("arm_hw_throttle_cmd", 1));
        mVelocityPub = std::make_unique<ros::Publisher>(nh.advertise<Velocity>("arm_hw_velocity_cmd", 1));
        mPositionPub = std::make_unique<ros::Publisher>(nh.advertise<Position>("arm_hw_position_cmd", 1));
        mJointDataPub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::JointState>("arm_joint_data", 1));


        mUpdateDETimer = nh.createTimer(mUpdateDEDuration, &ArmTranslator::updateDEEncoderStates, this);
    }

    // void ArmTranslator::clampValues(float& val1, float& val2, float minValue1, float maxValue1, float minValue2, float maxValue2) {
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
    // }

    auto ArmTranslator::mapValue(float& val, float inputMinValue, float inputMaxValue, float outputMinValue, float outputMaxValue) -> void {
        val = (val - inputMinValue) / (inputMaxValue - inputMinValue) * (outputMaxValue - outputMinValue) + outputMinValue;
    }

    auto ArmTranslator::processThrottleCmd(Throttle::ConstPtr const& msg) const -> void {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->throttles.size()) {
            ROS_ERROR("Throttle requests for arm is ignored!");
            return;
        }

        Throttle throttle = *msg;
        ROS_INFO("pitch throttle: %f    roll throttle: %f", msg->throttles.at(mJointDEPitchIndex), msg->throttles.at(mJointDERollIndex));

        Eigen::Vector2f jointDe01Throttles = transformPitchRollToMotorOutputs({msg->throttles.at(mJointDEPitchIndex), msg->throttles.at(mJointDERollIndex)});

        ROS_INFO("pre-mapped values: de_0 %f   de_1 %f", jointDe01Throttles[0], jointDe01Throttles[1]);

        mapValue(
                jointDe01Throttles[0],
                -80.0f,
                80.0f,
                -1.0f,
                1.0f);

        mapValue(
                jointDe01Throttles[1],
                -80.0f,
                80.0f,
                -1.0f,
                1.0f);

        throttle.names.at(mJointDEPitchIndex) = "joint_de_0";
        throttle.names.at(mJointDERollIndex) = "joint_de_1";
        throttle.throttles.at(mJointDEPitchIndex) = jointDe01Throttles[0];
        throttle.throttles.at(mJointDERollIndex) = jointDe01Throttles[1];

        ROS_INFO("post-mapped values: de_0 %f   de_1 %f", jointDe01Throttles[0], jointDe01Throttles[1]);

        mThrottlePub->publish(throttle);
    }

    auto ArmTranslator::jointDEIsCalibrated() const -> bool {
        return mJointDE0PosOffset.has_value() && mJointDE1PosOffset.has_value();
    }

    auto ArmTranslator::updatePositionOffsets() -> void {
        if (!mCurrentRawJointDEPitch.has_value() || !mCurrentRawJointDERoll.has_value() || !mCurrentRawJointDE0Position.has_value() || !mCurrentRawJointDE1Position.has_value()) {
            return;
        }

        Eigen::Vector2f currentRawJointDePitchRoll = transformPitchRollToMotorOutputs({mCurrentRawJointDEPitch->get(), mCurrentRawJointDERoll->get()});
        if (mCurrentRawJointDE0Position.has_value()) {
            mJointDE0PosOffset = *mCurrentRawJointDE0Position - Radians{currentRawJointDePitchRoll[0]};
        }
        if (mCurrentRawJointDE1Position.has_value()) {
            mJointDE1PosOffset = *mCurrentRawJointDE1Position - Radians{currentRawJointDePitchRoll[1]};
        }
    }

    // auto ArmTranslator::processPitchRawPositionData(std_msgs::Float32::ConstPtr const& msg) -> void {
    //     mCurrentRawJointDEPitch = Radians{msg->data};
    //     updatePositionOffsets();
    // }

    // auto ArmTranslator::processRollRawPositionData(std_msgs::Float32::ConstPtr const& msg) -> void {
    //     mCurrentRawJointDERoll = Radians{msg->data};
    //     updatePositionOffsets();
    // }

    auto ArmTranslator::processVelocityCmd(Velocity::ConstPtr const& msg) -> void {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->velocities.size()) {
            ROS_ERROR("Velocity requests for arm is ignored!");
            return;
        }

        Velocity velocity = *msg;

        auto jointDe01Vel = transformPitchRollToMotorOutputs({msg->velocities.at(mJointDEPitchIndex), msg->velocities.at(mJointDERollIndex)});

        mapValue(
                jointDe01Vel[0],
                -800.0,
                800.0,
                mMinRadPerSecDE1.get(),
                mMaxRadPerSecDE1.get());

        mapValue(
                jointDe01Vel[1],
                -800.0,
                800.0,
                mMinRadPerSecDE1.get(),
                mMaxRadPerSecDE1.get());

        velocity.names.at(mJointDEPitchIndex) = "joint_de_0";
        velocity.names.at(mJointDERollIndex) = "joint_de_1";
        velocity.velocities.at(mJointDEPitchIndex) = jointDe01Vel[0];
        velocity.velocities.at(mJointDERollIndex) = jointDe01Vel[1];

        // joint a convert linear velocity (meters/s) to revolution/s
        float joint_a_vel = convertLinVel(msg->velocities.at(mJointAIndex), mJointALinMult.get());
        velocity.velocities.at(mJointAIndex) = joint_a_vel;
        ROS_INFO("joint a velocity after conversion: %f", joint_a_vel);

        mVelocityPub->publish(velocity);
    }

    auto ArmTranslator::processPositionCmd(Position::ConstPtr const& msg) -> void {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->positions.size()) {
            ROS_ERROR("Position requests for arm is ignored!");
            return;
        }

        if (!jointDEIsCalibrated()) {
            ROS_ERROR("Position requests for arm is ignored because jointDEIsNotCalibrated!");
            return;
        }

        Position position = *msg;

        Eigen::Vector2f jointDe01RawPos = transformPitchRollToMotorOutputs({msg->positions.at(mJointDEPitchIndex), msg->positions.at(mJointDERollIndex)});

        float joint_de_0_pos = jointDe01RawPos[0] + mJointDE0PosOffset->get();
        float joint_de_1_pos = jointDe01RawPos[1] + mJointDE1PosOffset->get();

        position.names.at(mJointDEPitchIndex) = "joint_de_0";
        position.names.at(mJointDERollIndex) = "joint_de_1";
        position.positions.at(mJointDEPitchIndex) = joint_de_0_pos;
        position.positions.at(mJointDERollIndex) = joint_de_1_pos;

        // joint a convert linear position (meters) to radians
        auto joint_a_pos = convertLinPos(msg->positions.at(mJointAIndex), mJointALinMult.get());
        position.positions.at(mJointAIndex) = joint_a_pos;

        mPositionPub->publish(position);
    }

    auto ArmTranslator::processArmHWJointData(sensor_msgs::JointState::ConstPtr const& msg) -> void {
        if (mArmHWNames != msg->name || mArmHWNames.size() != msg->position.size() || mArmHWNames.size() != msg->velocity.size() || mArmHWNames.size() != msg->effort.size()) {
            ROS_ERROR("Forwarding joint data for arm is ignored!");
            return;
        }

        // Convert joint state of joint a from radians/revolutions to meters
        auto jointALinVel = convertLinVel(static_cast<float>(msg->velocity.at(mJointAIndex)), mJointALinMult.get());
        auto jointALinPos = convertLinPos(static_cast<float>(msg->position.at(mJointAIndex)), mJointALinMult.get());


        sensor_msgs::JointState jointState = *msg;

        Eigen::Vector2f jointDePitchRollVel = transformMotorOutputsToPitchRoll({static_cast<float>(msg->velocity.at(mJointDE0Index)), static_cast<float>(msg->velocity.at(mJointDE1Index))});

        auto jointDEPitchPos = static_cast<float>(msg->position.at(mJointDE0Index));
        auto jointDERollPos = static_cast<float>(msg->position.at(mJointDE1Index));

        Eigen::Vector2f jointDePitchRollEff = transformMotorOutputsToPitchRoll({static_cast<float>(msg->effort.at(mJointDE0Index)), static_cast<float>(msg->effort.at(mJointDE1Index))});

        mCurrentRawJointDEPitch = Radians{msg->position.at(mJointDE0Index)};
        mCurrentRawJointDERoll = Radians{msg->position.at(mJointDE1Index)};

        Eigen::Vector2f jointDe01Pos = transformPitchRollToMotorOutputs({jointDEPitchPos, jointDERollPos});
        mCurrentRawJointDE0Position = Radians{jointDe01Pos[0]};
        mCurrentRawJointDE1Position = Radians{jointDe01Pos[1]};


        jointState.name.at(mJointDE0Index) = "joint_de_0";
        jointState.name.at(mJointDE1Index) = "joint_de_1";
        jointState.velocity.at(mJointDE0Index) = jointDePitchRollVel[0];
        jointState.velocity.at(mJointDE1Index) = jointDePitchRollVel[1];
        jointState.position.at(mJointDE0Index) = jointDEPitchPos;
        jointState.position.at(mJointDE1Index) = jointDERollPos;
        jointState.effort.at(mJointDE0Index) = jointDePitchRollEff[0];
        jointState.effort.at(mJointDE1Index) = jointDePitchRollEff[1];

        jointState.velocity.at(mJointAIndex) = jointALinVel;
        jointState.position.at(mJointAIndex) = jointALinPos;

        mJointDataPub->publish(jointState);
    }

    auto ArmTranslator::adjustServiceCallback(AdjustMotor::Request& req, AdjustMotor::Response& res) -> bool {

        if (req.name == "joint_de_roll") {
            mJointDERollAdjust = req.value;
        } else if (req.name == "joint_de_pitch") {
            mJointDEPitchAdjust = req.value;
        } else if (req.name == "joint_a") {
            AdjustMotor::Request controllerReq;
            AdjustMotor::Response controllerRes = res;
            controllerReq.value = convertLinPos(req.value, mJointALinMult.get());

            mAdjustClientsByArmHWNames[req.name].call(controllerReq, controllerRes);
            res.success = controllerRes.success;
        } else {
            AdjustMotor::Request controllerReq = req;
            AdjustMotor::Response controllerRes = res;
            mAdjustClientsByArmHWNames[req.name].call(controllerReq, controllerRes);
            res.success = controllerRes.success;
        }

        if (mJointDEPitchAdjust && mJointDERollAdjust) {
            // convert DE_roll and DE_pitch into DE_0 and DE_1 (outgoing message to arm_hw_bridge)
            Eigen::Vector2f jointDe01RawValue = transformPitchRollToMotorOutputs({mJointDEPitchAdjust.value(), mJointDERollAdjust.value()});
            mJointDEPitchAdjust = std::nullopt;
            mJointDERollAdjust = std::nullopt;

            float joint_de_0_value = jointDe01RawValue[0] + mJointDE0PosOffset->get();
            float joint_de_1_value = jointDe01RawValue[1] + mJointDE1PosOffset->get();

            AdjustMotor::Response controllerResDE0;
            AdjustMotor::Request controllerReqDE0;
            controllerReqDE0.name = "joint_de_0";
            controllerReqDE0.value = joint_de_0_value;
            mAdjustClientsByArmHWNames[controllerReqDE0.name].call(controllerResDE0, controllerReqDE0);

            AdjustMotor::Response controllerResDE1;
            AdjustMotor::Request controllerReqDE1;
            controllerReqDE1.name = "joint_de_1";
            controllerReqDE1.value = joint_de_1_value;
            mAdjustClientsByArmHWNames[controllerReqDE1.name].call(controllerReqDE1, controllerResDE1);

            res.success = controllerResDE0.success && controllerResDE1.success;
        } else {
            // adjust service was for de, but both de joints have not adjusted yet
            res.success = false;
        }
        return true;
    }

    auto ArmTranslator::updateDEEncoderStates(ros::TimerEvent const&) -> void {
        // Update the DE joint states from the absolute encoders

        // Transform to real values
        Eigen::Vector2f jointDe01Value = transformPitchRollToMotorOutputs({mCurrentRawJointDEPitch->get(), mCurrentRawJointDERoll->get()});

        // Publish to controller
        MotorsAdjust de0AdjustMsg;
        MotorsAdjust de1AdjustMsg;

        de0AdjustMsg.names = {"joint_de_0"};
        de0AdjustMsg.values = {jointDe01Value[0]};

        de1AdjustMsg.names = {"joint_de_1"};
        de1AdjustMsg.values = {jointDe01Value[1]};

        mAdjustMotorsPub["joint_de_1"].publish(de0AdjustMsg);
        mAdjustMotorsPub["joint_de_1"].publish(de1AdjustMsg);
    }

} // namespace mrover
