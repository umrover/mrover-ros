#include "arm_translator.hpp"

namespace mrover {

    ArmTranslator::ArmTranslator(ros::NodeHandle& nh) {
        assert(joint_de_pitch_index == joint_de_0_index);
        assert(joint_de_roll_index == joint_de_1_index);
        assert(armHWNames.size() == rawArmNames.size());
        for (size_t i = 0; i < rawArmNames.size(); ++i) {
            if (i != joint_de_pitch_index && i != joint_de_roll_index) {
                assert(armHWNames.at(i) == rawArmNames.at(i));
            }
        }

        jointDEPitchOffset = Radians{*get_unique_float_from_ros_param(nh, "joint_de/pitch_offset")};
        jointDEPitchOffset = Radians{*get_unique_float_from_ros_param(nh, "joint_de/roll_offset")};

        min_rad_per_sec_de_0 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_0/min_velocity");
        max_rad_per_sec_de_0 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_0/max_velocity");
        min_rad_per_sec_de_1 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_1/min_velocity");
        max_rad_per_sec_de_1 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_1/max_velocity");

        jointDEPitchPosSub = nh.subscribe<std_msgs::Float32>("joint_de_pitch_raw_position_data", 1, &ArmTranslator::processPitchRawPositionData, this);
        jointDERollPosSub = nh.subscribe<std_msgs::Float32>("joint_de_roll_raw_position_data", 1, &ArmTranslator::processRollRawPositionData, this);

        throttleSub = nh.subscribe<Throttle>("arm_throttle_cmd", 1, &ArmTranslator::processThrottleCmd, this);
        velocitySub = nh.subscribe<Velocity>("arm_velocity_cmd", 1, &ArmTranslator::processVelocityCmd, this);
        positionSub = nh.subscribe<Position>("arm_position_cmd", 1, &ArmTranslator::processPositionCmd, this);
        armHWJointDataSub = nh.subscribe<sensor_msgs::JointState>("arm_hw_joint_data", 1, &ArmTranslator::processArmHWJointData, this);

        throttlePub = std::make_unique<ros::Publisher>(nh.advertise<Throttle>("arm_hw_throttle_cmd", 1));
        velocityPub = std::make_unique<ros::Publisher>(nh.advertise<Velocity>("arm_hw_velocity_cmd", 1));
        positionPub = std::make_unique<ros::Publisher>(nh.advertise<Position>("arm_hw_position_cmd", 1));
        jointDataPub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::JointState>("arm_joint_data", 1));
    }

    void ArmTranslator::clampValues(float& val1, float& val2, float minValue1, float maxValue1, float minValue2, float maxValue2) {
        if (val1 < minValue1) {
            float const ratio = minValue1 / val1;
            val1 *= ratio;
            val2 *= ratio;
        }
        if (maxValue1 < val1) {
            float const ratio = maxValue1 / val1;
            val1 *= ratio;
            val2 *= ratio;
        }
        if (val2 < minValue2) {
            float const ratio = minValue2 / val2;
            val1 *= ratio;
            val2 *= ratio;
        }
        if (maxValue2 < val2) {
            float const ratio = maxValue2 / val2;
            val1 *= ratio;
            val2 *= ratio;
        }
    }

    void ArmTranslator::processThrottleCmd(Throttle::ConstPtr const& msg) {
        if (rawArmNames != msg->names || rawArmNames.size() != msg->throttles.size()) {
            ROS_ERROR("Throttle requests for arm is ignored!");
            return;
        }

        Throttle throttle = *msg;

        auto [joint_de_0_throttle, joint_de_1_throttle] = transformPitchRollToMotorOutputs(
                msg->throttles.at(joint_de_pitch_index),
                msg->throttles.at(joint_de_roll_index));

        clampValues(
                joint_de_0_throttle,
                joint_de_1_throttle,
                *min_rad_per_sec_de_0,
                *max_rad_per_sec_de_0,
                *min_rad_per_sec_de_1,
                *max_rad_per_sec_de_1);

        throttle.names.at(joint_de_pitch_index) = "joint_de_0";
        throttle.names.at(joint_de_roll_index) = "joint_de_1";
        throttle.throttles.at(joint_de_pitch_index) = joint_de_0_throttle;
        throttle.throttles.at(joint_de_roll_index) = joint_de_1_throttle;

        throttlePub->publish(throttle);
    }

    bool ArmTranslator::jointDEIsCalibrated() {
        return jointDE0PosOffset.has_value() && jointDE1PosOffset.has_value();
    }

    void ArmTranslator::updatePositionOffsets() {
        if (!currentRawJointDEPitch.has_value() || !currentRawJointDERoll.has_value() || !currentRawJointDE0Position.has_value() || !currentRawJointDE1Position.has_value()) {
            return;
        }

        std::pair<float, float> expected_motor_outputs = transformPitchRollToMotorOutputs(currentRawJointDEPitch->get(), currentRawJointDERoll->get());
        if (currentRawJointDE0Position.has_value()) {
            jointDE0PosOffset = *currentRawJointDE0Position - Radians{expected_motor_outputs.first};
        }
        if (currentRawJointDE1Position.has_value()) {
            jointDE1PosOffset = *currentRawJointDE1Position - Radians{expected_motor_outputs.second};
        }
    }

    void ArmTranslator::processPitchRawPositionData(std_msgs::Float32::ConstPtr const& msg) {
        currentRawJointDEPitch = Radians{msg->data};
        updatePositionOffsets();
    }

    void ArmTranslator::processRollRawPositionData(std_msgs::Float32::ConstPtr const& msg) {
        currentRawJointDERoll = Radians{msg->data};
        updatePositionOffsets();
    }

    void ArmTranslator::processVelocityCmd(Velocity::ConstPtr const& msg) {
        if (rawArmNames != msg->names || rawArmNames.size() != msg->velocities.size()) {
            ROS_ERROR("Velocity requests for arm is ignored!");
            return;
        }

        Velocity velocity = *msg;

        auto [joint_de_0_vel, joint_de_1_vel] = transformPitchRollToMotorOutputs(
                msg->velocities.at(joint_de_pitch_index),
                msg->velocities.at(joint_de_roll_index));

        clampValues(
                joint_de_0_vel,
                joint_de_1_vel,
                -1.0f,
                1.0f,
                -1.0f,
                1.0f);

        velocity.names.at(joint_de_pitch_index) = "joint_de_0";
        velocity.names.at(joint_de_roll_index) = "joint_de_1";
        velocity.velocities.at(joint_de_pitch_index) = joint_de_0_vel;
        velocity.velocities.at(joint_de_roll_index) = joint_de_1_vel;

        velocityPub->publish(velocity);
    }


    void ArmTranslator::processPositionCmd(Position::ConstPtr const& msg) {
        if (rawArmNames != msg->names || rawArmNames.size() != msg->positions.size()) {
            ROS_ERROR("Position requests for arm is ignored!");
            return;
        }

        if (!jointDEIsCalibrated()) {
            ROS_ERROR("Position requests for arm is ignored because jointDEIsNotCalibrated!");
            return;
        }

        Position position = *msg;

        auto [joint_de_0_raw_pos, joint_de_1_raw_pos] = transformPitchRollToMotorOutputs(
                msg->positions.at(joint_de_pitch_index),
                msg->positions.at(joint_de_roll_index));

        float joint_de_0_pos = joint_de_0_raw_pos + jointDE0PosOffset->get();
        float joint_de_1_pos = joint_de_1_raw_pos + jointDE1PosOffset->get();

        position.names.at(joint_de_pitch_index) = "joint_de_0";
        position.names.at(joint_de_roll_index) = "joint_de_1";
        position.positions.at(joint_de_pitch_index) = joint_de_0_pos;
        position.positions.at(joint_de_roll_index) = joint_de_1_pos;

        positionPub->publish(position);
    }

    void ArmTranslator::processArmHWJointData(sensor_msgs::JointState::ConstPtr const& msg) {
        if (armHWNames != msg->name || armHWNames.size() != msg->position.size() || armHWNames.size() != msg->velocity.size() || armHWNames.size() != msg->effort.size()) {
            ROS_ERROR("Forwarding joint data for arm is ignored!");
            return;
        }

        sensor_msgs::JointState jointState = *msg;

        auto [joint_de_pitch_vel, joint_de_roll_vel] = transformMotorOutputsToPitchRoll(
                static_cast<float>(msg->velocity.at(joint_de_0_index)),
                static_cast<float>(msg->velocity.at(joint_de_1_index)));

        auto [joint_de_pitch_pos, joint_de_roll_pos] = transformMotorOutputsToPitchRoll(
                static_cast<float>(msg->position.at(joint_de_0_index)),
                static_cast<float>(msg->position.at(joint_de_1_index)));

        auto [joint_de_pitch_eff, joint_de_roll_eff] = transformMotorOutputsToPitchRoll(
                static_cast<float>(msg->effort.at(joint_de_0_index)),
                static_cast<float>(msg->effort.at(joint_de_1_index)));

        currentRawJointDE0Position = Radians{msg->position.at(joint_de_0_index)};
        currentRawJointDE1Position = Radians{msg->position.at(joint_de_1_index)};

        jointState.name.at(joint_de_0_index) = "joint_de_0";
        jointState.name.at(joint_de_1_index) = "joint_de_1";
        jointState.velocity.at(joint_de_0_index) = joint_de_pitch_vel;
        jointState.velocity.at(joint_de_1_index) = joint_de_roll_vel;
        jointState.position.at(joint_de_0_index) = joint_de_pitch_vel;
        jointState.position.at(joint_de_1_index) = joint_de_roll_vel;
        jointState.effort.at(joint_de_0_index) = joint_de_pitch_eff;
        jointState.effort.at(joint_de_1_index) = joint_de_roll_eff;

        jointDataPub->publish(jointState);
    }


} // namespace mrover
