#include <ros/ros.h>

#include "units/units.hpp"
#include <memory.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

const std::vector<std::string> rawArmNames = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
const std::vector<std::string> armHWNames = {"joint_a", "joint_b", "joint_c", "joint_de_0", "joint_de_1", "allen_key", "gripper"};
std::unique_ptr<ros::Publisher> throttlePub;
std::unique_ptr<ros::Publisher> velocityPub;
std::unique_ptr<ros::Publisher> positionPub;
std::unique_ptr<ros::Publisher> jointDataPub;
const size_t joint_de_pitch_index = std::find(rawArmNames.begin(), rawArmNames.end(), "joint_de_pitch") - rawArmNames.begin();
const size_t joint_de_roll_index = std::find(rawArmNames.begin(), rawArmNames.end(), "joint_de_roll") - rawArmNames.begin();
const size_t joint_de_0_index = std::find(armHWNames.begin(), armHWNames.end(), "joint_de_0") - armHWNames.begin();
const size_t joint_de_1_index = std::find(armHWNames.begin(), armHWNames.end(), "joint_de_1") - armHWNames.begin();

std::optional<mrover::Radians> jointDE0PosOffset = mrover::Radians{0};
std::optional<mrover::Radians> jointDE1PosOffset = mrover::Radians{0};

std::optional<mrover::Radians> currentRawJointDEPitch;
std::optional<mrover::Radians> currentRawJointDERoll;
std::optional<mrover::Radians> currentRawJointDE0Position;
std::optional<mrover::Radians> currentRawJointDE1Position;

// Define the transformation matrix
const std::array<std::array<float, 2>, 2> CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX = {{{{40, 40}},
                                                                                  {{40, -40}}}};

std::unique_ptr<float> min_rad_per_sec_de_0;
std::unique_ptr<float> min_rad_per_sec_de_1;
std::unique_ptr<float> max_rad_per_sec_de_0;
std::unique_ptr<float> max_rad_per_sec_de_1;

std::array<std::array<float, 2>, 2> inverseMatrix(std::array<std::array<float, 2>, 2> const& matrix) {
    float determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    float invDet = 1.0f / determinant;

    std::array<std::array<float, 2>, 2> result = {{{matrix[1][1] * invDet, -matrix[0][1] * invDet},
                                                   {-matrix[1][0] * invDet, matrix[0][0] * invDet}}};

    return result;
}

const std::array<std::array<float, 2>, 2> CONVERT_MOTORS_TO_PITCH_ROLL_MATRIX = inverseMatrix(CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX);

void clampValues(float& val1, float& val2, float minValue1, float maxValue1, float minValue2, float maxValue2) {
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

std::array<float, 2> multiplyMatrixByVector(std::array<float, 2> const& vector, std::array<std::array<float, 2>, 2> const& matrix) {
    std::array<float, 2> result = {
            matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
            matrix[1][0] * vector[0] + matrix[1][1] * vector[1]};
    return result;
}

std::pair<float, float> transformMotorOutputsToPitchRoll(float motor0, float motor1) {

    std::array<float, 2> motorsVector = {motor0, motor1};

    // Perform matrix multiplication
    auto [pitch, roll] = multiplyMatrixByVector(motorsVector, CONVERT_MOTORS_TO_PITCH_ROLL_MATRIX);

    return std::make_pair(pitch, roll);
}

// Function to transform coordinates
std::pair<float, float> transformPitchRollToMotorOutputs(float pitch, float roll) {
    // Create the input vector
    std::array<float, 2> pitchRollVector = {pitch, roll};

    // Perform matrix multiplication
    auto [m1, m2] = multiplyMatrixByVector(pitchRollVector, CONVERT_PITCH_ROLL_TO_MOTORS_MATRIX);

    return std::make_pair(m1, m2);
}

void processThrottleCmd(mrover::Throttle::ConstPtr const& msg) {
    if (rawArmNames != msg->names || rawArmNames.size() != msg->throttles.size()) {
        ROS_ERROR("Throttle requests for arm is ignored!");
        return;
    }

    mrover::Throttle throttle = *msg;

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

bool jointDEIsCalibrated() {
    return jointDE0PosOffset.has_value() && jointDE1PosOffset.has_value();
}

void updatePositionOffsets() {
    if (!currentRawJointDEPitch.has_value() || !currentRawJointDERoll.has_value() || !currentRawJointDE0Position.has_value() || !currentRawJointDE1Position.has_value()) {
        return;
    }

    std::pair<float, float> expected_motor_outputs = transformPitchRollToMotorOutputs(currentRawJointDEPitch->get(), currentRawJointDERoll->get());
    if (currentRawJointDE0Position.has_value()) {
        jointDE0PosOffset = *currentRawJointDE0Position - mrover::Radians{expected_motor_outputs.first};
    }
    if (currentRawJointDE1Position.has_value()) {
        jointDE1PosOffset = *currentRawJointDE1Position - mrover::Radians{expected_motor_outputs.second};
    }
}

void processPitchRawPositionData(std_msgs::Float32::ConstPtr const& msg) {
    currentRawJointDEPitch = mrover::Radians{msg->data};
    updatePositionOffsets();
}

void processRollRawPositionData(std_msgs::Float32::ConstPtr const& msg) {
    currentRawJointDERoll = mrover::Radians{msg->data};
    updatePositionOffsets();
}

void processVelocityCmd(mrover::Velocity::ConstPtr const& msg) {
    if (rawArmNames != msg->names || rawArmNames.size() != msg->velocities.size()) {
        ROS_ERROR("Velocity requests for arm is ignored!");
        return;
    }

    mrover::Velocity velocity = *msg;

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

void processPositionCmd(mrover::Position::ConstPtr const& msg) {
    if (rawArmNames != msg->names || rawArmNames.size() != msg->positions.size()) {
        ROS_ERROR("Position requests for arm is ignored!");
        return;
    }

    if (!jointDEIsCalibrated()) {
        ROS_ERROR("Position requests for arm is ignored because jointDEIsNotCalibrated!");
        return;
    }

    mrover::Position position = *msg;

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

void processArmHWJointData(sensor_msgs::JointState::ConstPtr const& msg) {
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

    currentRawJointDE0Position = mrover::Radians{msg->position.at(joint_de_0_index)};
    currentRawJointDE1Position = mrover::Radians{msg->position.at(joint_de_1_index)};

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

std::unique_ptr<float> get_unique_float_from_ros_param(ros::NodeHandle const& nh, std::string const& param_name) {
    XmlRpc::XmlRpcValue rpcValue;
    assert(nh.hasParam(param_name));
    nh.getParam(param_name, rpcValue);
    assert(rpcValue.getType() == XmlRpc::XmlRpcValue::TypeDouble);
    auto value = (float) static_cast<double>(rpcValue);
    return std::make_unique<float>(value);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_translator_bridge");
    ros::NodeHandle nh;

    assert(joint_de_pitch_index == joint_de_0_index);
    assert(joint_de_roll_index == joint_de_1_index);
    assert(armHWNames.size() == rawArmNames.size());
    for (size_t i = 0; i < rawArmNames.size(); ++i) {
        if (i != joint_de_pitch_index && i != joint_de_roll_index) {
            assert(armHWNames.at(i) == rawArmNames.at(i));
        }
    }

    min_rad_per_sec_de_0 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_0/min_velocity");
    max_rad_per_sec_de_0 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_0/max_velocity");
    min_rad_per_sec_de_1 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_1/min_velocity");
    max_rad_per_sec_de_1 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_1/max_velocity");

    ros::Subscriber jointDEPitchPosSub = nh.subscribe<std_msgs::Float32>("joint_de_pitch_raw_position_data", 1, processPitchRawPositionData);
    ros::Subscriber jointDERollPosSub = nh.subscribe<std_msgs::Float32>("joint_de_roll_raw_position_data", 1, processRollRawPositionData);

    ros::Subscriber throttleSub = nh.subscribe<mrover::Throttle>("arm_throttle_cmd", 1, processThrottleCmd);
    ros::Subscriber velocitySub = nh.subscribe<mrover::Velocity>("arm_velocity_cmd", 1, processVelocityCmd);
    ros::Subscriber positionSub = nh.subscribe<mrover::Position>("arm_position_cmd", 1, processPositionCmd);
    ros::Subscriber armHWJointDataSub = nh.subscribe<sensor_msgs::JointState>("arm_hw_joint_data", 1, processArmHWJointData);

    throttlePub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Throttle>("arm_hw_throttle_cmd", 1));
    velocityPub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Velocity>("arm_hw_velocity_cmd", 1));
    positionPub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Position>("arm_hw_position_cmd", 1));
    jointDataPub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::JointState>("arm_joint_data", 1));

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}