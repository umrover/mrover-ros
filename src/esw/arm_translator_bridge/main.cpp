#include <ros/ros.h>

#include "units/units.hpp"
#include <memory.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <std_msgs/Float32.h>

std::vector<std::string> rawArmNames = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
std::unique_ptr<ros::Publisher> throttlePub;
std::unique_ptr<ros::Publisher> velocityPub;
std::unique_ptr<ros::Publisher> positionPub;
size_t joint_de_pitch_index = std::find(rawArmNames.begin(), rawArmNames.end(), "joint_de_pitch") - rawArmNames.begin();
size_t joint_de_roll_index = std::find(rawArmNames.begin(), rawArmNames.end(), "joint_de_roll") - rawArmNames.begin();
std::unique_ptr<ros::Subscriber> jointDEPitchPosSub;
std::unique_ptr<ros::Subscriber> jointDERollPosSub;

std::optional<mrover::Radians> jointDE0PosOffset = mrover::Radians{0};
std::optional<mrover::Radians> jointDE1PosOffset = mrover::Radians{0};

std::optional<mrover::Radians> currentJointDEPitch;
std::optional<mrover::Radians> currentJointDERoll;
std::optional<mrover::Radians> currentRawJointDE0Position; // TODO - need to update this
std::optional<mrover::Radians> currentRawJointDE1Position; // TODO - need to update this

// Define the transformation matrix
const std::array<std::array<float, 2>, 2> TRANSFORMATION_MATRIX = {{{{40, 40}},
                                                                    {{40, -40}}}};

std::unique_ptr<float> min_rad_per_sec_de_0;
std::unique_ptr<float> min_rad_per_sec_de_1;
std::unique_ptr<float> max_rad_per_sec_de_0;
std::unique_ptr<float> max_rad_per_sec_de_1;

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

// Function to transform coordinates
std::pair<float, float> transformPitchRollToMotorOutputs(float pitch, float roll) {
    // Create the input vector
    std::array<float, 2> inputVector = {pitch, roll};

    // Perform matrix multiplication
    std::array<float, 2> resultVector = {
            TRANSFORMATION_MATRIX[0][0] * inputVector[0] + TRANSFORMATION_MATRIX[0][1] * inputVector[1],
            TRANSFORMATION_MATRIX[1][0] * inputVector[0] + TRANSFORMATION_MATRIX[1][1] * inputVector[1]};

    // Extract m1 and m2 from the result vector
    float m1 = resultVector[0];
    float m2 = resultVector[1];

    return std::make_pair(m1, m2);
}

void processThrottleCmd(mrover::Throttle::ConstPtr const& msg) {
    if (rawArmNames != msg->names || rawArmNames.size() != msg->throttles.size()) {
        ROS_ERROR("Throttle requests for arm is ignored!");
        return;
    }

    mrover::Throttle throttle = *msg;

    auto [joint_de_0_throttle, joint_de_1_throttle] = transformPitchRollToMotorOutputs(
            msg->throttles[joint_de_pitch_index],
            msg->throttles[joint_de_roll_index]);

    clampValues(
            joint_de_0_throttle,
            joint_de_1_throttle,
            *min_rad_per_sec_de_0,
            *max_rad_per_sec_de_0,
            *min_rad_per_sec_de_1,
            *max_rad_per_sec_de_1);

    throttle.names[joint_de_pitch_index] = "joint_de_0";
    throttle.names[joint_de_roll_index] = "joint_de_1";
    throttle.throttles[joint_de_pitch_index] = joint_de_0_throttle;
    throttle.throttles[joint_de_roll_index] = joint_de_1_throttle;

    throttlePub->publish(throttle);
}

bool jointDEIsCalibrated() {
    return jointDE0PosOffset.has_value() && jointDE1PosOffset.has_value();
}

void updatePositionOffsets() {
    if (!currentJointDEPitch.has_value() || !currentJointDERoll.has_value() || !currentRawJointDE0Position.has_value() || !currentRawJointDE1Position.has_value()) {
        return;
    }

    std::pair<float, float> expected_motor_outputs = transformPitchRollToMotorOutputs(currentJointDEPitch->get(), currentJointDERoll->get());
    if (currentRawJointDE0Position.has_value()) {
        jointDE0PosOffset = *currentRawJointDE0Position - mrover::Radians{expected_motor_outputs.first};
    }
    if (currentRawJointDE1Position.has_value()) {
        jointDE1PosOffset = *currentRawJointDE1Position - mrover::Radians{expected_motor_outputs.second};
    }
}

void processPitchRawPositionData(std_msgs::Float32::ConstPtr const& msg) {
    currentJointDEPitch = mrover::Radians{msg->data};
    updatePositionOffsets();
}

void processRollRawPositionData(std_msgs::Float32::ConstPtr const& msg) {
    currentJointDERoll = mrover::Radians{msg->data};
    updatePositionOffsets();
}

void processVelocityCmd(mrover::Velocity::ConstPtr const& msg) {
    if (rawArmNames != msg->names || rawArmNames.size() != msg->velocities.size()) {
        ROS_ERROR("Velocity requests for arm is ignored!");
        return;
    }

    mrover::Velocity velocity = *msg;

    auto [joint_de_0_vel, joint_de_1_vel] = transformPitchRollToMotorOutputs(
            msg->velocities[joint_de_pitch_index],
            msg->velocities[joint_de_roll_index]);

    clampValues(
            joint_de_0_vel,
            joint_de_1_vel,
            -1.0f,
            1.0f,
            -1.0f,
            1.0f);

    velocity.names[joint_de_pitch_index] = "joint_de_0";
    velocity.names[joint_de_roll_index] = "joint_de_1";
    velocity.velocities[joint_de_pitch_index] = joint_de_0_vel;
    velocity.velocities[joint_de_roll_index] = joint_de_1_vel;

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
            msg->positions[joint_de_pitch_index],
            msg->positions[joint_de_roll_index]);

    float joint_de_0_pos = joint_de_0_raw_pos + jointDE0PosOffset->get();
    float joint_de_1_pos = joint_de_1_raw_pos + jointDE1PosOffset->get();

    position.names[joint_de_pitch_index] = "joint_de_0";
    position.names[joint_de_roll_index] = "joint_de_1";
    position.positions[joint_de_pitch_index] = joint_de_0_pos;
    position.positions[joint_de_roll_index] = joint_de_1_pos;

    positionPub->publish(position);
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

    min_rad_per_sec_de_0 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_0/min_velocity");
    max_rad_per_sec_de_0 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_0/max_velocity");
    min_rad_per_sec_de_1 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_1/min_velocity");
    max_rad_per_sec_de_1 = get_unique_float_from_ros_param(nh, "brushless_motors/controllers/joint_de_1/max_velocity");

    jointDEPitchPosSub = std::make_unique<ros::Subscriber>(nh.subscribe<std_msgs::Float32>("joint_de_pitch_raw_position_data", 1, processPitchRawPositionData));
    jointDERollPosSub = std::make_unique<ros::Subscriber>(nh.subscribe<std_msgs::Float32>("joint_de_roll_raw_position_data", 1, processRollRawPositionData));

    ros::Subscriber throttleSub = nh.subscribe<mrover::Throttle>("arm_throttle_cmd", 1, processThrottleCmd);
    ros::Subscriber velocitySub = nh.subscribe<mrover::Velocity>("arm_velocity_cmd", 1, processVelocityCmd);
    ros::Subscriber positionSub = nh.subscribe<mrover::Position>("arm_position_cmd", 1, processPositionCmd);

    throttlePub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Throttle>("arm_hw_throttle_cmd", 1));
    velocityPub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Velocity>("arm_hw_velocity_cmd", 1));
    positionPub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Position>("arm_hw_position_cmd", 1));

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}