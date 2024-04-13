#include "arm_translator.hpp"

#include <params_utils.hpp>
#include <ros/duration.h>
#include <units/units.hpp>

#include <Eigen/LU>

namespace Eigen {

    template<
            typename Rep1, typename Conversion1, typename MeterExp1, typename KilogramExp1, typename SecondExp1, typename RadianExp1, typename AmpereExp1, typename KelvinExp1, typename ByteExp1, typename TickExp1,
            typename Rep2, typename Conversion2, typename MeterExp2, typename KilogramExp2, typename SecondExp2, typename RadianExp2, typename AmpereExp2, typename KelvinExp2, typename ByteExp2, typename TickExp2>
    struct ScalarBinaryOpTraits<
            mrover::Unit<Rep1, Conversion1, MeterExp1, KilogramExp1, SecondExp1, RadianExp1, AmpereExp1, KelvinExp1, ByteExp1, TickExp1>,
            mrover::Unit<Rep2, Conversion2, MeterExp2, KilogramExp2, SecondExp2, RadianExp2, AmpereExp2, KelvinExp2, ByteExp2, TickExp2>,
            internal::scalar_product_op<
                    mrover::Unit<Rep1, Conversion1, MeterExp1, KilogramExp1, SecondExp1, RadianExp1, AmpereExp1, KelvinExp1, ByteExp1, TickExp1>,
                    mrover::Unit<Rep2, Conversion2, MeterExp2, KilogramExp2, SecondExp2, RadianExp2, AmpereExp2, KelvinExp2, ByteExp2, TickExp2>>> {
        using U1 = mrover::Unit<Rep1, Conversion1, MeterExp1, KilogramExp1, SecondExp1, RadianExp1, AmpereExp1, KelvinExp1, ByteExp1, TickExp1>;
        using U2 = mrover::Unit<Rep2, Conversion2, MeterExp2, KilogramExp2, SecondExp2, RadianExp2, AmpereExp2, KelvinExp2, ByteExp2, TickExp2>;
        using ReturnType = mrover::multiply<U1, U2>;
    };

    template<>
    struct ScalarBinaryOpTraits<
            mrover::Dimensionless,
            mrover::Dimensionless,
            internal::scalar_product_op<mrover::Dimensionless>> {
        using ReturnType = mrover::Dimensionless;
    };

    // template<typename Rep1, typename Conversion1, typename MeterExp1, typename KilogramExp1, typename SecondExp1, typename RadianExp1, typename AmpereExp1, typename KelvinExp1, typename ByteExp1, typename TickExp1>
    // struct NumTraits<mrover::Unit<Rep1, Conversion1, MeterExp1, KilogramExp1, SecondExp1, RadianExp1, AmpereExp1, KelvinExp1, ByteExp1, TickExp1>> : NumTraits<float> {
    //     using U = mrover::Unit<Rep1, Conversion1, MeterExp1, KilogramExp1, SecondExp1, RadianExp1, AmpereExp1, KelvinExp1, ByteExp1, TickExp1>;
    //     using Real = U;
    //     using NonInteger = U;
    //     using Nested = U;
    //     enum {
    //         IsComplex = 0,
    //         IsInteger = 0,
    //         IsSigned = 1,
    //         RequireInitialization = 1,
    //         ReadCost = 1,
    //         AddCost = 3,
    //         MulCost = 3,
    //     };
    // };

} // namespace Eigen

namespace mrover {

    ArmTranslator::ArmTranslator(ros::NodeHandle& nh) {
        for (std::string const& hwName: mArmHWNames) {
            auto [_, was_inserted] = mAdjustClientsByArmHwNames.try_emplace(hwName, nh.serviceClient<AdjustMotor>(std::format("{}_adjust", hwName)));
            assert(was_inserted);
        }

        mThrottleSub = nh.subscribe<Throttle>("arm_throttle_cmd", 1, &ArmTranslator::processThrottleCmd, this);
        mVelocitySub = nh.subscribe<Velocity>("arm_velocity_cmd", 1, &ArmTranslator::processVelocityCmd, this);
        mPositionSub = nh.subscribe<Position>("arm_position_cmd", 1, &ArmTranslator::processPositionCmd, this);
        mJointDataSub = nh.subscribe<sensor_msgs::JointState>("arm_hw_joint_data", 1, &ArmTranslator::processJointState, this);

        mThrottlePub = std::make_unique<ros::Publisher>(nh.advertise<Throttle>("arm_hw_throttle_cmd", 1));
        mVelocityPub = std::make_unique<ros::Publisher>(nh.advertise<Velocity>("arm_hw_velocity_cmd", 1));
        mPositionPub = std::make_unique<ros::Publisher>(nh.advertise<Position>("arm_hw_position_cmd", 1));
        mJointDataPub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::JointState>("arm_joint_data", 1));

        mDeOffsetTimer = nh.createTimer(ros::Duration{1}, &ArmTranslator::updateDeOffsets, this);
    }

    auto static const PITCH_ROLL_TO_0_1 = (Matrix2<Dimensionless>{} << 1, 1, 1, -1).finished();

    auto findJointByName(std::vector<std::string> const& names, std::string const& name) -> std::optional<std::size_t> {
        auto it = std::ranges::find(names, name);
        return it == names.end() ? std::nullopt : std::make_optional(std::distance(names.begin(), it));
    }

    auto ArmTranslator::processThrottleCmd(Throttle::ConstPtr const& msg) const -> void {
        Throttle throttle = *msg;

        // If present, replace the entries for DE0 and DE1 with the pitch and roll values respectively
        std::optional<std::size_t> jointDePitchIndex = findJointByName(msg->names, "joint_de_pitch"), jointDeRollIndex = findJointByName(msg->names, "joint_de_roll");
        if (jointDePitchIndex && jointDeRollIndex) {

            std::size_t pitchIndex = jointDePitchIndex.value(), rollIndex = jointDeRollIndex.value();

            Vector2<Dimensionless> pitchRollThrottles{msg->throttles.at(pitchIndex), msg->throttles.at(rollIndex)};
            Vector2<Dimensionless> motorThrottles = PITCH_ROLL_TO_0_1 * pitchRollThrottles;

            throttle.names[pitchIndex] = "joint_de_0";
            throttle.names[rollIndex] = "joint_de_1";
            throttle.throttles[pitchIndex] = motorThrottles[0].get();
            throttle.throttles[rollIndex] = motorThrottles[1].get();
        }

        mThrottlePub->publish(throttle);
    }

    // constexpr Dimensionless PITCH_ROLL_TO_01_SCALE = 40;
    // Matrix2<Dimensionless> static const PITCH_ROLL_TO_01_SCALED = PITCH_ROLL_TO_0_1 * PITCH_ROLL_TO_01_SCALE;
    // Note (Isabel) PITCH_ROLL_TO_01_SCALE is unnecessary, moteus config will scale for gear ratio

    auto ArmTranslator::processVelocityCmd(Velocity::ConstPtr const& msg) -> void {
        Velocity velocity = *msg;

        // TODO(quintin): Decrease code duplication
        // If present, replace the entries for DE0 and DE1 with the pitch and roll values respectively
        std::optional<std::size_t> jointDePitchIndex = findJointByName(msg->names, "joint_de_pitch"), jointDeRollIndex = findJointByName(msg->names, "joint_de_roll");
        if (jointDePitchIndex && jointDeRollIndex) {
            std::size_t pitchIndex = jointDePitchIndex.value(), rollIndex = jointDeRollIndex.value();

            Vector2<RadiansPerSecond> pitchRollVelocities{msg->velocities.at(pitchIndex), msg->velocities.at(rollIndex)};
            Vector2<RadiansPerSecond> motorVelocities = PITCH_ROLL_TO_0_1 * pitchRollVelocities;

            velocity.names[pitchIndex] = "joint_de_0";
            velocity.names[rollIndex] = "joint_de_1";
            velocity.velocities[pitchIndex] = motorVelocities[0].get();
            velocity.velocities[rollIndex] = motorVelocities[1].get();
        }

        mVelocityPub->publish(velocity);
    }

    auto ArmTranslator::processPositionCmd(Position::ConstPtr const& msg) -> void {
        Position position = *msg;

        // TODO(quintin): Decrease code duplication
        // If present, replace the entries for DE0 and DE1 with the pitch and roll values respectively
        std::optional<std::size_t> jointDePitchIndex = findJointByName(msg->names, "joint_de_pitch"), jointDeRollIndex = findJointByName(msg->names, "joint_de_roll");
        if (jointDePitchIndex && jointDeRollIndex) {
            std::size_t pitchIndex = jointDePitchIndex.value(), rollIndex = jointDeRollIndex.value();

            Vector2<RadiansPerSecond> pitchRoll{msg->positions.at(pitchIndex), msg->positions.at(rollIndex)};
            Vector2<RadiansPerSecond> motorPositions = 40 * PITCH_ROLL_TO_0_1 * pitchRoll;

            position.names[pitchIndex] = "joint_de_0";
            position.names[rollIndex] = "joint_de_1";
            position.positions[pitchIndex] = motorPositions[0].get();
            position.positions[rollIndex] = motorPositions[1].get();
        }

        mPositionPub->publish(position);
    }

    auto wrapAngle(float angle) -> float {
        constexpr float pi = std::numbers::pi_v<float>;
        constexpr float tau = 2 * pi;
        return std::fmod(angle + pi, tau) - pi;
    }

    auto ArmTranslator::processJointState(sensor_msgs::JointState::ConstPtr const& msg) -> void {
        if (mArmHWNames != msg->name || mArmHWNames.size() != msg->position.size() || mArmHWNames.size() != msg->velocity.size() || mArmHWNames.size() != msg->effort.size()) {
            ROS_ERROR("Forwarding joint data for arm is ignored!");
            return;
        }

        sensor_msgs::JointState jointState = *msg;

        std::optional<std::size_t> jointDe0Index = findJointByName(msg->name, "joint_de_0"), jointDe1Index = findJointByName(msg->name, "joint_de_1");
        if (jointDe0Index && jointDe1Index) {
            auto pitchWrapped = -wrapAngle(static_cast<float>(msg->position.at(jointDe0Index.value())));
            auto rollWrapped = wrapAngle(static_cast<float>(msg->position.at(jointDe1Index.value())));
            mJointDePitchRoll = {pitchWrapped, rollWrapped};
            jointState.name[jointDe0Index.value()] = "joint_de_pitch";
            jointState.name[jointDe1Index.value()] = "joint_de_roll";
            jointState.position[jointDe0Index.value()] = pitchWrapped;
            jointState.position[jointDe1Index.value()] = rollWrapped;
        }

        mJointDataPub->publish(jointState);
    }

    auto ArmTranslator::updateDeOffsets(ros::TimerEvent const&) -> void {
        if (!mJointDePitchRoll) return;

        Vector2<Radians> motorPositions = 40 * PITCH_ROLL_TO_0_1 * mJointDePitchRoll.value();
        {
            AdjustMotor adjust;
            adjust.request.name = "joint_de_0";
            adjust.request.value = motorPositions[0].get();
            mAdjustClientsByArmHwNames["joint_de_0"].call(adjust);
        }
        {
            AdjustMotor adjust;
            adjust.request.name = "joint_de_1";
            adjust.request.value = motorPositions[1].get();
            mAdjustClientsByArmHwNames["joint_de_1"].call(adjust);
        }
    }

} // namespace mrover
