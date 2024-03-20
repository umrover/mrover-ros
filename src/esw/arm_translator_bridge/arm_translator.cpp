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
        assert(mJointDEPitchIndex == mJointDE0Index);
        assert(mJointDERollIndex == mJointDE1Index);
        assert(mArmHWNames.size() == mRawArmNames.size());

        for (std::size_t i = 0; i < mRawArmNames.size(); ++i) {
            if (i != mJointDEPitchIndex && i != mJointDERollIndex) {
                assert(mArmHWNames.at(i) == mRawArmNames.at(i));
            }
            auto hwName = static_cast<std::string>(mArmHWNames[i]);
            auto [_, was_inserted] = mAdjustClientsByArmHwNames.try_emplace(hwName, nh.serviceClient<AdjustMotor>(std::format("{}_adjust", hwName)));
            assert(was_inserted);
        }

        mJointARadiansToMeters = requireParamAsUnit<RadiansPerMeter>(nh, "brushless_motors/controllers/joint_a/rad_to_meters_ratio");

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

    Eigen::Matrix2<Dimensionless> static const PITCH_ROLL_TO_0_1{{1, 1},
                                                                 {1, -1}};

    auto ArmTranslator::processThrottleCmd(Throttle::ConstPtr const& msg) const -> void {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->throttles.size()) {
            ROS_ERROR("Throttle requests for arm is ignored!");
            return;
        }

        Eigen::Vector2<Dimensionless> pitchRollThrottles{msg->throttles.at(mJointDEPitchIndex), msg->throttles.at(mJointDERollIndex)};
        Eigen::Vector2<Dimensionless> motorThrottles = PITCH_ROLL_TO_0_1 * pitchRollThrottles;

        Throttle throttle = *msg;
        throttle.names[mJointDEPitchIndex] = "joint_de_0";
        throttle.names[mJointDERollIndex] = "joint_de_1";
        throttle.throttles[mJointDEPitchIndex] = motorThrottles[0].get();
        throttle.throttles[mJointDERollIndex] = motorThrottles[1].get();
        mThrottlePub->publish(throttle);
    }

    constexpr Dimensionless PITCH_ROLL_TO_01_SCALE = 40;
    Eigen::Matrix2<Dimensionless> static const PITCH_ROLL_TO_01_SCALED = PITCH_ROLL_TO_0_1 * PITCH_ROLL_TO_01_SCALE;

    auto ArmTranslator::processVelocityCmd(Velocity::ConstPtr const& msg) -> void {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->velocities.size()) {
            ROS_ERROR("Velocity requests for arm is ignored!");
            return;
        }

        Eigen::Vector2<RadiansPerSecond> pitchRollVelocities{msg->velocities.at(mJointDEPitchIndex), msg->velocities.at(mJointDERollIndex)};
        Eigen::Vector2<RadiansPerSecond> motorVelocities = PITCH_ROLL_TO_01_SCALED * pitchRollVelocities;
        // Eigen::Vector2<RadiansPerSecond> motorVelocities = PITCH_ROLL_TO_0_1 * pitchRollVelocities;

        Velocity velocity = *msg;
        velocity.names[mJointDEPitchIndex] = "joint_de_0";
        velocity.names[mJointDERollIndex] = "joint_de_1";
        velocity.velocities[mJointDEPitchIndex] = motorVelocities[0].get();
        velocity.velocities[mJointDERollIndex] = motorVelocities[1].get();
        RadiansPerSecond jointAVelocity = MetersPerSecond{msg->velocities[mJointAIndex]} * mJointARadiansToMeters;
        velocity.velocities[mJointAIndex] = jointAVelocity.get();
        mVelocityPub->publish(velocity);
    }

    auto ArmTranslator::processPositionCmd(Position::ConstPtr const& msg) -> void {
        if (mRawArmNames != msg->names || mRawArmNames.size() != msg->positions.size()) {
            ROS_ERROR("Position requests for arm is ignored!");
            return;
        }

        Eigen::Vector2<RadiansPerSecond> pitchRoll{msg->positions.at(mJointDEPitchIndex), msg->positions.at(mJointDERollIndex)};
        Eigen::Vector2<RadiansPerSecond> motorPositions = PITCH_ROLL_TO_01_SCALED * pitchRoll;

        Position position = *msg;
        position.names[mJointDEPitchIndex] = "joint_de_0";
        position.names[mJointDERollIndex] = "joint_de_1";
        position.positions[mJointDEPitchIndex] = motorPositions[0].get();
        position.positions[mJointDERollIndex] = motorPositions[1].get();
        RadiansPerSecond jointAPosition = MetersPerSecond{msg->positions[mJointAIndex]} * mJointARadiansToMeters;
        position.positions[mJointAIndex] = jointAPosition.get();
        mPositionPub->publish(position);
    }

    auto ArmTranslator::processJointState(sensor_msgs::JointState::ConstPtr const& msg) -> void {
        if (mArmHWNames != msg->name || mArmHWNames.size() != msg->position.size() || mArmHWNames.size() != msg->velocity.size() || mArmHWNames.size() != msg->effort.size()) {
            ROS_ERROR("Forwarding joint data for arm is ignored!");
            return;
        }

        // Convert joint state of joint a from radians/s to meters/s
        MetersPerSecond jointALinearVelocity = RadiansPerSecond{msg->velocity.at(mJointAIndex)} / mJointARadiansToMeters;
        Meters jointAPosition = Radians{msg->position.at(mJointAIndex)} / mJointARadiansToMeters;

        mJointDePitchRoll = {
                msg->position.at(mJointDE0Index),
                msg->position.at(mJointDE1Index),
        };

        sensor_msgs::JointState jointState = *msg;
        jointState.velocity[mJointAIndex] = jointALinearVelocity.get();
        jointState.position[mJointAIndex] = jointAPosition.get();
        mJointDataPub->publish(jointState);
    }

    auto ArmTranslator::updateDeOffsets(ros::TimerEvent const&) -> void {
        if (!mJointDePitchRoll) return;

        Eigen::Matrix2<Radians> motorPositions = PITCH_ROLL_TO_01_SCALED * mJointDePitchRoll.value();
        {
            AdjustMotorRequest request;
            request.name = "joint_de_0";
            request.value = motorPositions[0].get();
            mAdjustClientsByArmHwNames["joint_de_0"].call(request);
        }
        {
            AdjustMotorRequest request;
            request.name = "joint_de_1";
            request.value = motorPositions[1].get();
            mAdjustClientsByArmHwNames["joint_de_1"].call(request);
        }
    }

} // namespace mrover
