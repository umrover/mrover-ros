
#include "sa_translator.hpp"

namespace mrover {

    SATranslator::SATranslator(ros::NodeHandle& nh) {


        mXAxisMult = requireParamAsUnit<RadiansPerMeter>(nh, "brushed_motors/controllers/sa_x/rad_to_meters_ratio");
        mYAxisMult = requireParamAsUnit<RadiansPerMeter>(nh, "brushed_motors/controllers/sa_y/rad_to_meters_ratio");
        mZAxisMult = requireParamAsUnit<RadiansPerMeter>(nh, "brushless_motors/controllers/sa_z/rad_to_meters_ratio");
        
        mThrottleSub = nh.subscribe<Throttle>("sa_throttle_cmd", 1, &SATranslator::processThrottleCmd, this);
        mVelocitySub = nh.subscribe<Velocity>("sa_velocity_cmd", 1, &SATranslator::processVelocityCmd, this);
        mPositionSub = nh.subscribe<Position>("sa_position_cmd", 1, &SATranslator::processPositionCmd, this);
        mSAHWJointDataSub = nh.subscribe<sensor_msgs::JointState>("sa_hw_joint_data", 1, &SATranslator::processSAHWJointData, this);

        mThrottlePub = std::make_unique<ros::Publisher>(nh.advertise<Throttle>("sa_hw_throttle_cmd", 1));
        mVelocityPub = std::make_unique<ros::Publisher>(nh.advertise<Velocity>("sa_hw_velocity_cmd", 1));
        mPositionPub = std::make_unique<ros::Publisher>(nh.advertise<Position>("sa_hw_position_cmd", 1));
        mJointDataPub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::JointState>("sa_joint_data", 1));
    }

   
    void SATranslator::processThrottleCmd(Throttle::ConstPtr const& msg) {

        Throttle throttle = *msg;

        mThrottlePub->publish(throttle);
    }

   

    void SATranslator::processVelocityCmd(Velocity::ConstPtr const& msg) {
        if (mSAHWNames != msg->names || mSAHWNames.size() != msg->velocities.size()) {
            ROS_ERROR("Velocity requests for SA is ignored!");
            return;
        }

        Velocity velocity = *msg;

        // joint a convert linear velocity (meters/s) to revolution/s
        auto x_axis_vel = static_cast<float>(msg->velocities.at(mXAxisIndex)) *mXAxisMult.get();
        velocity.velocities.at(mXAxisIndex) = x_axis_vel;

        auto y_axis_vel = static_cast<float>(msg->velocities.at(mYAxisIndex)) * mYAxisMult.get();
        velocity.velocities.at(mYAxisIndex) = y_axis_vel;

        auto z_axis_vel = static_cast<float>(msg->velocities.at(mZAxisIndex)) * mZAxisMult.get();
        velocity.velocities.at(mZAxisIndex) = z_axis_vel; 

        mVelocityPub->publish(velocity);
    }


    void SATranslator::processPositionCmd(Position::ConstPtr const& msg) {
        if (mSAHWNames != msg->names || mSAHWNames.size() != msg->positions.size()) {
            ROS_ERROR("Position requests for SA is ignored!");
            return;
        }

        Position position = *msg;

        // joint a convert linear velocity (meters/s) to revolution/s
        auto x_axis_pos = static_cast<float>(msg->positions.at(mXAxisIndex)) * mXAxisMult.get();
        position.positions.at(mXAxisIndex) = x_axis_pos;

        auto y_axis_pos = static_cast<float>(msg->positions.at(mYAxisIndex)) * mYAxisMult.get();
        position.positions.at(mYAxisIndex) = y_axis_pos;

        auto z_axis_pos = static_cast<float>(msg->positions.at(mZAxisIndex)) * mZAxisMult.get();
        position.positions.at(mZAxisIndex) = z_axis_pos;                

        mPositionPub->publish(position);
    }

    void SATranslator::processSAHWJointData(sensor_msgs::JointState::ConstPtr const& msg) {
        if (mSAHWNames != msg->name || mSAHWNames.size() != msg->position.size() || mSAHWNames.size() != msg->velocity.size() || mSAHWNames.size() != msg->effort.size()) {
            ROS_ERROR("Position requests for SA is ignored!");
            return;
        }

        // Convert joint  state of joint a from radians/revolutions to meters
        auto xAxisLinVel = static_cast<float>(msg->velocity.at(mXAxisIndex))* mXAxisMult.get();
        auto xAxisLinPos = static_cast<float>(msg->position.at(mXAxisIndex))* mXAxisMult.get();

        auto yAxisLinVel = static_cast<float>(msg->velocity.at(mYAxisIndex))* mYAxisMult.get();
        auto yAxisLinPos = static_cast<float>(msg->position.at(mYAxisIndex))* mYAxisMult.get();

        auto zAxisLinVel = static_cast<float>(msg->velocity.at(mZAxisIndex))* mZAxisMult.get();
        auto zAxisLinPos = static_cast<float>(msg->position.at(mZAxisIndex))* mZAxisMult.get();

        sensor_msgs::JointState jointState = *msg;
        jointState.velocity.at(mXAxisIndex) = xAxisLinVel;
        jointState.position.at(mXAxisIndex) = xAxisLinPos;
        jointState.velocity.at(mYAxisIndex) = yAxisLinVel;
        jointState.position.at(mYAxisIndex) = yAxisLinPos;
        jointState.velocity.at(mZAxisIndex) = zAxisLinVel;
        jointState.position.at(mZAxisIndex) = zAxisLinPos;

        mJointDataPub->publish(jointState);
    }

} // namespace mrover
