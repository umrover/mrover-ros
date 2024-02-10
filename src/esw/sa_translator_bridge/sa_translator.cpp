
// #include "sa_translator.hpp"

// namespace mrover {

//     SATranslator::SATranslator(ros::NodeHandle& nh) {
//         // assert(mJointDEPitchIndex == mJointDE0Index);
//         // assert(mJointDERollIndex == mJointDE1Index);
//         // assert(mArmHWNames.size() == mRawArmNames.size());
//         // for (size_t i = 0; i < mRawArmNames.size(); ++i) {
//         //     if (i != mJointDEPitchIndex && i != mJointDERollIndex) {
//         //         assert(mArmHWNames.at(i) == mRawArmNames.at(i));
//         //     }

//         //     // // adjust and calibrate services
//         //     // std::string rawName = static_cast<std::string>(mRawArmNames[i]);
//         //     // mAdjustServersByRawArmNames[rawName] = std::make_unique<ros::ServiceServer>(nh.advertiseService(std::format("{}_adjust", rawName), &ArmTranslator::adjustServiceCallback, this));

//         //     // std::string hwName = static_cast<std::string>(mArmHWNames[i]);
//         //     // mAdjustClientsByArmHWNames[hwName] = nh.serviceClient<mrover::AdjustMotor>(std::format("{}_adjust", hwName));
//         // }


//         mXAxisMult = RadiansPerMeter{getFloatFromRosParam(nh, "brushed_motors/controllers/sa_x/rad_to_meters_ratio")};
//         mYAxisMult = RadiansPerMeter{getFloatFromRosParam(nh, "brushed_motors/controllers/sa_y/rad_to_meters_ratio")};
//         mZAxisMult = RadiansPerMeter{getFloatFromRosParam(nh, "brushless_motors/controllers/sa_z/rad_to_meters_ratio")};
        
//         mThrottleSub = nh.subscribe<Throttle>("sa_throttle_cmd", 1, &SATranslator::processThrottleCmd, this);
//         mVelocitySub = nh.subscribe<Velocity>("sa_velocity_cmd", 1, &SATranslator::processVelocityCmd, this);
//         mPositionSub = nh.subscribe<Position>("sa_position_cmd", 1, &SATranslator::processPositionCmd, this);
//         mSAHWJointDataSub = nh.subscribe<sensor_msgs::JointState>("sa_hw_joint_data", 1, &SATranslator::processSAHWJointData, this);

//         mThrottlePub = std::make_unique<ros::Publisher>(nh.advertise<Throttle>("sa_hw_throttle_cmd", 1));
//         mVelocityPub = std::make_unique<ros::Publisher>(nh.advertise<Velocity>("sa_hw_throttle_cmd", 1));
//         mPositionPub = std::make_unique<ros::Publisher>(nh.advertise<Position>("sa_hw_throttle_cmd", 1));
//         mJointDataPub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::JointState>("sa_joint_data", 1));
//     }

   
//     void ArmTranslator::processThrottleCmd(Throttle::ConstPtr const& msg) {
//         if (mRawArmNames != msg->names || mRawArmNames.size() != msg->throttles.size()) {
//             ROS_ERROR("Throttle requests for arm is ignored!");
//             return;
//         }

//         Throttle throttle = *msg;
//         ROS_INFO("pitch throttle: %f    roll throttle: %f", msg->throttles.at(mJointDEPitchIndex), msg->throttles.at(mJointDERollIndex));

//         auto [joint_de_0_throttle, joint_de_1_throttle] = transformPitchRollToMotorOutputs(
//                 msg->throttles.at(mJointDEPitchIndex),
//                 msg->throttles.at(mJointDERollIndex));

//         ROS_INFO("pre-mapped values: de_0 %f   de_1 %f", joint_de_0_throttle, joint_de_1_throttle);

//         mapValue(
//                 joint_de_0_throttle,
//                 -80.0f,
//                 80.0f,
//                 -1.0f,
//                 1.0f);

//         mapValue(
//                 joint_de_1_throttle,
//                 -80.0f,
//                 80.0f,
//                 -1.0f,
//                 1.0f);

//         throttle.names.at(mJointDEPitchIndex) = "joint_de_0";
//         throttle.names.at(mJointDERollIndex) = "joint_de_1";
//         throttle.throttles.at(mJointDEPitchIndex) = joint_de_0_throttle;
//         throttle.throttles.at(mJointDERollIndex) = joint_de_1_throttle;

//         ROS_INFO("post-mapped values: de_0 %f   de_1 %f", joint_de_0_throttle, joint_de_1_throttle);

//         mThrottlePub->publish(throttle);
//     }

   

//     void ArmTranslator::processVelocityCmd(Velocity::ConstPtr const& msg) {
//         if (mRawArmNames != msg->names || mRawArmNames.size() != msg->velocities.size()) {
//             ROS_ERROR("Velocity requests for arm is ignored!");
//             return;
//         }

//         Velocity velocity = *msg;

//         auto [joint_de_0_vel, joint_de_1_vel] = transformPitchRollToMotorOutputs(
//                 msg->velocities.at(mJointDEPitchIndex),
//                 msg->velocities.at(mJointDERollIndex));

//         mapValue(
//                 joint_de_0_vel,
//                 -800.0,
//                 800.0,
//                 mMinRadPerSecDE1.get(),
//                 mMaxRadPerSecDE1.get());

//         mapValue(
//                 joint_de_1_vel,
//                 -800.0,
//                 800.0,
//                 mMinRadPerSecDE1.get(),
//                 mMaxRadPerSecDE1.get());

//         ROS_INFO("max velocity: %f", joint_de_0_vel);
//         velocity.names.at(mJointDEPitchIndex) = "joint_de_0";
//         velocity.names.at(mJointDERollIndex) = "joint_de_1";
//         velocity.velocities.at(mJointDEPitchIndex) = joint_de_0_vel;
//         velocity.velocities.at(mJointDERollIndex) = joint_de_1_vel;

//         // joint a convert linear velocity (meters/s) to revolution/s
//         auto joint_a_vel = convertLinVel(msg->velocities.at(mJointAIndex), mJointALinMult.get());
//         velocity.velocities.at(mJointAIndex) = joint_a_vel;

//         mVelocityPub->publish(velocity);
//     }


//     void ArmTranslator::processPositionCmd(Position::ConstPtr const& msg) {
//         if (mRawArmNames != msg->names || mRawArmNames.size() != msg->positions.size()) {
//             ROS_ERROR("Position requests for arm is ignored!");
//             return;
//         }

//         if (!jointDEIsCalibrated()) {
//             ROS_ERROR("Position requests for arm is ignored because jointDEIsNotCalibrated!");
//             return;
//         }

//         Position position = *msg;

//         auto [joint_de_0_raw_pos, joint_de_1_raw_pos] = transformPitchRollToMotorOutputs(
//                 msg->positions.at(mJointDEPitchIndex),
//                 msg->positions.at(mJointDERollIndex));

//         float joint_de_0_pos = joint_de_0_raw_pos + mJointDE0PosOffset->get();
//         float joint_de_1_pos = joint_de_1_raw_pos + mJointDE1PosOffset->get();

//         position.names.at(mJointDEPitchIndex) = "joint_de_0";
//         position.names.at(mJointDERollIndex) = "joint_de_1";
//         position.positions.at(mJointDEPitchIndex) = joint_de_0_pos;
//         position.positions.at(mJointDERollIndex) = joint_de_1_pos;

//         // joint a convert linear position (meters) to radians
//         auto joint_a_pos = convertLinVel(msg->positions.at(mJointAIndex), mJointALinMult.get());
//         position.positions.at(mJointAIndex) = joint_a_pos;

//         mPositionPub->publish(position);
//     }

//     void ArmTranslator::processArmHWJointData(sensor_msgs::JointState::ConstPtr const& msg) {
//         if (mArmHWNames != msg->name || mArmHWNames.size() != msg->position.size() || mArmHWNames.size() != msg->velocity.size() || mArmHWNames.size() != msg->effort.size()) {
//             ROS_ERROR("Forwarding joint data for arm is ignored!");
//             return;
//         }

//         sensor_msgs::JointState jointState = *msg;

//         auto [jointDEPitchVel, jointDERollVel] = transformMotorOutputsToPitchRoll(
//                 static_cast<float>(msg->velocity.at(mJointDE0Index)),
//                 static_cast<float>(msg->velocity.at(mJointDE1Index)));

//         auto [jointDEPitchPos, jointDERollPos] = transformMotorOutputsToPitchRoll(
//                 static_cast<float>(msg->position.at(mJointDE0Index)),
//                 static_cast<float>(msg->position.at(mJointDE1Index)));

//         auto [jointDEPitchEff, jointDERollEff] = transformMotorOutputsToPitchRoll(
//                 static_cast<float>(msg->effort.at(mJointDE0Index)),
//                 static_cast<float>(msg->effort.at(mJointDE1Index)));

//         mCurrentRawJointDE0Position = Radians{msg->position.at(mJointDE0Index)};
//         mCurrentRawJointDE1Position = Radians{msg->position.at(mJointDE1Index)};

//         jointState.name.at(mJointDE0Index) = "joint_de_0";
//         jointState.name.at(mJointDE1Index) = "joint_de_1";
//         jointState.velocity.at(mJointDE0Index) = jointDEPitchVel;
//         jointState.velocity.at(mJointDE1Index) = jointDERollVel;
//         jointState.position.at(mJointDE0Index) = jointDEPitchVel;
//         jointState.position.at(mJointDE1Index) = jointDERollVel;
//         jointState.effort.at(mJointDE0Index) = jointDEPitchEff;
//         jointState.effort.at(mJointDE1Index) = jointDERollEff;

//         mJointDataPub->publish(jointState);
//     }

//     bool ArmTranslator::adjustServiceCallback(AdjustMotor::Request& req, AdjustMotor::Response& res) {

//         if (req.name == "joint_de_roll") {
//             mJointDERollAdjust = req.value;
//         } else if (req.name == "joint_de_pitch") {
//             mJointDEPitchAdjust = req.value;
//         } else {
//             AdjustMotor::Response controllerRes;
//             AdjustMotor::Request controllerReq = req;
//             mAdjustClientsByArmHWNames[req.name].call(controllerReq, controllerRes);
//             res.success = controllerRes.success;
//         }

//         if (mJointDEPitchAdjust && mJointDERollAdjust) {
//             // convert DE_roll and DE_pitch into DE_0 and DE_1 (outgoing message to arm_hw_bridge)
//             auto [joint_de_0_raw_value, joint_de_1_raw_value] = transformPitchRollToMotorOutputs(
//                     mJointDEPitchAdjust.value(),
//                     mJointDERollAdjust.value());
//             mJointDEPitchAdjust = std::nullopt;
//             mJointDERollAdjust = std::nullopt;

//             float joint_de_0_value = joint_de_0_raw_value + mJointDE0PosOffset->get();
//             float joint_de_1_value = joint_de_1_raw_value + mJointDE1PosOffset->get();

//             AdjustMotor::Response controllerResDE0;
//             AdjustMotor::Request controllerReqDE0;
//             controllerReqDE0.name = "joint_de_0";
//             controllerReqDE0.value = joint_de_0_value;
//             mAdjustClientsByArmHWNames[controllerReqDE0.name].call(controllerResDE0, controllerReqDE0);

//             AdjustMotor::Response controllerResDE1;
//             AdjustMotor::Request controllerReqDE1;
//             controllerReqDE1.name = "joint_de_1";
//             controllerReqDE1.value = joint_de_1_value;
//             mAdjustClientsByArmHWNames[controllerReqDE1.name].call(controllerReqDE1, controllerResDE1);

//             res.success = controllerResDE0.success && controllerResDE1.success;
//         } else {
//             // adjust service was for de, but both de joints have not adjusted yet
//             res.success = false;
//         }
//         return true;
//     }


// } // namespace mrover
