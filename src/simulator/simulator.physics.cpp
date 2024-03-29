#include "simulator.hpp"

namespace mrover {

    auto btTransformToSe3(btTransform const& transform) -> SE3d {
        btVector3 const& p = transform.getOrigin();
        btQuaternion const& q = transform.getRotation();
        // Note: Must convert the Bullet quaternion (floats) to a normalized Eigen quaternion (doubles).
        //       Otherwise the normality check will fail in the SO3 constructor.
        return SE3d{R3{p.x(), p.y(), p.z()}, Eigen::Quaterniond{q.w(), q.x(), q.y(), q.z()}.normalized()};
    }

    auto SimulatorNodelet::initPhysics() -> void {
        NODELET_INFO_STREAM(std::format("Using Bullet Physics Version: {}", btGetVersion()));

        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();

        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());

        mOverlappingPairCache = std::make_unique<btHashedOverlappingPairCache>();

        mBroadphase = std::make_unique<btDbvtBroadphase>(mOverlappingPairCache.get());

        // mSolver = std::make_unique<btMultiBodyMLCPConstraintSolver>(new btDantzigSolver{});
        mSolver = std::make_unique<btMultiBodyConstraintSolver>();

        mDynamicsWorld = std::make_unique<btMultiBodyDynamicsWorld>(mDispatcher.get(), mBroadphase.get(), mSolver.get(), mCollisionConfig.get());
        // mDynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;
    }

    auto SimulatorNodelet::physicsUpdate(Clock::duration dt) -> void {
        mDynamicsWorld->setGravity(mGravityAcceleration);

        // TODO(quintin): clean this up
        if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
            URDF const& rover = it->second;

            for (auto const& name: {"left_rocker_link", "right_rocker_link"}) {
                int linkIndex = rover.linkNameToMeta.at(name).index;
                auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(linkIndex).m_userPtr);
                motor->setMaxAppliedImpulse(0.5);
                motor->setPositionTarget(0);
            }
        }

        // TODO(quintin): clean this up
        int maxSubSteps = 0;
        float updateDuration = std::clamp(std::chrono::duration_cast<std::chrono::duration<float>>(dt).count(), 0.0f, 0.1f);
        [[maybe_unused]] int simStepCount = mDynamicsWorld->stepSimulation(updateDuration, maxSubSteps, 1 / 120.0f);

        // // TODO(quintin): Figure out why this fails
        // if (auto* mlcpSolver = dynamic_cast<btMLCPSolver*>(mDynamicsWorld->getConstraintSolver())) {
        //     // if (int fallbackCount = mlcpSolver->getNumFallbacks()) {
        //     //     NODELET_WARN_STREAM_THROTTLE(1, std::format("MLCP solver failed {} times", fallbackCount));
        //     // }
        //     mlcpSolver->setNumFallbacks(0);
        // }

        // if (simStepCount) {
        //     if (simStepCount > maxSubSteps) {
        //         int droppedSteps = simStepCount - maxSubSteps;
        //         NODELET_WARN_STREAM(std::format("Dropped {} simulation steps out of {}", droppedSteps, simStepCount));
        //     }
        // }

        linksToTfUpdate();

        gpsAndImusUpdate(dt);

        motorStatusUpdate();
    }

    auto SimulatorNodelet::linksToTfUpdate() -> void {

        for (auto const& [name, urdf]: mUrdfs) {
            auto publishLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                if (link->parent_joint) {
                    int index = urdf.linkNameToMeta.at(link->name).index;
                    // TODO(quintin): figure out why we need to negate rvector
                    btTransform parentToChild{urdf.physics->getParentToLocalRot(index), -urdf.physics->getRVector(index)};
                    SE3d childInParent = btTransformToSe3(parentToChild.inverse());

                    SE3Conversions::pushToTfTree(mTfBroadcaster, link->name, link->getParent()->name, childInParent);
                }
                // TODO(quintin): This is kind of hacky
                if (name.contains("tag"sv) || name.contains("hammer"sv) || name.contains("bottle"sv)) {
                    SE3d modelInMap = btTransformToSe3(urdf.physics->getBaseWorldTransform());
                    SE3Conversions::pushToTfTree(mTfBroadcaster, std::format("{}_truth", name), "map", modelInMap);
                }

                for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
                    self(self, urdf.model.getLink(child_joint->child_link_name));
                }
            };
            publishLink(publishLink, urdf.model.getRoot());
        }

        if (auto roverOpt = getUrdf("rover")) {
            URDF const& rover = *roverOpt;

            auto publishModel = [&](std::string const& modelName, double threshold) {
                if (auto modelOpt = getUrdf(modelName)) {
                    URDF const& model = *modelOpt;

                    SE3d modelInMap = btTransformToSe3(model.physics->getBaseWorldTransform());
                    SE3d roverInMap = btTransformToSe3(rover.physics->getBaseWorldTransform());

                    R3 roverToModel = modelInMap.translation() - roverInMap.translation();
                    double roverDistanceToModel = roverToModel.norm();
                    roverToModel /= roverDistanceToModel;
                    R3 roverForward = roverInMap.rotation().matrix().col(0);
                    double roverDotModel = roverToModel.dot(roverForward);

                    if (roverDotModel > 0 && roverDistanceToModel < threshold) {
                        SE3Conversions::pushToTfTree(mTfBroadcaster, modelName, "map", modelInMap);
                    }
                }
            };

            if (mPublishBottleDistanceThreshold > 0) publishModel("bottle", mPublishBottleDistanceThreshold);
            if (mPublishHammerDistanceThreshold > 0) publishModel("hammer", mPublishHammerDistanceThreshold);
        }
    }

    auto URDF::linkInWorld(std::string const& linkName) const -> SE3d {
        int index = linkNameToMeta.at(linkName).index;
        return btTransformToSe3(index == -1 ? physics->getBaseWorldTransform() : physics->getLink(index).m_cachedWorldTransform);
    }

} // namespace mrover