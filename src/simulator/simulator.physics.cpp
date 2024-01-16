#include "simulator.hpp"

namespace mrover {

    auto btTransformToSe3(btTransform const& transform) -> SE3 {
        btVector3 const& p = transform.getOrigin();
        btQuaternion const& q = transform.getRotation();
        return SE3{R3{p.x(), p.y(), p.z()}, SO3{q.w(), q.x(), q.y(), q.z()}};
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
                auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(rover.linkNameToIndex.at(name)).m_userPtr);
                motor->setMaxAppliedImpulse(0.5);
                motor->setPositionTarget(0);
            }
        }

        // TODO(quintin): clean this up
        int maxSubSteps = 0;
        int simStepCount = mDynamicsWorld->stepSimulation(std::chrono::duration_cast<std::chrono::duration<float>>(dt).count(), maxSubSteps, 1 / 120.0f);

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

        gpsAndImusUpdate();
    }

    auto SimulatorNodelet::linksToTfUpdate() -> void {

        for (auto const& [_, urdf]: mUrdfs) {

            auto publishLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                if (link->parent_joint) {
                    int index = urdf.linkNameToIndex.at(link->name);
                    // TODO(quintin): figure out why we need to negate rvector
                    btTransform parentToChild{urdf.physics->getParentToLocalRot(index), -urdf.physics->getRVector(index)};
                    SE3 childInParent = btTransformToSe3(parentToChild.inverse());

                    SE3::pushToTfTree(mTfBroadcaster, link->name, link->getParent()->name, childInParent);
                }

                for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
                    self(self, urdf.model.getLink(child_joint->child_link_name));
                }
            };

            publishLink(publishLink, urdf.model.getRoot());
        }
    }

    auto URDF::linkInWorld(std::string const& linkName) const -> SE3 {
        int index = linkNameToIndex.at(linkName);
        return btTransformToSe3(index == -1 ? physics->getBaseWorldTransform() : physics->getLink(index).m_cachedWorldTransform);
    }

} // namespace mrover
