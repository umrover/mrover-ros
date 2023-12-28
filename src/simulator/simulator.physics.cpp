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

        mOverlappingPairCache = std::make_unique<btDbvtBroadphase>();

        mSolver = std::make_unique<btMLCPSolver>(new btDantzigSolver{});

        mDynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(mDispatcher.get(), mOverlappingPairCache.get(), mSolver.get(), mCollisionConfig.get());
        mDynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;
    }

    auto SimulatorNodelet::physicsUpdate(Clock::duration dt) -> void {
        mDynamicsWorld->setGravity(mGravityAcceleration);

        for (auto const& name: {"rover#chassis_link_to_left_rocker_link", "rover#chassis_link_to_right_rocker_link"}) {
            auto it = mJointNameToHinges.find(name);
            if (it == mJointNameToHinges.end()) continue;

            btHingeConstraint* hinge = it->second;
            hinge->enableMotor(true);
            hinge->setMaxMotorImpulse(0.45);
            hinge->setMotorTarget(0.0, std::chrono::duration_cast<std::chrono::duration<float>>(dt).count());
        }

        int maxSubSteps = 10;
        int simStepCount = mDynamicsWorld->stepSimulation(std::chrono::duration_cast<std::chrono::duration<float>>(dt).count(), maxSubSteps, 1 / 120.0f);

        // TODO(quintin): Figure out why this fails
        if (auto* mlcpSolver = dynamic_cast<btMLCPSolver*>(mDynamicsWorld->getConstraintSolver())) {
            // if (int fallbackCount = mlcpSolver->getNumFallbacks()) {
            //     NODELET_WARN_STREAM_THROTTLE(1, std::format("MLCP solver failed {} times", fallbackCount));
            // }
            mlcpSolver->setNumFallbacks(0);
        }

        if (simStepCount) {
            if (simStepCount > maxSubSteps) {
                int droppedSteps = simStepCount - maxSubSteps;
                NODELET_WARN_STREAM(std::format("Dropped {} simulation steps out of {}", droppedSteps, simStepCount));
            }
        }

        linksToTfUpdate();

        gpsAndImusUpdate();
    }

    auto SimulatorNodelet::linksToTfUpdate() -> void {

        for (URDF const& urdf: mUrdfs) {

            auto renderLink = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
                if (link->parent_joint) {
                    btRigidBody* parentLinkRb = mLinkNameToRigidBody.at(globalName(urdf.name, link->getParent()->name));
                    btRigidBody* childLinkRb = mLinkNameToRigidBody.at(globalName(urdf.name, link->name));
                    SE3 childInParent = btTransformToSe3(parentLinkRb->getWorldTransform().inverseTimes(childLinkRb->getWorldTransform()));

                    SE3::pushToTfTree(mTfBroadcaster, link->name, link->getParent()->name, childInParent);
                }

                for (urdf::JointSharedPtr const& child_joint: link->child_joints) {
                    self(self, urdf.model.getLink(child_joint->child_link_name));
                }
            };

            renderLink(renderLink, urdf.model.getRoot());
        }
    }

} // namespace mrover
