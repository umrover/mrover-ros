#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::initPhysics() -> void {
        NODELET_INFO_STREAM(std::format("Using Bullet Physics Version: {}", btGetVersion()));

        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();

        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());

        mOverlappingPairCache = std::make_unique<btDbvtBroadphase>();

        mSolver = std::make_unique<btMLCPSolver>(new btDantzigSolver{});

        mDynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(mDispatcher.get(), mOverlappingPairCache.get(), mSolver.get(), mCollisionConfig.get());
        mDynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;
    }

    auto SimulatorNodelet::physicsUpdate(ros::Rate const& rate) -> void {
        mDynamicsWorld->setGravity(mGravityAcceleration);

        auto dt = static_cast<btScalar>(rate.expectedCycleTime().toSec());

        for (auto const& name: {"rover#chassis_link_to_left_rocker_link", "rover#chassis_link_to_right_rocker_link"}) {
            btHingeConstraint* hinge = mJointNameToHinges.at(name);
            hinge->enableMotor(true);
            hinge->setMaxMotorImpulse(0.45);
            hinge->setMotorTarget(0.0, dt);
        }

        int maxSubSteps = 10;
        int simStepCount = mDynamicsWorld->stepSimulation(dt, maxSubSteps, 1 / 120.0f);

        if (auto* mlcpSolver = dynamic_cast<btMLCPSolver*>(mDynamicsWorld->getConstraintSolver())) {
            if (int fallbackCount = mlcpSolver->getNumFallbacks()) {
                NODELET_WARN_STREAM_THROTTLE(1, std::format("MLCP solver failed {} times", fallbackCount));
            }
            mlcpSolver->setNumFallbacks(0);
        }

        if (simStepCount) {
            if (simStepCount > maxSubSteps) {
                int droppedSteps = simStepCount - maxSubSteps;
                NODELET_WARN_STREAM(std::format("Dropped {} simulation steps out of {}", droppedSteps, simStepCount));
            }
        }
    }

} // namespace mrover
