#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::initPhysics() -> void {
        NODELET_INFO_STREAM(std::format("Using Bullet Physics Version: {}", btGetVersion()));

        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();

        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());

        mOverlappingPairCache = std::make_unique<btDbvtBroadphase>();

        mSolver = std::make_unique<btMLCPSolver>(new btDantzigSolver{});

        // mSolver = std::make_unique<btSequentialImpulseConstraintSolver>();

        mDynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(mDispatcher.get(), mOverlappingPairCache.get(), mSolver.get(), mCollisionConfig.get());
        mDynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;

        btVector3 planeNormal(0, 0, 1);
        btScalar planeConstant = -1;
        auto* planeShape = new btStaticPlaneShape(planeNormal, planeConstant);

        auto* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));

        btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0, motionState, planeShape);
        auto* rigidBody = new btRigidBody(rigidBodyCI);

        mDynamicsWorld->addRigidBody(rigidBody);
    }

    auto SimulatorNodelet::physicsUpdate(ros::Rate const& rate) -> void {
        mDynamicsWorld->setGravity(mGravityAcceleration);

        auto timeStep = static_cast<btScalar>(rate.expectedCycleTime().toSec());
        int maxSubSteps = 2;
        int simStepCount = mDynamicsWorld->stepSimulation(timeStep, maxSubSteps, 1 / 120.0f);

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
