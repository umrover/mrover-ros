#include "simulator.hpp"

namespace mrover {

    constexpr static float GRAVITY_Z_ACCELERATION = -9.81f;

    auto SimulatorNodelet::initPhysics() -> void {
        NODELET_INFO_STREAM(std::format("Using Bullet Physics Version: {}", btGetVersion()));

        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();

        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());

        mOverlappingPairCache = std::make_unique<btDbvtBroadphase>();

        mSolver = std::make_unique<btMLCPSolver>(new btDantzigSolver{});

        // mSolver = std::make_unique<btSequentialImpulseConstraintSolver>();

        mDynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(mDispatcher.get(), mOverlappingPairCache.get(), mSolver.get(), mCollisionConfig.get());
        mDynamicsWorld->setGravity(btVector3{0, 0, GRAVITY_Z_ACCELERATION});
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
        // for (auto const& name: {"center_left_wheel_joint", "center_right_wheel_joint", "front_left_wheel_joint", "front_right_wheel_joint", "back_left_wheel_joint", "back_right_wheel_joint"}) {
        //     btHingeConstraint* hinge = mJointNameToHinges.at(name);
        //     hinge->enableAngularMotor(true, mFloat1, 2.68);
        // }

        auto timeStep = static_cast<btScalar>(rate.expectedCycleTime().toSec());
        int simStepCount = mDynamicsWorld->stepSimulation(timeStep, 20, 1 / 120.0f);

        // if (auto* mlcpSolver = dynamic_cast<btMLCPSolver*>(mDynamicsWorld->getConstraintSolver())) {
        //     if (int fallbackCount = mlcpSolver->getNumFallbacks()) {
        //         NODELET_WARN_STREAM(std::format("MLCP solver failed {} times {}", fallbackCount, simStepCount));
        //     }
        //     mlcpSolver->setNumFallbacks(0);
        // }
    }

} // namespace mrover
