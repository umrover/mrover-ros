#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::initPhysics() -> void {
        NODELET_INFO_STREAM(std::format("Using Bullet Physics Version: {}", btGetVersion()));

        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();

        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());

        mOverlappingPairCache = std::make_unique<btDbvtBroadphase>();

        mSolver = std::make_unique<btSequentialImpulseConstraintSolver>();

        mDynamicsWorld = std::make_unique<btDiscreteDynamicsWorld>(mDispatcher.get(), mOverlappingPairCache.get(), mSolver.get(), mCollisionConfig.get());
        mDynamicsWorld->setGravity(btVector3{0, 0, 0});

        // Step 1: Create a btStaticPlaneShape object
        btVector3 planeNormal(0, 0, 1); // The plane normal points upwards
        btScalar planeConstant = -1;    // The plane passes through the origin
        btStaticPlaneShape* planeShape = new btStaticPlaneShape(planeNormal, planeConstant);

        // Step 2: Create a btDefaultMotionState object
        btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));

        // Step 3: Create a btRigidBody object
        btScalar mass = 0; // The mass is 0 because the ground is static
        btVector3 inertia(0, 0, 0);
        planeShape->calculateLocalInertia(mass, inertia);
        btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, motionState, planeShape, inertia);
        btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);

        // Step 4: Add the rigid body to the btDiscreteDynamicsWorld object
        mDynamicsWorld->addRigidBody(rigidBody);
    }

    auto SimulatorNodelet::physicsUpdate() -> void {
        mDynamicsWorld->stepSimulation(1.0f / 300, 10);
    }

} // namespace mrover
