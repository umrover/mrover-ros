#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::initPhysics() -> void {
        mCollisionConfig = std::make_unique<btDefaultCollisionConfiguration>();

        mDispatcher = std::make_unique<btCollisionDispatcher>(mCollisionConfig.get());

        mOverlappingPairCache = std::make_unique<btDbvtBroadphase>();

        mSolver = std::make_unique<btSequentialImpulseConstraintSolver>();

        mWorld = std::make_unique<btDiscreteDynamicsWorld>(mDispatcher.get(), mOverlappingPairCache.get(), mSolver.get(), mCollisionConfig.get());
    }

} // namespace mrover
