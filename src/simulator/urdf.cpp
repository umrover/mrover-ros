#include "simulator.hpp"

namespace mrover {

    URDF::URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init) {
        auto paramName = xmlRpcValueToTypeOrDefault<std::string>(init, "param_name");
        if (!model.initParam(paramName)) throw std::runtime_error{std::format("Failed to parse URDF from param: {}", paramName)};

        for (auto const& [link_name, link]: model.links_) {

            if (link->visual && link->visual->geometry && link->visual->geometry->type == urdf::Geometry::MESH) {
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
                std::string const& uri = mesh->filename;
                simulator.mUriToMesh.try_emplace(uri, uri);
            }

            if (link->collision && link->collision->geometry && link->collision->geometry->type == urdf::Geometry::BOX) {
                auto box = std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);

                btVector3 boxHalfExtents{static_cast<btScalar>(box->dim.x / 2), static_cast<btScalar>(box->dim.y / 2), static_cast<btScalar>(box->dim.z / 2)};
                auto& boxShape = simulator.mCollisionShapes.emplace_back(std::make_unique<btBoxShape>(boxHalfExtents));
                auto& motionState = simulator.mMotionStates.emplace_back(std::make_unique<btDefaultMotionState>(btTransform{btQuaternion{0, 0, 0, 1}, btVector3{0, 0, 0}}));
                auto mass = static_cast<btScalar>(link->inertial->mass);
                btVector3 boxInertia{static_cast<btScalar>(link->inertial->ixx), static_cast<btScalar>(link->inertial->iyy), static_cast<btScalar>(link->inertial->izz)};
                auto& rb = simulator.mCollisionObjects.emplace_back(std::make_unique<btRigidBody>(btRigidBody::btRigidBodyConstructionInfo{mass, motionState.get(), boxShape.get(), boxInertia}));
                simulator.mPhysicsWorld->addRigidBody(dynamic_cast<btRigidBody*>(rb.get()));
            }
        }
    }

}
