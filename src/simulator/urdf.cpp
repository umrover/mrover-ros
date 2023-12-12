#include "simulator.hpp"

namespace mrover {

    auto urdfVec3ToBtVec3(urdf::Vector3 const& vec) -> btVector3 {
        return btVector3{static_cast<btScalar>(vec.x), static_cast<btScalar>(vec.y), static_cast<btScalar>(vec.z)};
    }

    auto urdfQuatToBtQuat(urdf::Rotation const& quat) -> btQuaternion {
        return btQuaternion{static_cast<btScalar>(quat.x), static_cast<btScalar>(quat.y), static_cast<btScalar>(quat.z), static_cast<btScalar>(quat.w)};
    }

    auto urdfPoseToBtTransform(urdf::Pose const& pose) -> btTransform {
        return btTransform{urdfQuatToBtQuat(pose.rotation), urdfVec3ToBtVec3(pose.position)};
    }

    URDF::URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init) {
        auto paramName = xmlRpcValueToTypeOrDefault<std::string>(init, "param_name");
        if (!model.initParam(paramName)) throw std::runtime_error{std::format("Failed to parse URDF from param: {}", paramName)};

        auto traverse = [&](auto&& self, btRigidBody* parentLinkRb, urdf::LinkConstSharedPtr const& link) -> void {
            if (link->visual && link->visual->geometry && link->visual->geometry->type == urdf::Geometry::MESH) {
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
                std::string const& uri = mesh->filename;
                simulator.mUriToMesh.try_emplace(uri, uri);
            }

            btRigidBody::btRigidBodyConstructionInfo rbCreateInfo{0, nullptr, nullptr};
            if (link->collision && link->collision->geometry) {
                switch (link->collision->geometry->type) {
                    case urdf::Geometry::BOX: {
                        auto box = std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);
                        assert(box);

                        btVector3 boxHalfExtents{static_cast<btScalar>(box->dim.x / 2), static_cast<btScalar>(box->dim.y / 2), static_cast<btScalar>(box->dim.z / 2)};
                        auto* boxShape = simulator.makeBulletObject<btBoxShape>(simulator.mCollisionShapes, boxHalfExtents);
                        rbCreateInfo.m_collisionShape = boxShape;

                        rbCreateInfo.m_motionState = simulator.makeBulletObject<btDefaultMotionState>(simulator.mMotionStates, btTransform{btQuaternion{0, 0, 0, 1}, btVector3{0, 0, 0}});

                        auto mass = static_cast<btScalar>(link->inertial->mass);
                        rbCreateInfo.m_mass = mass;

                        btVector3 boxInertia{static_cast<btScalar>(link->inertial->ixx), static_cast<btScalar>(link->inertial->iyy), static_cast<btScalar>(link->inertial->izz)};
                        boxShape->calculateLocalInertia(mass, boxInertia);
                    }
                    default:
                        break;
                }
            }
            auto* linkRb = simulator.makeBulletObject<btRigidBody>(simulator.mCollisionObjects, rbCreateInfo);
            simulator.mPhysicsWorld->addRigidBody(linkRb);

            // ReSharper disable once CppDFAConstantConditions
            if (parentLinkRb) {
                // ReSharper disable once CppDFAUnreachableCode
                urdf::JointConstSharedPtr parentJoint = link->parent_joint;
                btTransform jointTransform = urdfPoseToBtTransform(parentJoint->parent_to_joint_origin_transform);

                btTypedConstraint* constraint;
                switch (parentJoint->type) {
                    case urdf::Joint::FIXED: {
                        constraint = simulator.makeBulletObject<btFixedConstraint>(simulator.mConstraints, *parentLinkRb, *linkRb, jointTransform, btTransform{});
                        break;
                    }
                    case urdf::Joint::REVOLUTE: {
                        btVector3 axis = urdfVec3ToBtVec3(parentJoint->axis);
                        auto* hingeConstraint = simulator.makeBulletObject<btHingeConstraint>(simulator.mConstraints, *parentLinkRb, *linkRb, jointTransform.getOrigin(), btVector3{}, btVector3{}, axis);
                        if (parentJoint->limits) {
                            hingeConstraint->setLimit(static_cast<btScalar>(parentJoint->limits->lower), static_cast<btScalar>(parentJoint->limits->upper));
                        }
                        constraint = hingeConstraint;
                        break;
                    }
                    default:
                        throw std::invalid_argument{"Unsupported joint type"};
                }
                simulator.mPhysicsWorld->addConstraint(constraint);
            }

            for (urdf::LinkConstSharedPtr childLink: link->child_links) {
                self(self, linkRb, childLink);
            }
        };

        traverse(traverse, nullptr, model.getRoot());
    }

}
