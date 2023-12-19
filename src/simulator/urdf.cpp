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
        auto urdfUri = xmlRpcValueToTypeOrDefault<std::string>(init, "uri");
        if (!model.initString(performXacro(uriToPath(urdfUri)))) throw std::runtime_error{std::format("Failed to parse URDF from URI: {}", urdfUri)};

        auto traverse = [&](auto&& self, btRigidBody* parentLinkRb, urdf::LinkConstSharedPtr const& link) -> void {
            ROS_INFO_STREAM(std::format("Adding link: {}", link->name));
            if (link->visual && link->visual->geometry && link->visual->geometry->type == urdf::Geometry::MESH) {
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
                std::string const& fileUri = mesh->filename;
                simulator.mUriToMesh.try_emplace(fileUri, fileUri);
            }

            btCollisionShape* shape = nullptr;
            btScalar mass = 0.001;
            btVector3 inertia{0.001, 0.001, 0.001};
            if (link->collision && link->collision->geometry) {
                switch (link->collision->geometry->type) {
                    case urdf::Geometry::BOX: {
                        auto box = std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);
                        assert(box);

                        btVector3 boxHalfExtents{static_cast<btScalar>(box->dim.x / 2), static_cast<btScalar>(box->dim.y / 2), static_cast<btScalar>(box->dim.z / 2)};
                        shape = simulator.makeBulletObject<btBoxShape>(simulator.mCollisionShapes, boxHalfExtents);
                        break;
                    }
                    case urdf::Geometry::SPHERE: {
                        auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(link->collision->geometry);
                        assert(sphere);

                        shape = simulator.makeBulletObject<btSphereShape>(simulator.mCollisionShapes, static_cast<btScalar>(sphere->radius));
                        break;
                    }
                    case urdf::Geometry::CYLINDER: {
                        auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(link->collision->geometry);
                        assert(cylinder);

                        btVector3 cylinderHalfExtents{static_cast<btScalar>(cylinder->radius), static_cast<btScalar>(cylinder->radius), static_cast<btScalar>(cylinder->length / 2)};
                        shape = simulator.makeBulletObject<btCylinderShapeZ>(simulator.mCollisionShapes, cylinderHalfExtents);
                        break;
                    }
                    default:
                        throw std::invalid_argument{"Unsupported collision type"};
                }
            } else {
                shape = simulator.makeBulletObject<btEmptyShape>(simulator.mCollisionShapes);
            }
            if (link->inertial) {
                mass = static_cast<btScalar>(link->inertial->mass);
                inertia = {static_cast<btScalar>(link->inertial->ixx), static_cast<btScalar>(link->inertial->iyy), static_cast<btScalar>(link->inertial->izz)};
            }
            btTransform jointInWorld;
            if (link->parent_joint) {
                assert(parentLinkRb);

                auto parentToWorld = parentLinkRb->getWorldTransform();
                auto jointInParent = urdfPoseToBtTransform(link->parent_joint->parent_to_joint_origin_transform);
                jointInWorld = parentToWorld * jointInParent;
            } else {
                jointInWorld = btTransform::getIdentity();
            }
            auto* motionState = simulator.makeBulletObject<btDefaultMotionState>(simulator.mMotionStates, jointInWorld);
            auto* linkRb = simulator.makeBulletObject<btRigidBody>(simulator.mCollisionObjects, btRigidBody::btRigidBodyConstructionInfo{mass, motionState, shape, inertia});
            simulator.mLinkNameToRigidBody.emplace(link->name, linkRb);
            linkRb->setActivationState(DISABLE_DEACTIVATION);
            simulator.mDynamicsWorld->addRigidBody(linkRb);

            if (urdf::JointConstSharedPtr parentJoint = link->parent_joint) {
                assert(parentLinkRb);

                btTransform jointInParent = urdfPoseToBtTransform(parentJoint->parent_to_joint_origin_transform);

                btTypedConstraint* constraint;
                switch (parentJoint->type) {
                    case urdf::Joint::FIXED: {
                        constraint = simulator.makeBulletObject<btFixedConstraint>(simulator.mConstraints, *parentLinkRb, *linkRb, jointInParent, btTransform::getIdentity());
                        break;
                    }
                    case urdf::Joint::CONTINUOUS:
                    case urdf::Joint::REVOLUTE: {
                        btVector3 axisInJoint = urdfVec3ToBtVec3(parentJoint->axis);
                        btVector3 axisInParent = jointInParent.getBasis().inverse() * axisInJoint;
                        auto* hingeConstraint = simulator.makeBulletObject<btHingeConstraint>(simulator.mConstraints, *parentLinkRb, *linkRb, jointInParent.getOrigin(), btTransform::getIdentity().getOrigin(), axisInParent, axisInJoint, true);
                        if (parentJoint->limits) {
                            hingeConstraint->setLimit(static_cast<btScalar>(parentJoint->limits->lower), static_cast<btScalar>(parentJoint->limits->upper));
                        } else {
                            hingeConstraint->setLimit(-BT_LARGE_FLOAT, BT_LARGE_FLOAT);
                        }
                        simulator.mJointNameToHinges.emplace(parentJoint->name, hingeConstraint);
                        constraint = hingeConstraint;
                        break;
                    }
                    default:
                        throw std::invalid_argument{"Unsupported joint type"};
                }
                simulator.mDynamicsWorld->addConstraint(constraint, true);
            }

            for (urdf::LinkConstSharedPtr childLink: link->child_links) {
                self(self, linkRb, childLink);
            }
        };

        traverse(traverse, nullptr, model.getRoot());
    }

} // namespace mrover
