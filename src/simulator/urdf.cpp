#include "simulator.hpp"

namespace mrover {

    constexpr float MASS_MULTIPLIER = 1.0f;
    constexpr float DISTANCE_MULTIPLIER = 1.0f;
    constexpr float INERTIA_MULTIPLIER = MASS_MULTIPLIER * DISTANCE_MULTIPLIER * DISTANCE_MULTIPLIER;

    auto urdfPosToBtPos(urdf::Vector3 const& vec) -> btVector3 {
        return btVector3{static_cast<btScalar>(vec.x), static_cast<btScalar>(vec.y), static_cast<btScalar>(vec.z)} * DISTANCE_MULTIPLIER;
    }

    auto urdfQuatToBtQuat(urdf::Rotation const& quat) -> btQuaternion {
        return btQuaternion{static_cast<btScalar>(quat.x), static_cast<btScalar>(quat.y), static_cast<btScalar>(quat.z), static_cast<btScalar>(quat.w)};
    }

    auto urdfPoseToBtTransform(urdf::Pose const& pose) -> btTransform {
        return btTransform{urdfQuatToBtQuat(pose.rotation), urdfPosToBtPos(pose.position)};
    }

    auto urdfDistToBtDist(double scalar) -> btScalar {
        return static_cast<btScalar>(scalar) * DISTANCE_MULTIPLIER;
    }

    auto urdfMassToBtMass(double scalar) -> btScalar {
        return static_cast<btScalar>(scalar) * MASS_MULTIPLIER;
    }

    auto urdfInertiaToBtInertia(urdf::InertialSharedPtr const& inertia) -> btVector3 {
        return btVector3{static_cast<btScalar>(inertia->ixx), static_cast<btScalar>(inertia->iyy), static_cast<btScalar>(inertia->izz)} * INERTIA_MULTIPLIER;
    }

    URDF::URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init) {
        auto urdfUri = xmlRpcValueToTypeOrDefault<std::string>(init, "uri");
        if (!model.initString(performXacro(uriToPath(urdfUri)))) throw std::runtime_error{std::format("Failed to parse URDF from URI: {}", urdfUri)};

        name = xmlRpcValueToTypeOrDefault<std::string>(init, "name");

        auto rootTransform = btTransform::getIdentity();
        if (init.hasMember("translation")) {
            std::array<double, 3> translation = xmlRpcValueToNumberArray<3>(init, "translation");
            rootTransform.setOrigin(btVector3{urdfDistToBtDist(translation[0]), urdfDistToBtDist(translation[1]), urdfDistToBtDist(translation[2])});
        }

        auto traverse = [&](auto&& self, btRigidBody* parentLinkRb, urdf::LinkConstSharedPtr const& link) -> void {
            ROS_INFO_STREAM(std::format("Processing link: {}", link->name));
            if (link->visual && link->visual->geometry && link->visual->geometry->type == urdf::Geometry::MESH) {
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
                std::string const& fileUri = mesh->filename;
                simulator.mUriToModel.try_emplace(fileUri, fileUri);
            }
            if (std::size_t size = link->visual_array.size(); size > 1) {
                ROS_WARN_STREAM(std::format("Link {} has {} visual elements, only the first one will be used", link->name, size));
            }

            boost::container::static_vector<std::pair<btCollisionShape*, btTransform>, 4> shapes;
            for (urdf::CollisionSharedPtr const& collision: link->collision_array) {
                if (!collision->geometry) throw std::invalid_argument{"Collision has no geometry"};

                switch (collision->geometry->type) {
                    case urdf::Geometry::BOX: {
                        auto box = std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
                        assert(box);

                        btVector3 boxHalfExtents = urdfPosToBtPos(box->dim) / 2;
                        shapes.emplace_back(simulator.makeBulletObject<btBoxShape>(simulator.mCollisionShapes, boxHalfExtents), urdfPoseToBtTransform(collision->origin));
                        break;
                    }
                    case urdf::Geometry::SPHERE: {
                        auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
                        assert(sphere);

                        btScalar radius = urdfDistToBtDist(sphere->radius);
                        shapes.emplace_back(simulator.makeBulletObject<btSphereShape>(simulator.mCollisionShapes, radius), urdfPoseToBtTransform(collision->origin));
                        break;
                    }
                    case urdf::Geometry::CYLINDER: {
                        auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
                        assert(cylinder);

                        btScalar radius = urdfDistToBtDist(cylinder->radius);
                        btScalar halfLength = urdfDistToBtDist(cylinder->length) / 2;
                        btVector3 cylinderHalfExtents{radius, radius, halfLength};
                        shapes.emplace_back(simulator.makeBulletObject<btCylinderShapeZ>(simulator.mCollisionShapes, cylinderHalfExtents), urdfPoseToBtTransform((collision->origin)));
                        break;
                    }
                    case urdf::Geometry::MESH: {
                        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);
                        assert(mesh);

                        std::string const& fileUri = mesh->filename;
                        auto [it, was_inserted] = simulator.mUriToModel.try_emplace(fileUri, fileUri);
                        Model& model = it->second;

                        model.waitMeshes();

                        if (model.meshes.size() != 1) throw std::invalid_argument{"Mesh collider must be constructed from exactly one mesh"};

                        Model::Mesh const& meshData = model.meshes.front();

                        auto* triangleMesh = new btTriangleMesh{};
                        triangleMesh->preallocateVertices(static_cast<int>(meshData.vertices.data.size()));
                        triangleMesh->preallocateIndices(static_cast<int>(meshData.indices.data.size()));
                        for (std::size_t i = 0; i < meshData.indices.data.size(); i += 3) {
                            Eigen::Vector3f v0 = meshData.vertices.data[meshData.indices.data[i + 0]];
                            Eigen::Vector3f v1 = meshData.vertices.data[meshData.indices.data[i + 1]];
                            Eigen::Vector3f v2 = meshData.vertices.data[meshData.indices.data[i + 2]];
                            triangleMesh->addTriangle(btVector3{v0.x(), v0.y(), v0.z()}, btVector3{v1.x(), v1.y(), v1.z()}, btVector3{v2.x(), v2.y(), v2.z()});
                        }
                        auto* meshShape = simulator.makeBulletObject<btBvhTriangleMeshShape>(simulator.mCollisionShapes, triangleMesh, true);
                        shapes.emplace_back(meshShape, urdfPoseToBtTransform(collision->origin));
                        simulator.mMeshToUri.emplace(meshShape, fileUri);
                        break;
                    }
                    default:
                        throw std::invalid_argument{"Unsupported collision type"};
                }
            }
            btCollisionShape* finalShape;
            switch (shapes.size()) {
                case 0:
                    finalShape = simulator.makeBulletObject<btEmptyShape>(simulator.mCollisionShapes);
                    break;
                case 1:
                    finalShape = shapes.front().first;
                    break;
                default:
                    auto* compoundShape = simulator.makeBulletObject<btCompoundShape>(simulator.mCollisionShapes);
                    for (auto const& [shape, transform]: shapes) {
                        compoundShape->addChildShape(transform, shape);
                    }
                    finalShape = compoundShape;
                    break;
            }

            btScalar mass = 1;
            btVector3 inertia{1, 1, 1};
            if (link->inertial) {
                mass = urdfMassToBtMass(link->inertial->mass);
                inertia = urdfInertiaToBtInertia(link->inertial);
            }

            btTransform jointInWorld;
            if (link->parent_joint) {
                assert(parentLinkRb);

                auto parentToWorld = parentLinkRb->getWorldTransform();
                auto jointInParent = urdfPoseToBtTransform(link->parent_joint->parent_to_joint_origin_transform);
                jointInWorld = parentToWorld * jointInParent;
            } else {
                jointInWorld = rootTransform;
            }
            auto* motionState = simulator.makeBulletObject<btDefaultMotionState>(simulator.mMotionStates, jointInWorld);
            auto* linkRb = simulator.makeBulletObject<btRigidBody>(simulator.mCollisionObjects, btRigidBody::btRigidBodyConstructionInfo{mass, motionState, finalShape, inertia});
            simulator.mLinkNameToRigidBody.emplace(SimulatorNodelet::globalName(name, link->name), linkRb);
            linkRb->setActivationState(DISABLE_DEACTIVATION);
            simulator.mDynamicsWorld->addRigidBody(linkRb);

            if (urdf::JointConstSharedPtr parentJoint = link->parent_joint) {
                assert(parentLinkRb);

                btTransform jointInParent = urdfPoseToBtTransform(parentJoint->parent_to_joint_origin_transform);

                switch (parentJoint->type) {
                    case urdf::Joint::FIXED: {
                        ROS_INFO_STREAM(std::format("Fixed joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        auto* constraint = simulator.makeBulletObject<btFixedConstraint>(simulator.mConstraints, *parentLinkRb, *linkRb, jointInParent, btTransform::getIdentity());
                        simulator.mDynamicsWorld->addConstraint(constraint, true);
                        break;
                    }
                    case urdf::Joint::CONTINUOUS:
                    case urdf::Joint::REVOLUTE: {
                        ROS_INFO_STREAM(std::format("Rotating joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        if (link->name.contains("axle"sv)) {
                            ROS_INFO_STREAM("\tSpring");
                            auto* constraint = simulator.makeBulletObject<btGeneric6DofSpring2Constraint>(simulator.mConstraints, *parentLinkRb, *linkRb, jointInParent, btTransform::getIdentity());
                            constraint->setLinearLowerLimit(btVector3{0, -0.02, 0});
                            constraint->setLinearUpperLimit(btVector3{0, 0.02, 0});
                            constraint->setAngularLowerLimit(btVector3{0, 0, 1});
                            constraint->setAngularUpperLimit(btVector3{0, 0, -1});
                            constraint->enableSpring(1, true);
                            constraint->setStiffness(1, 40);
                            constraint->setDamping(1, 2);
                            constraint->setParam(BT_CONSTRAINT_CFM, 0.15f, 1);
                            constraint->setParam(BT_CONSTRAINT_ERP, 0.35f, 1);
                            constraint->setEquilibriumPoint();
                            constraint->setEquilibriumPoint(1, 0.1);
                            simulator.mJointNameToSpringHinges.emplace(SimulatorNodelet::globalName(name, parentJoint->name), constraint);
                            simulator.mDynamicsWorld->addConstraint(constraint, true);
                        } else {
                            btVector3 axisInJoint = urdfPosToBtPos(parentJoint->axis);
                            btVector3 axisInParent = jointInParent.getBasis() * axisInJoint;
                            auto* hingeConstraint = simulator.makeBulletObject<btHingeConstraint>(simulator.mConstraints, *parentLinkRb, *linkRb, jointInParent.getOrigin(), btTransform::getIdentity().getOrigin(), axisInParent, axisInJoint, true);
                            if (parentJoint->type == urdf::Joint::REVOLUTE) {
                                hingeConstraint->setLimit(static_cast<btScalar>(parentJoint->limits->lower), static_cast<btScalar>(parentJoint->limits->upper));
                            }
                            simulator.mJointNameToHinges.emplace(SimulatorNodelet::globalName(name, parentJoint->name), hingeConstraint);
                            simulator.mDynamicsWorld->addConstraint(hingeConstraint, true);

                            if (link->name.contains("wheel"sv)) {
                                ROS_INFO_STREAM("\tGear");
                                auto* gearConstraint = simulator.makeBulletObject<btGearConstraint>(simulator.mConstraints, *parentLinkRb, *linkRb, axisInParent, axisInJoint, 50.0);
                                simulator.mDynamicsWorld->addConstraint(gearConstraint, true);
                                linkRb->setFriction(1100);
                            }
                        }
                        break;
                    }
                    default:
                        throw std::invalid_argument{"Unsupported joint type"};
                }
            }

            for (urdf::LinkConstSharedPtr childLink: link->child_links) {
                self(self, linkRb, childLink);
            }
        };

        traverse(traverse, nullptr, model.getRoot());
    }

} // namespace mrover
