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

    auto SimulatorNodelet::initUrdfsFromParams() -> void {
        {
            XmlRpc::XmlRpcValue objects;
            mNh.getParam("objects", objects);
            if (objects.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::invalid_argument{"URDFs to load must be an array. Did you rosparam load a simulator config file properly?"};

            for (int i = 0; i < objects.size(); ++i) { // NOLINT(*-loop-convert)
                XmlRpc::XmlRpcValue const& object = objects[i];

                auto type = xmlRpcValueToTypeOrDefault<std::string>(object, "type");
                auto name = xmlRpcValueToTypeOrDefault<std::string>(object, "name", "<unnamed>");

                NODELET_INFO_STREAM(std::format("Loading object: {} of type: {}", name, type));

                if (type == "urdf") {
                    if (auto [_, was_added] = mUrdfs.try_emplace(name, *this, object); !was_added) {
                        throw std::invalid_argument{std::format("Duplicate object name: {}", name)};
                    }
                }
            }
        }
    }

    auto URDF::makeCollisionShapeForLink(SimulatorNodelet& simulator, urdf::LinkConstSharedPtr const& link) -> btCollisionShape* {
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
                    shapes.emplace_back(simulator.makeBulletObject<btCylinderShapeZ>(simulator.mCollisionShapes, cylinderHalfExtents), urdfPoseToBtTransform(collision->origin));
                    break;
                }
                case urdf::Geometry::MESH: {
                    auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);

                    assert(mesh);

                    std::string const& fileUri = mesh->filename;
                    auto [it, was_inserted] = simulator.mUriToModel.try_emplace(fileUri, fileUri);
                    Model& model = it->second;

                    model.waitMeshes();

                    // Colliders with non-zero mass must be convex
                    // This is a limitation of most physics engines
                    if (link->inertial->mass > std::numeric_limits<float>::epsilon()) {
                        auto* convexHullShape = simulator.makeBulletObject<btConvexHullShape>(simulator.mCollisionShapes);
                        for (Model::Mesh const& meshData: model.meshes) {
                            for (Eigen::Vector3f const& point: meshData.vertices.data) {
                                convexHullShape->addPoint(btVector3{point.x(), point.y(), point.z()}, false);
                            }
                        }
                        convexHullShape->recalcLocalAabb();
                        shapes.emplace_back(convexHullShape, urdfPoseToBtTransform(collision->origin));
                        simulator.mMeshToUri.emplace(convexHullShape, fileUri);
                    } else {
                        auto* triangleMesh = new btTriangleMesh{};
                        triangleMesh->preallocateVertices(static_cast<int>(model.meshes.front().vertices.data.size()));
                        triangleMesh->preallocateIndices(static_cast<int>(model.meshes.front().indices.data.size()));

                        for (Model::Mesh const& meshData: model.meshes) {
                            for (std::size_t i = 0; i < meshData.indices.data.size(); i += 3) {
                                Eigen::Vector3f v0 = meshData.vertices.data[meshData.indices.data[i + 0]];
                                Eigen::Vector3f v1 = meshData.vertices.data[meshData.indices.data[i + 1]];
                                Eigen::Vector3f v2 = meshData.vertices.data[meshData.indices.data[i + 2]];
                                triangleMesh->addTriangle(btVector3{v0.x(), v0.y(), v0.z()}, btVector3{v1.x(), v1.y(), v1.z()}, btVector3{v2.x(), v2.y(), v2.z()});
                            }
                        }

                        auto* meshShape = simulator.makeBulletObject<btBvhTriangleMeshShape>(simulator.mCollisionShapes, triangleMesh, true);
                        meshShape->setMargin(0.01);
                        shapes.emplace_back(meshShape, urdfPoseToBtTransform(collision->origin));
                        simulator.mMeshToUri.emplace(meshShape, fileUri);
                    }
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
            default:
                auto* compoundShape = simulator.makeBulletObject<btCompoundShape>(simulator.mCollisionShapes);
                for (auto const& [shape, transform]: shapes) {
                    compoundShape->addChildShape(transform, shape);
                }
                finalShape = compoundShape;
                break;
        }
        return finalShape;
    }

    auto URDF::makeCameraForLink(SimulatorNodelet& simulator, btMultibodyLink const* link) -> Camera {
        Camera camera;
        camera.link = link;
        camera.resolution = {640, 480};
        camera.updateTask = PeriodicTask{20};
        // TODO(quintin): Why do I have to cast this
        wgpu::TextureUsage usage = static_cast<wgpu::TextureUsage::W>(wgpu::TextureUsage::RenderAttachment | wgpu::TextureUsage::TextureBinding);
        wgpu::TextureUsage colorUsage = static_cast<wgpu::TextureUsage::W>(usage | wgpu::TextureUsage::CopySrc);
        std::tie(camera.colorTexture, camera.colorTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), COLOR_FORMAT, colorUsage, wgpu::TextureAspect::All);
        std::tie(camera.normalTexture, camera.normalTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), NORMAL_FORMAT, usage, wgpu::TextureAspect::All);
        std::tie(camera.depthTexture, camera.depthTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), DEPTH_FORMAT, usage, wgpu::TextureAspect::DepthOnly);
        return camera;
    }

    URDF::URDF(SimulatorNodelet& simulator, XmlRpc::XmlRpcValue const& init) {
        auto urdfUri = xmlRpcValueToTypeOrDefault<std::string>(init, "uri");
        if (!model.initString(performXacro(uriToPath(urdfUri)))) throw std::runtime_error{std::format("Failed to parse URDF from URI: {}", urdfUri)};

        name = xmlRpcValueToTypeOrDefault<std::string>(init, "name");

        auto rootLinkInMap = btTransform::getIdentity();
        if (init.hasMember("translation")) {
            std::array<double, 3> translation = xmlRpcValueToNumberArray<3>(init, "translation");
            rootLinkInMap.setOrigin(btVector3{urdfDistToBtDist(translation[0]), urdfDistToBtDist(translation[1]), urdfDistToBtDist(translation[2])});
        }
        if (init.hasMember("rotation")) {
            std::array<double, 4> rotation = xmlRpcValueToNumberArray<4>(init, "rotation");
            rootLinkInMap.setRotation(btQuaternion{static_cast<btScalar>(rotation[0]), static_cast<btScalar>(rotation[1]), static_cast<btScalar>(rotation[2]), static_cast<btScalar>(rotation[3])});
        }

        std::size_t multiBodyLinkCount = model.links_.size() - 1; // Root link is treated separately by multibody, so subtract it off
        auto* multiBody = physics = simulator.makeBulletObject<btMultiBody>(simulator.mMultiBodies, multiBodyLinkCount, 0, btVector3{0, 0, 0}, false, false);
        multiBody->setBaseWorldTransform(rootLinkInMap);

        std::vector<btMultiBodyLinkCollider*> collidersToFinalize;
        std::vector<btMultiBodyConstraint*> constraintsToFinalize;

        auto traverse = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
            ROS_INFO_STREAM(std::format("Processing link: {}", link->name));

            auto linkIndex = static_cast<int>(linkNameToMeta.size()) - 1;
            auto [it, was_inserted] = linkNameToMeta.emplace(link->name, linkIndex);
            assert(was_inserted);

            // TODO(quintin): Configure this from a plugins XML file?
            if (link->name.contains("camera"sv)) {
                Camera camera = makeCameraForLink(simulator, &multiBody->getLink(linkIndex));
                if (link->name.contains("zed"sv)) {
                    std::string frameId, imageTopic, pointCloudTopic;
                    if (link->name.contains("zed_mini")) {
                        frameId = "zed_mini_left_camera_frame";
                        imageTopic = "mast_camera/left/image";
                        pointCloudTopic = "mast_camera/left/points";
                    } else {
                        frameId = "zed_left_camera_frame";
                        imageTopic = "camera/left/image";
                        pointCloudTopic = "camera/left/points";
                    }
                    camera.frameId = frameId;
                    camera.pub = simulator.mNh.advertise<sensor_msgs::Image>(imageTopic, 1);
                    camera.fov = 60;
                    StereoCamera stereoCamera;
                    stereoCamera.base = std::move(camera);
                    stereoCamera.pcPub = simulator.mNh.advertise<sensor_msgs::PointCloud2>(pointCloudTopic, 1);
                    simulator.mStereoCameras.emplace_back(std::move(stereoCamera));
                } else {
                    camera.frameId = "long_range_camera_link";
                    camera.pub = simulator.mNh.advertise<sensor_msgs::Image>("long_range_camera/image", 1);
                    camera.fov = 15;
                    simulator.mCameras.push_back(std::move(camera));
                }
            } else if (link->name.contains("imu"sv)) {
                Imu& imu = simulator.mImus.emplace_back();
                imu.link = &multiBody->getLink(linkIndex);
                imu.updateTask = PeriodicTask{50};
                imu.pub = simulator.mNh.advertise<sensor_msgs::Imu>("imu/data", 1);
                imu.magPub = simulator.mNh.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
                imu.uncalibPub = simulator.mNh.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
                imu.calibStatusPub = simulator.mNh.advertise<CalibrationStatus>("imu/calibration", 1);
            } else if (link->name.contains("gps"sv)) {
                Gps& gps = simulator.mGps.emplace_back();
                gps.link = &multiBody->getLink(linkIndex);
                gps.updateTask = PeriodicTask{10};
                std::string topic;
                if (link->name.contains("left")) {
                    topic = "right_gps/fix";
                } else if (link->name.contains("right")) {
                    topic = "left_gps/fix";
                } else {
                    topic = "gps/fix";
                }
                gps.pub = simulator.mNh.advertise<sensor_msgs::NavSatFix>(topic, 1);
            }

            for (urdf::VisualSharedPtr const& visual: link->visual_array) {
                switch (visual->geometry->type) {
                    case urdf::Geometry::MESH: {
                        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
                        std::string const& fileUri = mesh->filename;
                        simulator.mUriToModel.try_emplace(fileUri, fileUri);
                        break;
                    }
                    default: {
                        ROS_WARN_STREAM("Currently only mesh visuals are supported");
                    }
                }
            }

            auto* collider = simulator.makeBulletObject<btMultiBodyLinkCollider>(simulator.mMultibodyCollider, multiBody, linkIndex);
            collider->setFriction(1);
            collidersToFinalize.push_back(collider);
            collider->setCollisionShape(makeCollisionShapeForLink(simulator, link));
            simulator.mDynamicsWorld->addCollisionObject(collider, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);

            btScalar mass = 1;
            btVector3 inertia{1, 1, 1}; // TODO(quintin): Is this a sane default?
            if (link->inertial) {
                mass = urdfMassToBtMass(link->inertial->mass);
                inertia = urdfInertiaToBtInertia(link->inertial);
            }

            if (urdf::JointConstSharedPtr parentJoint = link->parent_joint) {
                int parentIndex = linkNameToMeta.at(parentJoint->parent_link_name).index;
                btTransform jointInParent = urdfPoseToBtTransform(parentJoint->parent_to_joint_origin_transform);
                btTransform comInJoint = link->inertial ? urdfPoseToBtTransform(link->inertial->origin) : btTransform::getIdentity();
                btVector3 axisInJoint = urdfPosToBtPos(parentJoint->axis);

                switch (parentJoint->type) {
                    case urdf::Joint::FIXED: {
                        ROS_INFO_STREAM(std::format("Fixed joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        multiBody->setupFixed(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), jointInParent.getOrigin(), comInJoint.getOrigin());
                        break;
                    }
                    case urdf::Joint::CONTINUOUS:
                    case urdf::Joint::REVOLUTE: {
                        ROS_INFO_STREAM(std::format("Rotating joint {}: {} ({}) <-> {} ({})", parentJoint->name, parentJoint->parent_link_name, parentIndex, parentJoint->child_link_name, linkIndex));
                        multiBody->setupRevolute(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), axisInJoint, jointInParent.getOrigin(), comInJoint.getOrigin(), true);

                        if (link->name.contains("wheel"sv)) {
                            ROS_INFO_STREAM("\tWheel");

                            collider->setRollingFriction(0.0);
                            collider->setSpinningFriction(0.0);
                            collider->setFriction(0.8);
                            collider->setContactStiffnessAndDamping(30000, 1000);
                        }
                        break;
                    }
                    case urdf::Joint::PRISMATIC: {
                        ROS_INFO_STREAM(std::format("Prismatic joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        multiBody->setupPrismatic(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), axisInJoint, jointInParent.getOrigin(), comInJoint.getOrigin(), true);
                        break;
                    }
                    default:
                        throw std::invalid_argument{"Unsupported joint type"};
                }
                switch (parentJoint->type) {
                    case urdf::Joint::CONTINUOUS:
                    case urdf::Joint::REVOLUTE:
                    case urdf::Joint::PRISMATIC: {
                        ROS_INFO_STREAM("\tMotor");
                        auto* motor = simulator.makeBulletObject<btMultiBodyJointMotor>(simulator.mMultibodyConstraints, multiBody, linkIndex, 0, 0);
                        constraintsToFinalize.push_back(motor);
                        multiBody->getLink(linkIndex).m_userPtr = motor;
                        break;
                    }
                    default:
                        break;
                }
                switch (parentJoint->type) {
                    case urdf::Joint::REVOLUTE:
                    case urdf::Joint::PRISMATIC: {
                        auto lower = static_cast<btScalar>(parentJoint->limits->lower), upper = static_cast<btScalar>(parentJoint->limits->upper);
                        auto* limitConstraint = simulator.makeBulletObject<btMultiBodyJointLimitConstraint>(simulator.mMultibodyConstraints, multiBody, linkIndex, lower, upper);
                        constraintsToFinalize.push_back(limitConstraint);
                    }
                    default:
                        break;
                }

                multiBody->getLink(linkIndex).m_collider = collider; // Bullet WHY? Why is this not exposed via a function call? This took a LONG time to figure out btw.
            } else {
                multiBody->setBaseMass(mass);
                multiBody->setBaseInertia(inertia);
                multiBody->setBaseCollider(collider);
            }

            it->second.collisionUniforms.resize(link->collision_array.size());
            it->second.visualUniforms.resize(link->visual_array.size());

            for (urdf::LinkConstSharedPtr childLink: link->child_links) {
                self(self, childLink);
            }
        };

        traverse(traverse, model.getRoot());

        multiBody->finalizeMultiDof();
        btAlignedObjectArray<btQuaternion> q;
        btAlignedObjectArray<btVector3> m;
        multiBody->forwardKinematics(q, m);
        multiBody->updateCollisionObjectWorldTransforms(q, m);
        simulator.mDynamicsWorld->addMultiBody(multiBody);

        for (btMultiBodyConstraint* constraint: constraintsToFinalize) {
            constraint->finalizeMultiDof();
            simulator.mDynamicsWorld->addMultiBodyConstraint(constraint);
        }
    }

    auto SimulatorNodelet::getUrdf(std::string const& name) -> std::optional<std::reference_wrapper<URDF>> {
        auto it = mUrdfs.find(name);
        if (it == mUrdfs.end()) return std::nullopt;

        return it->second;
    }

} // namespace mrover