#include "simulator.hpp"

namespace mrover {

    Mesh::Mesh(std::string_view uri) {
        constexpr auto PACKAGE_URI_PREFIX = "package://mrover/"sv;

        if (uri.starts_with(PACKAGE_URI_PREFIX)) {
            uri.remove_prefix(PACKAGE_URI_PREFIX.size());
        } else {
            throw std::invalid_argument{std::format("Invalid URI: {}", uri)};
        }

        if (!uri.ends_with("glb")) {
            ROS_WARN_STREAM(std::format("URDF mesh visual importer has only been tested with the glTF binary format (.glb): {}", uri));
        }

        Assimp::Importer importer;
        importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);

        // aiScene const* scene = importer.ReadFile(uri.data(),aiProcessPreset_TargetRealtime_MaxQuality);
        aiScene const* scene = importer.ReadFile(uri.data(), aiProcessPreset_TargetRealtime_Quality);
        if (!scene) {
            throw std::runtime_error{std::format("Scene import error: {} on path: {}", importer.GetErrorString(), uri)};
        }
        ROS_INFO_STREAM(std::format("Loaded scene: {} with mesh count: {}", uri, scene->mNumMeshes));
        if (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) {
            throw std::runtime_error{std::format("Incomplete asset: {}", uri)};
        }

        bindings.reserve(scene->mNumMeshes);

        for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
            aiMesh const* mesh = scene->mMeshes[meshIndex];

            auto& [vao, vbo, ebo, indicesCount] = bindings.emplace_back();

            glGenVertexArrays(1, &vao);
            assert(vao != GL_INVALID_HANDLE);
            glGenBuffers(1, &vbo);
            assert(vbo != GL_INVALID_HANDLE);
            glGenBuffers(1, &ebo);
            assert(ebo != GL_INVALID_HANDLE);
            glBindVertexArray(vao);

            std::vector<Eigen::Vector3f> vertices(mesh->mNumVertices);
            std::for_each(std::execution::par, mesh->mVertices, mesh->mVertices + mesh->mNumVertices, [&](aiVector3D const& vertex) {
                std::size_t vertexIndex = &vertex - mesh->mVertices;
                // This importer has only been tested with Blender exporting to glTF
                // Blender must be using ROS's coordinate system: +x forward, +y left, +z up
                vertices[vertexIndex] = Eigen::Vector3f{vertex.x, -vertex.z, vertex.y};
            });
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(decltype(vertices)::value_type)), vertices.data(), GL_STATIC_DRAW);

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            std::vector<uint> indices(mesh->mNumFaces * 3);
            for (uint faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex) {
                aiFace const& face = mesh->mFaces[faceIndex];
                assert(face.mNumIndices == 3);

                indices[faceIndex * 3 + 0] = face.mIndices[0];
                indices[faceIndex * 3 + 1] = face.mIndices[1];
                indices[faceIndex * 3 + 2] = face.mIndices[2];
            }
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(decltype(indices)::value_type)), indices.data(), GL_STATIC_DRAW);
            indicesCount = static_cast<GLsizei>(indices.size());

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(aiVector3D), nullptr);
            glEnableVertexAttribArray(0);

            glBindBuffer(GL_ARRAY_BUFFER, 0);

            glBindVertexArray(0);

            ROS_INFO_STREAM(std::format("\tLoaded mesh: {} with {} vertices and {} faces", meshIndex, mesh->mNumVertices, mesh->mNumFaces));
        }
    }

    Mesh::~Mesh() {
        for (auto& [vao, vbo, ebo, _]: bindings) {
            glDeleteVertexArrays(1, &vao);
            glDeleteBuffers(1, &vbo);
            glDeleteBuffers(1, &ebo);
        }
    }
}
