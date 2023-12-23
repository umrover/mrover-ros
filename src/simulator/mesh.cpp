#include "simulator.hpp"

namespace mrover {

    Model::Model(std::string_view uri) {
        if (!uri.ends_with("glb")) {
            ROS_WARN_STREAM(std::format("Model importer has only been tested with the glTF binary format (.glb): {}", uri));
        }

        Assimp::Importer importer;
        importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);

        // aiScene const* scene = importer.ReadFile(uri.data(),aiProcessPreset_TargetRealtime_MaxQuality);
        aiScene const* scene = importer.ReadFile(uriToPath(uri), aiProcessPreset_TargetRealtime_Quality);
        if (!scene) {
            throw std::runtime_error{std::format("Scene import error: {} on path: {}", importer.GetErrorString(), uri)};
        }
        ROS_INFO_STREAM(std::format("Loaded scene: {} with mesh count: {}", uri, scene->mNumMeshes));
        if (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) throw std::runtime_error{std::format("Incomplete asset: {}", uri)};

        meshes.reserve(scene->mNumMeshes);

        for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
            aiMesh const* mesh = scene->mMeshes[meshIndex];

            if (!mesh->HasNormals()) throw std::invalid_argument{std::format("Mesh #{} has no normals", meshIndex)};
            if (!mesh->HasTextureCoords(0)) throw std::invalid_argument{std::format("Mesh #{} has no texture coordinates", meshIndex)};

            auto& [vao, vbo, nbo, ebo, vertices, normals, uvs, indices] = meshes.emplace_back();

            glGenVertexArrays(1, &vao);
            assert(vao != GL_INVALID_HANDLE);
            glGenBuffers(1, &vbo);
            assert(vbo != GL_INVALID_HANDLE);
            glGenBuffers(1, &nbo);
            assert(nbo != GL_INVALID_HANDLE);
            glGenBuffers(1, &ebo);
            assert(ebo != GL_INVALID_HANDLE);
            glBindVertexArray(vao);

            assert(mesh->HasPositions());
            vertices.resize(mesh->mNumVertices);
            std::for_each(std::execution::par, mesh->mVertices, mesh->mVertices + mesh->mNumVertices, [&](aiVector3D const& vertex) {
                std::size_t vertexIndex = &vertex - mesh->mVertices;
                // This importer has only been tested with Blender exporting to glTF
                // Blender must be using ROS's coordinate system: +x forward, +y left, +z up
                vertices[vertexIndex] = Eigen::Vector3f{vertex.x, -vertex.z, vertex.y};
            });
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(decltype(vertices)::value_type)), vertices.data(), GL_STATIC_DRAW);

            assert(mesh->HasFaces());
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            indices.resize(mesh->mNumFaces * 3);
            for (uint faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex) {
                aiFace const& face = mesh->mFaces[faceIndex];
                assert(face.mNumIndices == 3);

                indices[faceIndex * 3 + 0] = face.mIndices[0];
                indices[faceIndex * 3 + 1] = face.mIndices[1];
                indices[faceIndex * 3 + 2] = face.mIndices[2];
            }
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(decltype(indices)::value_type)), indices.data(), GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(aiVector3D), nullptr);
            glEnableVertexAttribArray(0);

            normals.resize(mesh->mNumVertices);
            std::for_each(std::execution::par, mesh->mNormals, mesh->mNormals + mesh->mNumVertices, [&](aiVector3D const& normal) {
                std::size_t normalIndex = &normal - mesh->mNormals;
                normals[normalIndex] = Eigen::Vector3f{normal.x, -normal.z, normal.y};
            });
            glBindBuffer(GL_ARRAY_BUFFER, nbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(normals.size() * sizeof(decltype(normals)::value_type)), normals.data(), GL_STATIC_DRAW);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(aiVector3D), nullptr);
            glEnableVertexAttribArray(1);

            uvs.resize(mesh->mNumUVComponents[0]);
            std::for_each(std::execution::par, mesh->mTextureCoords[0], mesh->mTextureCoords[0] + mesh->mNumUVComponents[0], [&](aiVector3D const& uv) {
                std::size_t uvIndex = &uv - mesh->mTextureCoords[0];
                uvs[uvIndex] = Eigen::Vector2f{uv.x, uv.y};
            });

            glBindBuffer(GL_ARRAY_BUFFER, 0);

            glBindVertexArray(0);

            ROS_INFO_STREAM(std::format("\tLoaded mesh: #{} with {} vertices and {} faces", meshIndex, mesh->mNumVertices, mesh->mNumFaces));
        }
    }

    Model::~Model() {
        for (auto& [vao, vbo, nbo, ebo, _v, _n, _u, _i]: meshes) {
            glDeleteVertexArrays(1, &vao);
            glDeleteBuffers(1, &vbo);
            glDeleteBuffers(1, &nbo);
            glDeleteBuffers(1, &ebo);
        }
    }
} // namespace mrover
