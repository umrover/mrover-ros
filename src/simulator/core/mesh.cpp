#include "simulator.hpp"

namespace mrover {

    Mesh::Mesh(std::string_view uri) {
        constexpr auto PACKAGE_URI_PREFIX = "package://mrover/"sv;

        if (uri.starts_with(PACKAGE_URI_PREFIX)) {
            uri.remove_prefix(PACKAGE_URI_PREFIX.size());
        } else {
            throw std::invalid_argument{std::format("Invalid URI: {}", uri)};
        }

        Assimp::Importer importer;
        importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);

        // aiScene const* scene = importer.ReadFile(uri.data(),aiProcessPreset_TargetRealtime_MaxQuality);
        aiScene const* scene = importer.ReadFile(uri.data(),aiProcessPreset_TargetRealtime_Quality);
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            throw std::runtime_error{std::format("Asset import error: {} on path: {}", importer.GetErrorString(), uri)};
        }
        ROS_INFO_STREAM(std::format("Loaded mesh: {}", uri));

        bindings.reserve(scene->mNumMeshes);

        for (std::size_t i = 0; i < scene->mNumMeshes; ++i) {
            aiMesh const* mesh = scene->mMeshes[i];

            auto& [vao, vbo, ebo, indicesCount] = bindings.emplace_back();

            glGenVertexArrays(1, &vao);
            assert(vao != GL_INVALID_HANDLE);
            glGenBuffers(1, &vbo);
            assert(vbo != GL_INVALID_HANDLE);
            glGenBuffers(1, &ebo);
            assert(ebo != GL_INVALID_HANDLE);
            glBindVertexArray(vao);

            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh->mNumVertices * sizeof(decltype(*mesh->mVertices))), mesh->mVertices, GL_STATIC_DRAW);

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            std::vector<uint> indices(mesh->mNumFaces * 3);
            for (uint j = 0; j < mesh->mNumFaces; ++j) {
                aiFace const& face = mesh->mFaces[j];
                assert(face.mNumIndices == 3);

                indices[j * 3 + 0] = face.mIndices[0];
                indices[j * 3 + 1] = face.mIndices[1];
                indices[j * 3 + 2] = face.mIndices[2];
            }
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(decltype(indices)::value_type)), indices.data(), GL_STATIC_DRAW);
            indicesCount = static_cast<GLsizei>(indices.size());

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(aiVector3D), nullptr);
            glEnableVertexAttribArray(0);

            glBindBuffer(GL_ARRAY_BUFFER, 0);

            glBindVertexArray(0);

            ROS_INFO_STREAM(std::format("\tLoaded mesh: {} with {} vertices and {} faces", i, mesh->mNumVertices, mesh->mNumFaces));
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
