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

        aiScene const* scene = importer.ReadFile(uri.data(),aiProcessPreset_TargetRealtime_MaxQuality);
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            throw std::runtime_error{std::format("Asset import error: {} on path: {}", importer.GetErrorString(), uri)};
        }
        ROS_INFO_STREAM(std::format("Loaded mesh: {}", uri));

        for (std::size_t i = 0; i < scene->mNumMeshes; ++i) {
            aiMesh const* mesh = scene->mMeshes[i];

            glGenVertexArrays(1, &vao);
            glBindVertexArray(vao);

            glGenBuffers(1, &vbo);
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh->mNumVertices * sizeof(decltype(*mesh->mVertices))), mesh->mVertices, GL_STATIC_DRAW);
            vertexCount = static_cast<GLsizei>(mesh->mNumVertices);

            glGenBuffers(1, &ebo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            std::vector<uint> indices(mesh->mNumFaces * 3);
            for (uint j = 0, k = 0; j < mesh->mNumFaces; ++j) {
                aiFace const& face = mesh->mFaces[j];
                assert(face.mNumIndices == 3);

                indices[k++] = face.mIndices[0];
                indices[k++] = face.mIndices[1];
                indices[k++] = face.mIndices[2];
            }
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(decltype(indices)::value_type)), indices.data(), GL_STATIC_DRAW);
            indicesCount = static_cast<GLsizei>(indices.size());

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(aiVector3D), nullptr);
            glEnableVertexAttribArray(0);

            glBindVertexArray(0);

            ROS_INFO_STREAM(std::format("\tLoaded mesh: {} with {} vertices and {} faces", i, mesh->mNumVertices, mesh->mNumFaces));
        }
    }

    Mesh::~Mesh() {
        if (vao != GL_INVALID_HANDLE)
            glDeleteVertexArrays(1, &vao);
        if (vbo != GL_INVALID_HANDLE)
            glDeleteBuffers(1, &vbo);
        if (ebo != GL_INVALID_HANDLE)
            glDeleteBuffers(1, &ebo);
    }
}
