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

            GLuint vao;
            glGenVertexArrays(1, &vao);
            glBindVertexArray(vao);

            GLuint vbo;
            glGenBuffers(1, &vbo);
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh->mNumVertices * sizeof(aiVector3D)), mesh->mVertices, GL_STATIC_DRAW);

            GLuint ebo;
            glGenBuffers(1, &ebo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);

            std::vector<std::array<uint, 3>> faces(mesh->mNumFaces);
            for (uint j = 0; j < mesh->mNumFaces; ++j) {
                aiFace const& face = mesh->mFaces[j];
                assert(face.mNumIndices == 3);

                faces[j] = {face.mIndices[0], face.mIndices[1], face.mIndices[2]};
            }
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(faces.size() * sizeof(std::array<uint, 3>)), faces.data(), GL_STATIC_DRAW);

            glBindVertexArray(0);

            ROS_INFO_STREAM(std::format("\tLoaded mesh: {} with {} vertices and {} faces", i, mesh->mNumVertices, mesh->mNumFaces));
        }
    }
}
