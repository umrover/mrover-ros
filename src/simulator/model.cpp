#include "simulator.hpp"

namespace mrover {

    using namespace std::literals;

    Model::Model(std::string_view uri) {
        // Note(quintin):
        // Ideally we would use glTF as it is open-source (unlike FBX) and has a binary format (unlike OBJ and STL)
        // However I could not get Blender to export the textures by filename
        // The only option was to embed them, but that results in needing to decode the data and duplicating it across models
        if (!uri.ends_with("fbx")) {
            ROS_WARN_STREAM(std::format("Model importer has only been tested with the FBX file format: {}", uri));
        }

        Assimp::Importer importer;
        importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE); // Drop points and lines

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

            auto& [vao, vbo, nbo, ebo, ubo, tbo, vertices, normals, uvs, indices] = meshes.emplace_back();

            glGenVertexArrays(1, &vao);
            assert(vao != GL_INVALID_HANDLE);
            glGenBuffers(1, &vbo);
            assert(vbo != GL_INVALID_HANDLE);
            glGenBuffers(1, &nbo);
            assert(nbo != GL_INVALID_HANDLE);
            glGenBuffers(1, &ebo);
            assert(ebo != GL_INVALID_HANDLE);
            glGenBuffers(1, &ubo);
            assert(ubo != GL_INVALID_HANDLE);
            glBindVertexArray(vao);

            vertices.resize(mesh->mNumVertices);
            indices.resize(mesh->mNumFaces * 3);
            normals.resize(mesh->mNumVertices);
            uvs.resize(mesh->mNumVertices);

            assert(mesh->HasFaces());
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            for (uint faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex) {
                aiFace const& face = mesh->mFaces[faceIndex];
                assert(face.mNumIndices == 3);

                indices[faceIndex * 3 + 0] = face.mIndices[0];
                indices[faceIndex * 3 + 1] = face.mIndices[1];
                indices[faceIndex * 3 + 2] = face.mIndices[2];
            }
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(decltype(indices)::value_type)), indices.data(), GL_STATIC_DRAW);

            Eigen::Matrix3f assimpToRos;
            if (uri.ends_with("glb") || uri.ends_with("gltf")) {
                // This importer has only been tested with Blender exporting to glTF
                // Blender must be using ROS's coordinate system: +x forward, +y left, +z up
                assimpToRos << 1, 0, 0,
                        0, 0, -1,
                        0, 1, 0;
            } else {
                assimpToRos = Eigen::Matrix3f::Identity();
            }

            assert(mesh->HasPositions());
            std::for_each(std::execution::par, mesh->mVertices, mesh->mVertices + mesh->mNumVertices, [&](aiVector3D const& vertex) {
                std::size_t vertexIndex = &vertex - mesh->mVertices;
                vertices[vertexIndex] = assimpToRos * Eigen::Vector3f{vertex.x, vertex.y, vertex.z};
            });
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(decltype(vertices)::value_type)), vertices.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(vertices)::value_type), nullptr);
            glEnableVertexAttribArray(0);

            std::for_each(std::execution::par, mesh->mNormals, mesh->mNormals + mesh->mNumVertices, [&](aiVector3D const& normal) {
                std::size_t normalIndex = &normal - mesh->mNormals;
                normals[normalIndex] = assimpToRos * Eigen::Vector3f{normal.x, normal.y, normal.z};
            });
            glBindBuffer(GL_ARRAY_BUFFER, nbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(normals.size() * sizeof(decltype(normals)::value_type)), normals.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(decltype(normals)::value_type), nullptr);
            glEnableVertexAttribArray(1);

            std::for_each(std::execution::par, mesh->mTextureCoords[0], mesh->mTextureCoords[0] + mesh->mNumVertices, [&](aiVector3D const& uv) {
                std::size_t uvIndex = &uv - mesh->mTextureCoords[0];
                uvs[uvIndex] = {uv.x, uv.y};
            });
            glBindBuffer(GL_ARRAY_BUFFER, ubo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(uvs.size() * sizeof(decltype(uvs)::value_type)), uvs.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(decltype(uvs)::value_type), nullptr);
            glEnableVertexAttribArray(2);

            if (aiMaterial const* material = scene->mMaterials[mesh->mMaterialIndex]) {
                if (material->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
                    if (aiString path; material->GetTexture(aiTextureType_DIFFUSE, 0, &path) == AI_SUCCESS) {
                        cv::Mat texture = readTexture(path.C_Str());
                        flip(texture, texture, 0);

                        glGenTextures(1, &tbo);
                        glBindTexture(GL_TEXTURE_2D, tbo);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

                        // Upload texture to GPU memory
                        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture.cols, texture.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, texture.data);
                        glGenerateMipmap(GL_TEXTURE_2D);

                        glBindTexture(GL_TEXTURE_2D, 0); // Unbind
                    }
                }
                aiString name;
                material->Get(AI_MATKEY_NAME, name);
                ROS_INFO_STREAM(std::format("\tLoaded material: {}", name.C_Str()));
            }

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);

            ROS_INFO_STREAM(std::format("\tLoaded mesh: #{} with {} vertices and {} faces", meshIndex, mesh->mNumVertices, mesh->mNumFaces));
        }
    }

    Model::~Model() {
        for (auto& [vao, vbo, nbo, ebo, ubo, tbo, _v, _n, _u, _i]: meshes) {
            glDeleteVertexArrays(1, &vao);
            glDeleteBuffers(1, &vbo);
            glDeleteBuffers(1, &nbo);
            glDeleteBuffers(1, &ebo);
            glDeleteBuffers(1, &ubo);
            glDeleteTextures(1, &tbo);
        }
    }
} // namespace mrover
