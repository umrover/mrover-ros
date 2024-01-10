#include "simulator.hpp"

namespace mrover {

    using namespace std::literals;

    Model::Model(std::string_view uri) {
        // Note(quintin):
        // Ideally we would use glTF as it is open-source (unlike FBX) and has a binary format (unlike OBJ and STL)
        // However I could not get Blender to export the textures by filename
        // The only option was to embed them, but that results in needing to decode the data and duplicating it across models
        if (!uri.ends_with("fbx")) {
            ROS_WARN_STREAM(fmt::format("Model importer has only been tested with the FBX file format: {}", uri));
        }

        // assimp's scene import is slow on the larger rover models, so we load it in a separate thread
        asyncMeshesLoader = boost::async(boost::launch::async, [uri = std::string{uri}] {
            Assimp::Importer importer;
            importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE); // Drop points and lines

            // aiScene const* scene = importer.ReadFile(uri.data(),aiProcessPreset_TargetRealtime_MaxQuality);
            aiScene const* scene = importer.ReadFile(uriToPath(uri), aiProcessPreset_TargetRealtime_Quality);
            if (!scene) {
                throw std::runtime_error{fmt::format("Scene import error: {} on path: {}", importer.GetErrorString(), uri)};
            }
            ROS_INFO_STREAM(fmt::format("Loaded scene: {} with mesh count: {}", uri, scene->mNumMeshes));
            if (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) throw std::runtime_error{fmt::format("Incomplete asset: {}", uri)};

            std::vector<Mesh> meshes;
            meshes.reserve(scene->mNumMeshes);

            for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
                aiMesh const* mesh = scene->mMeshes[meshIndex];

                if (!mesh->HasNormals()) throw std::invalid_argument{fmt::format("Mesh #{} has no normals", meshIndex)};
                if (!mesh->HasTextureCoords(0)) throw std::invalid_argument{fmt::format("Mesh #{} has no texture coordinates", meshIndex)};

                auto& [vertices, normals, uvs, indices, texture, uniforms, bindGroup] = meshes.emplace_back();

                assert(mesh->HasPositions());
                vertices.data.resize(mesh->mNumVertices);
                for (std::size_t vertexIndex = 0; vertexIndex < mesh->mNumVertices; ++vertexIndex) {
                    aiVector3D const& vertex = mesh->mVertices[vertexIndex];
                    vertices.data[vertexIndex] = Eigen::Vector3f{vertex.x, vertex.y, vertex.z};
                }

                indices.data.resize(mesh->mNumFaces * 3);
                for (uint faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex) {
                    aiFace const& face = mesh->mFaces[faceIndex];
                    assert(face.mNumIndices == 3);

                    indices.data[faceIndex * 3 + 0] = face.mIndices[0];
                    indices.data[faceIndex * 3 + 1] = face.mIndices[1];
                    indices.data[faceIndex * 3 + 2] = face.mIndices[2];
                }

                assert(mesh->HasNormals());
                normals.data.resize(mesh->mNumVertices);
                for (std::size_t normalIndex = 0; normalIndex < mesh->mNumVertices; ++normalIndex) {
                    aiVector3D const& normal = mesh->mNormals[normalIndex];
                    normals.data[normalIndex] = Eigen::Vector3f{normal.x, normal.y, normal.z};
                }

                assert(mesh->HasTextureCoords(0));
                uvs.data.resize(mesh->mNumVertices);
                for (std::size_t uvIndex = 0; uvIndex < mesh->mNumVertices; ++uvIndex) {
                    aiVector3D const& uv = mesh->mTextureCoords[0][uvIndex];
                    uvs.data[uvIndex] = Eigen::Vector2f{uv.x, uv.y};
                }

                if (aiMaterial const* material = scene->mMaterials[mesh->mMaterialIndex]) {
                    if (aiString path; material->GetTextureCount(aiTextureType_DIFFUSE) > 0 && material->GetTexture(aiTextureType_DIFFUSE, 0, &path) == AI_SUCCESS) {
                        texture.data = readTexture(path.C_Str());
                    } else {
                        texture.data = cv::Mat{1, 1, CV_8UC4, cv::Scalar{255, 255, 255, 255}};
                    }
                    aiString name;
                    material->Get(AI_MATKEY_NAME, name);
                    ROS_INFO_STREAM(fmt::format("\tLoaded material: {}", name.C_Str()));
                }

                ROS_INFO_STREAM(fmt::format("\tLoaded mesh: #{} with {} vertices and {} faces", meshIndex, mesh->mNumVertices, mesh->mNumFaces));
            }

            return meshes;
        });
    }

    auto Model::waitMeshes() -> void {
        if (!asyncMeshesLoader.valid()) return;

        meshes = asyncMeshesLoader.get();
        auto _ = std::move(asyncMeshesLoader);
    }

    auto Model::areMeshesReady() -> bool {
        if (!asyncMeshesLoader.valid()) return true;

        if (asyncMeshesLoader.wait_for(boost::chrono::seconds{0}) == boost::future_status::ready) {
            waitMeshes();
            return true;
        }

        return false;
    }

    Model::~Model() {
        waitMeshes();

        // for (Mesh& mesh: meshes) {
        //     GLuint vao = std::exchange(mesh.vao, GL_INVALID_HANDLE);
        //     if (vao == GL_INVALID_HANDLE) continue;

        //     glDeleteVertexArrays(1, &vao);
        // }
    }
} // namespace mrover
