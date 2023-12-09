#pragma once

#include "../pch.hpp"

#include "sdl_pointer.hpp"

using namespace std::literals;

namespace mrover {
    struct Mesh {


        Mesh(std::string_view uri) {
            constexpr auto PACKAGE_URI_PREFIX = "package://mrover/"sv;

            if (uri.starts_with(PACKAGE_URI_PREFIX)) {
                uri.remove_prefix(PACKAGE_URI_PREFIX.size());
            } else {
                throw std::invalid_argument{std::format("Invalid URI: {}", uri)};
            }

            Assimp::Importer importer;
            importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);

            aiScene const* scene = importer.ReadFile(uri.data(), aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_SortByPType);
            if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
                throw std::runtime_error{std::format("Asset import error: {} on path: {}", importer.GetErrorString(), uri)};
            }
            ROS_INFO_STREAM(std::format("Loaded mesh: {}", uri));

            for (uint i = 0; i < scene->mNumMeshes; ++i) {
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
    };

    struct URDF {
        urdf::Model mModel;
        std::unordered_map<std::string, Mesh> link_meshes;

        explicit URDF(XmlRpc::XmlRpcValue const& init) {
            auto paramName = xmlRpcValueToTypeOrDefault<std::string>(init, "param_name");
            if (!mModel.initParam(paramName)) throw std::runtime_error{std::format("Failed to parse URDF from param: {}", paramName)};

            for (auto const& [link_name, link]: mModel.links_) {
                if (!link->visual || !link->visual->geometry) continue;

                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
                if (!mesh) continue;

                link_meshes.emplace(link_name, mesh->filename);
            }
        }
    };

    using Object = std::variant<URDF>;

    class SimulatorNodelet final : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        std::vector<Object> mObjects;

        SDLPointer<SDL_Window, SDL_CreateWindow, SDL_DestroyWindow> mWindow;
        SDLPointer<std::remove_pointer_t<SDL_GLContext>, SDL_GL_CreateContext, SDL_GL_DeleteContext> mGlContext;

        std::jthread mRunThread;

    public:
        SimulatorNodelet() = default;

        ~SimulatorNodelet() override = default;

        auto initRender() -> void;

        auto parseParams() -> void;

        auto onInit() -> void override;

        auto run() -> void;

        auto renderObject(URDF const& urdf) -> void;

        auto renderUpdate() -> void;
    };
} // namespace mrover
