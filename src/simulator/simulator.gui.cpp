#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::guiUpdate(wgpu::RenderPassEncoder& pass) -> void {
        if (mSaveTask.shouldUpdate() && mEnablePhysics) {
            SaveData save;
            // TODO(quintin): save
            // for (auto const& [link_name, rb]: mLinkNameToRigidBody) {
            //     if (!link_name.starts_with("rover")) continue;
            //
            //     save.links.emplace_back(link_name, rb->getWorldTransform(), rb->getLinearVelocity(), rb->getAngularVelocity());
            // }

            mSaveHistory.push_back(std::move(save));
        }

        ImGui_ImplWGPU_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (mEnablePhysics) {
            mSelection = static_cast<int>(mSaveHistory.size()) - 1;
        } else {
            ImGui::Begin("History", nullptr, ImGuiWindowFlags_NoTitleBar);

            ImGui::SliderInt("History", &mSelection, 0, static_cast<int>(mSaveHistory.size()), "");
            std::size_t inverse = mSaveHistory.size() - static_cast<std::size_t>(mSelection);
            ImGui::SameLine();
            ImGui::Text("-%zu", inverse);

            // TODO(quintin): save
            // for (SaveData const& data = mSaveHistory.at(mSelection);
            //      auto const& [link_name, transform, linearVelocity, angularVelocity]: data.links) {
            //     btRigidBody* rb = mLinkNameToRigidBody.at(link_name);
            //     rb->getMotionState()->setWorldTransform(transform);
            //     rb->setWorldTransform(transform);
            //     rb->setLinearVelocity(linearVelocity);
            //     rb->setAngularVelocity(angularVelocity);
            // }

            ImGui::End();
        }

        // Draw fps text
        {
            ImGui::Begin("Settings", nullptr);
            ImGui::BeginDisabled(!mInGui);

            ImGui::SliderFloat("Target FPS", &mTargetUpdateRate, 5.0f, 1000.0f);
            ImGui::SliderFloat("Fly Speed", &mFlySpeed, 0.01f, 10.0f);
            ImGui::SliderFloat("Look Sense", &mLookSense, 0.0001f, 0.01f);
            ImGui::SliderFloat("FOV", &mFovDegrees, 10.0f, 120.0f);
            ImGui::InputFloat3("Gravity", mGravityAcceleration.m_floats);
            ImGui::SliderFloat("Rover Linear Speed", &mRoverLinearSpeed, 0.01f, 10.0f);
            ImGui::SliderFloat("Rover Angular Speed", &mRoverAngularSpeed, 0.01f, 10.0f);
            ImGui::Checkbox("Enable Physics (P)", &mEnablePhysics);
            ImGui::Checkbox("Render Models (M)", &mRenderModels);
            ImGui::Checkbox("Render Wireframe Colliders (C)", &mRenderWireframeColliders);

            ImGui::SliderFloat("Float", &mFloat, 0.0f, 1000.0f);

            ImGui::EndDisabled();
            ImGui::End();
        }

        {
            ImGui::Begin("Telemtry", nullptr);

            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF const& rover = it->second;

                {
                    SE3 baseLinkInMap = rover.linkInWorld("base_link");
                    R3 p = baseLinkInMap.position();
                    S3 q = baseLinkInMap.rotation().quaternion();
                    ImGui::Text("Rover Position: (%.2f, %.2f, %.2f)", p.x(), p.y(), p.z());
                    ImGui::Text("Rover Orientation: (%.2f, %.2f, %.2f, %.2f)", q.w(), q.x(), q.y(), q.z());
                }
                {
                    R3 p = mCameraInWorld.position();
                    S3 q = mCameraInWorld.rotation().quaternion();
                    ImGui::Text("Camera Position: (%.2f, %.2f, %.2f)", p.x(), p.y(), p.z());
                    ImGui::Text("Camera Orientation: (%.2f, %.2f, %.2f, %.2f)", q.w(), q.x(), q.y(), q.z());
                }

                // for (auto const& [name, i] : rover.linkNameToIndex) {
                //     if (i == -1) continue;
                //
                //     btScalar vel = rover.physics->getJointVel(i);
                //     btScalar pos = rover.physics->getJointPos(i);
                //
                //     ImGui::Text("%s: %.2f, %.2f", name.c_str(), pos, vel);
                // }
            }

            for (Camera const& camera: mCameras) {
                float aspect = static_cast<float>(camera.resolution.x()) / static_cast<float>(camera.resolution.y());
                ImGui::Image(camera.colorTextureView, {320, 320 / aspect}, {0, 0}, {1, 1});
            }

            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass);

        // ImGui_ImplWGPU_RenderDrawData(ImDrawData *draw_data, WGPURenderPassEncoder pass_encoder)
    }

} // namespace mrover
