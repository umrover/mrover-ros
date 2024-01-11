#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::guiUpdate(wgpu::RenderPassEncoder& pass) -> void {
        if (mSaveTask.shouldUpdate() && mEnablePhysics) {
            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF const& rover = it->second;

                SaveData save;
                save.basePosition = rover.physics->getBasePos();
                save.baseVelocity = rover.physics->getBaseVel();
                for (int link = 0; link < rover.physics->getNumLinks(); ++link) {
                    save.links.emplace_back(rover.physics->getJointPos(link), rover.physics->getJointVel(link));
                }
                mSaveHistory.push_back(std::move(save));
            }
        }

        ImGui_ImplWGPU_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (mEnablePhysics) {
            mSaveSelection = 0;
        } else if (int historySize = static_cast<int>(mSaveHistory.size()) - 1; historySize >= 0) {
            ImGui::Begin("History", nullptr, ImGuiWindowFlags_NoTitleBar);

            ImGui::SliderInt("History", &mSaveSelection, -historySize, 0);

            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF const& rover = it->second;

                SaveData const& saveData = mSaveHistory.at(mSaveSelection + historySize);
                rover.physics->setBasePos(saveData.basePosition);
                rover.physics->setBaseVel(saveData.baseVelocity);
                for (int link = 0; link < rover.physics->getNumLinks(); ++link) {
                    auto const& [position, velocity] = saveData.links.at(link);
                    rover.physics->setJointPos(link, position);
                    rover.physics->setJointVel(link, velocity);
                }

                btAlignedObjectArray<btQuaternion> q;
                btAlignedObjectArray<btVector3> m;
                rover.physics->forwardKinematics(q, m);
                rover.physics->updateCollisionObjectWorldTransforms(q, m);
            }

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
