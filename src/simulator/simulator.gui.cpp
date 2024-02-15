#include "simulator.hpp"

namespace mrover {

    auto r3ToBtVector3(R3 const& r3) -> btVector3 {
        return btVector3{static_cast<btScalar>(r3.x()), static_cast<btScalar>(r3.y()), static_cast<btScalar>(r3.z())};
    }

    auto SimulatorNodelet::guiUpdate(wgpu::RenderPassEncoder& pass) -> void {
        if (mSaveTask.shouldUpdate() && mEnablePhysics) {
            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF const& rover = it->second;

                SaveData save;
                save.baseTransform = rover.physics->getBaseWorldTransform();
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

                auto const& [baseTransform, baseVelocity, links] = mSaveHistory.at(mSaveSelection + historySize);
                rover.physics->setBaseWorldTransform(baseTransform);
                rover.physics->setBaseVel(baseVelocity);
                for (int link = 0; link < rover.physics->getNumLinks(); ++link) {
                    auto const& [position, velocity] = links.at(link);
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

            if (ImGui::BeginCombo("Sky Color", std::format("R: {:.3f} G: {:.3f} B: {:.3f} A: {:.3f}", mSkyColor[0], mSkyColor[1], mSkyColor[2], mSkyColor[3]).c_str(), ImGuiComboFlags_HeightLargest)) {
                ImGui::ColorPicker4("Sky Color", mSkyColor.data());
                ImGui::EndCombo();
            }

            // ImGui::SliderFloat("Float", &mFloat, 0.0f, 1000.0f);

            ImGui::SliderFloat3("IK", mIkTarget.data(), -1.f, 1.f);

            ImGui::EndDisabled();
            ImGui::End();
        }

        {
            ImGui::Begin("Telemtry", nullptr);

            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF const& rover = it->second;

                {
                    SE3d baseLinkInMap = rover.linkInWorld("base_link");
                    R3 p = baseLinkInMap.translation();
                    S3 q = baseLinkInMap.quat();
                    ImGui::Text("Rover Position: (%.2f, %.2f, %.2f)", p.x(), p.y(), p.z());
                    ImGui::Text("Rover Orientation: (%.2f, %.2f, %.2f, %.2f)", q.w(), q.x(), q.y(), q.z());
                }
                {
                    R3 p = mCameraInWorld.translation();
                    S3 q = mCameraInWorld.quat();
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

            {
                R3 rayStart = mCameraInWorld.translation();
                R3 rayEnd = rayStart + mCameraInWorld.rotation().matrix().col(0);
                // btMul
                // mDynamicsWorld->rayTest(r3ToBtVector3(rayStart), r3ToBtVector3(rayEnd), [
            }

            for (Camera const& camera: mCameras) {
                float aspect = static_cast<float>(camera.resolution.x()) / static_cast<float>(camera.resolution.y());
                ImGui::Image(camera.colorTextureView, {320, 320 / aspect}, {0, 0}, {1, 1});
            }

            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass);
    }

} // namespace mrover
