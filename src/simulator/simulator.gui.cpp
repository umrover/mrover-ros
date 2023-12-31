#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::guiUpdate() -> void {
        if (mSaveTask.shouldUpdate() && mEnablePhysics) {
            mBaseLinkHistory.push_back(mLinkNameToRigidBody.at("rover#base_link")->getWorldTransform());
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(mWindow.get());
        ImGui::NewFrame();

        if (mEnablePhysics) {
            mSelection = static_cast<int>(mBaseLinkHistory.size()) - 1;
        } else {
            ImGui::SliderInt("Selection", &mSelection, 0, static_cast<int>(mBaseLinkHistory.size()) - 1);
            mLinkNameToRigidBody.at("rover#base_link")->setWorldTransform(mBaseLinkHistory.at(mSelection));
        }

        // Draw fps text
        {
            ImGui::Begin("Settings", nullptr);
            ImGui::BeginDisabled(!mInGui);

            ImGui::SliderFloat("Target FPS", &mTargetUpdateRate, 5.0f, 1000.0f);
            ImGui::SliderFloat("Fly Speed", &mFlySpeed, 0.01f, 10.0f);
            ImGui::SliderFloat("Look Sense", &mLookSense, 0.0001f, 0.01f);
            ImGui::SliderFloat("FOV", &mFov, 10.0f, 120.0f);
            ImGui::InputFloat3("Gravity", mGravityAcceleration.m_floats);
            ImGui::Checkbox("Enable Physics (P)", &mEnablePhysics);
            ImGui::Checkbox("Render Models (M)", &mRenderModels);
            ImGui::Checkbox("Render Wireframe Colliders (C)", &mRenderWireframeColliders);

            ImGui::EndDisabled();
            ImGui::End();
        }

        {
            ImGui::Begin("Telemtry", nullptr);

            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            ImGui::SliderFloat("Rover Linear Speed", &mRoverLinearSpeed, 0.01f, 10.0f);
            ImGui::SliderFloat("Rover Angular Speed", &mRoverAngularSpeed, 0.01f, 10.0f);
            if (auto it = mLinkNameToRigidBody.find("rover#base_link"); it != mLinkNameToRigidBody.end()) {
                btTransform const& baseLinkInMap = it->second->getWorldTransform();
                btVector3 const& p = baseLinkInMap.getOrigin();
                btQuaternion const& q = baseLinkInMap.getRotation();
                ImGui::Text("Rover Position: (%.2f, %.2f, %.2f)", p.x(), p.y(), p.z());
                ImGui::Text("Rover Orientation: (%.2f, %.2f, %.2f, %.2f)", q.w(), q.x(), q.y(), q.z());
            }

            for (Camera const& camera: mCameras) {
                float aspect = static_cast<float>(camera.resolution.width) / static_cast<float>(camera.resolution.height);
                ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(camera.colorTextureHandle)), {320, 320 / aspect}, {0, 1}, {1, 0});
            }

            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

} // namespace mrover
