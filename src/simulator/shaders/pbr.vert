#version 410 core

layout (location = 0) in vec3 positionInModel;

uniform mat4 modelToWorld;
uniform mat4 worldToCamera;
uniform mat4 cameraToClip;

void main() {
    vec4 positionInClip = cameraToClip * worldToCamera * modelToWorld * vec4(positionInModel, 1.0);
    gl_Position = positionInClip;
}