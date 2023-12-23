#version 410 core

uniform mat4 modelToWorld, worldToCamera, cameraToClip, modelToWorldForNormals;

layout (location = 0) in vec3 positionInModel;
layout (location = 1) in vec3 normalInModel;

out vec3 positionInWorld, normalInWorld;

void main() {
    positionInWorld = vec3(modelToWorld * vec4(positionInModel, 1.0));
    // 0 in the w component means that the vector is a direction, not a position
    normalInWorld = vec3(modelToWorldForNormals * vec4(normalInModel, 0.0));

    mat4 modelToClip = cameraToClip * worldToCamera * modelToWorld;
    vec4 positionInClip = modelToClip * vec4(positionInModel, 1.0);
    gl_Position = positionInClip;
}