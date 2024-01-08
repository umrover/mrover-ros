#version 400 core

uniform mat4 modelToWorld, worldToCamera, cameraToClip, modelToWorldForNormals;

layout (location = 0) in vec3 positionInModel;
layout (location = 1) in vec3 normalInModel;
layout (location = 2) in vec2 inUv;

out vec3 positionInWorld, normalInWorld;
out vec2 uv;

void main() {
    positionInWorld = vec3(modelToWorld * vec4(positionInModel, 1.0));
    normalInWorld = normalize(vec3(modelToWorldForNormals * vec4(normalInModel, 0.0)));

    uv = inUv;

    mat4 modelToClip = cameraToClip * worldToCamera * modelToWorld;
    vec4 positionInClip = modelToClip * vec4(positionInModel, 1.0);
    gl_Position = positionInClip;
}