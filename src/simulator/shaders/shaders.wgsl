struct SceneUniforms {
    worldToCamera: mat4x4f,
    cameraToClip: mat4x4f,

    lightInWorld: vec4f,
    cameraInWorld: vec4f,
    lightColor: vec4f,
}
@group(1) @binding(0) var<uniform> su: SceneUniforms;

struct MeshUniforms {
    modelToWorld: mat4x4f,
    modelToWorldForNormals: mat4x4f,

    material: u32,
}
@group(0) @binding(0) var<uniform> mu: MeshUniforms;
@group(0) @binding(1) var texture: texture_2d<f32>;
@group(0) @binding(2) var textureSampler: sampler;

struct InVertex {
    @location(0) positionInModel: vec3f,
    @location(1) normalInModel: vec3f,
    @location(2) uv: vec2f,
}
struct OutVertex {
   @builtin(position) positionInClip: vec4f,
   @location(0) positionInWorld: vec4f,
   @location(1) normalInWorld: vec4f,
   @location(2) uv: vec2f,
}
@vertex fn vs_main(in: InVertex) -> OutVertex {
    let positionInWorld = mu.modelToWorld * vec4(in.positionInModel, 1);
    var out : OutVertex;
    out.positionInClip = su.cameraToClip * su.worldToCamera * positionInWorld;
    out.positionInWorld = positionInWorld;
    out.normalInWorld = mu.modelToWorldForNormals * vec4(in.normalInModel, 0);
    out.uv = in.uv;
    return out;
}

@fragment fn fs_main(in: OutVertex) -> @location(0) vec4f {
    let baseColor = textureSample(texture, textureSampler, in.uv);

    switch (mu.material) {
        case 0: {
            return baseColor;
        }
        case 1: {
            // Ambient
            let ambientStrength = 0.6;
            let ambient = ambientStrength * su.lightColor;
            // Diffuse
            let lightDirInWorld = normalize(su.lightInWorld - in.positionInWorld);
            let diff = max(dot(in.normalInWorld, lightDirInWorld), 0.0);
            let diffuse = diff * su.lightColor;
            // Specular
            let specularStrength = 0.5;
            let viewDirInWolrd = normalize(su.cameraInWorld - in.positionInWorld);
            let reflectDir = reflect(-lightDirInWorld, in.normalInWorld);
            let spec = pow(max(dot(viewDirInWolrd, reflectDir), 0.0), 32);
            let specular = specularStrength * spec * su.lightColor;
            // Combination
            return vec4(((ambient + diffuse + specular) * baseColor).rgb, 1);
        }
        default: {
           // Magenta for error (like Source engine :D)
            return vec4(1, 0, 1, 1);
        }
    }
}
