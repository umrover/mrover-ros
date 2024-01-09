struct VertexUniforms {
    modelToWorld: mat4x4f,
    worldToCamera: mat4x4f,
    cameraToClip: mat4x4f,
    modelToWorldForNormals: mat4x4f,
}
@group(0) @binding(0) var<uniform> vu: VertexUniforms;

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
    let modelToClip = vu.cameraToClip * vu.worldToCamera * vu.modelToWorld;
    // TODO(quintin): Does WGSL support a constructor?
    var out : OutVertex;
    out.positionInClip = modelToClip * vec4(in.positionInModel, 1);
    out.positionInWorld = vu.modelToWorld * vec4(in.positionInModel, 1);
    out.normalInWorld = vu.modelToWorldForNormals * vec4(in.normalInModel, 0);
    out.uv = in.uv;
    return out;
}

struct FragmentUniforms {
    material: u32,
    hasTexture: u32,
    lightInWorld: vec4f,
    cameraInWorld: vec4f,
    lightColor: vec4f,
    objectColor: vec4f,
}
@group(1) @binding(0) var<uniform> fu: FragmentUniforms;
@group(1) @binding(1) var texture: texture_2d<f32>;
@group(1) @binding(2) var textureSampler: sampler;

@fragment fn fs_main(in: OutVertex) -> @location(0) vec4f {
    let baseColor = select(
        fu.objectColor,
        textureSample(texture, textureSampler, in.uv),
        fu.hasTexture == 1u,
    );

    switch (fu.material) {
        case 0: {
            return baseColor;
        }
        case 1: {
            // Ambient
            let ambientStrength = 0.6;
            let ambient = ambientStrength * fu.lightColor;
            // Diffuse
            let lightDirInWorld = normalize(fu.lightInWorld - in.positionInWorld);
            let diff = max(dot(in.normalInWorld, lightDirInWorld), 0.0);
            let diffuse = diff * fu.lightColor;
            // Specular
            let specularStrength = 0.5;
            let viewDirInWolrd = normalize(fu.cameraInWorld - in.positionInWorld);
            let reflectDir = reflect(-lightDirInWorld, in.normalInWorld);
            let spec = pow(max(dot(viewDirInWolrd, reflectDir), 0.0), 32);
            let specular = specularStrength * spec * fu.lightColor;
            // Combination
            return (ambient + diffuse + specular) * baseColor;
        }
        default: {
            return vec4(1, 0, 1, 1);
        }
    }
}
