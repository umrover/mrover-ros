struct VertexUniforms {
    modelToWorld: mat4x4<f32>,
    worldToCamera: mat4x4<f32>,
    cameraToClip: mat4x4<f32>,
    modelToWorldForNormals: mat4x4<f32>,
}
@group(0) @binding(0) var<uniform> vu: VertexUniforms;

struct InVertex {
    @location(0) positionInModel: vec3<f32>,
    @location(1) normalInModel: vec3<f32>,
    @location(2) uv: vec2<f32>,
}
struct OutVertex {
   @builtin(position) positionInClip: vec4<f32>,
   @location(0) normalInWorld: vec4<f32>,
   @location(1) uv: vec2<f32>,
}
@vertex fn vs_main(in: InVertex) -> OutVertex {
    let modelToClip = vu.cameraToClip * vu.worldToCamera * vu.modelToWorld;
    // TODO(quintin): Does WGSL support a constructor?
    var out : OutVertex;
    out.positionInClip = modelToClip * vec4(in.positionInModel, 1);
    out.normalInWorld = vu.modelToWorldForNormals * vec4(in.normalInModel, 0);
    out.uv = in.uv;
    return out;
}

struct FragmentUniforms {
    material: u32,
    hasTexture: u32,
    lightInWorld: vec3<f32>,
    cameraInWorld: vec3<f32>,
    lightColor: vec3<f32>,
    objectColor: vec3<f32>,
    _padding: vec2<u32>,
}
@group(1) @binding(0) var<uniform> fu: FragmentUniforms;
@group(1) @binding(1) var texture: texture_2d<f32>;
@group(1) @binding(2) var textureSampler: sampler;

@fragment fn fs_main(in: OutVertex) -> @location(0) vec4f {
    let baseColor = textureSample(texture, textureSampler, in.uv);
    return baseColor;
}
