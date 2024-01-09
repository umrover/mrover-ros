@group(0) @binding(0) var<uniform> modelToWorld: mat4x4<f32>;
@group(0) @binding(1) var<uniform> worldToCamera: mat4x4<f32>;
@group(0) @binding(2) var<uniform> cameraToClip: mat4x4<f32>;
@group(0) @binding(3) var<uniform> modelToWorldForNormals: mat4x4<f32>;

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
    let modelToClip = cameraToClip * worldToCamera * modelToWorld;
    // TODO(quintin): Does WGSL support a constructor?
    var out : OutVertex;
    out.positionInClip = modelToClip * vec4(in.positionInModel, 1);
    out.normalInWorld = modelToWorldForNormals * vec4(in.normalInModel, 0);
    out.uv = in.uv;
    return out;
}

//@group(1) @binding(0) var<uniform> material: u32;
//@group(1) @binding(1) var<uniform> hasTexture: u32;
//@group(1) @binding(2) var<uniform> lightInWorld: vec3<f32>;
//@group(1) @binding(3) var<uniform> cameraInWorld: vec3<f32>;
//@group(1) @binding(4) var<uniform> lightColor: vec3<f32>;
//@group(1) @binding(5) var<uniform> objectColor: vec3<f32>;
//@group(1) @binding(6) var textureSampler: sampler;

@fragment fn fs_main(in: OutVertex) -> @location(0) vec4f {
    return vec4f(1.0, 0.2, 0.4, 1.0);
}
