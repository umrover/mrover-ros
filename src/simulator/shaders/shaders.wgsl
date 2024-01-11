// See: https://www.w3.org/TR/WGSL/
// All uniform structs MUST match their C++ counterparts

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

struct ComputeUniforms {
    clipToCamera: mat4x4f,
    resolution: vec2u,
}
// An array of points is eventually copied to CPU memory
// As such it must follow the layout defined in "point.hpp"
struct Point {
    xyz: vec3f,
    rgb: f32, // 3 rgb bytes packed into a 32-bit float
    normalXyz: vec3f,
    curvature: f32,
}
@group(0) @binding(0) var<uniform> cu: ComputeUniforms;
@group(0) @binding(1) var colorImage: texture_2d<f32>;
@group(0) @binding(2) var depthImage: texture_depth_2d;
@group(0) @binding(3) var<storage, read_write> points: array<Point>;

// Convert a BGRA color with float values in [0, 1] to a byte vector in [0, 255]
// Then pack the bytes into a 32-bit float in RGBA order
// This is what ROS expects for point cloud colors
fn pack(color: vec4f) -> f32 {
    let c = vec4u(color * 255);
    return bitcast<f32>(c.b | (c.g << 8) | (c.r << 16));
}

// Workgroup size ensures one invocation per pixel
@compute @workgroup_size(1, 1, 1) fn cs_main(@builtin(global_invocation_id) id: vec3u) {
    let pixelInImage = id.xy; // 2D pixel coordinate in the camera image

    let depth = textureLoad(depthImage, pixelInImage, 0);
    let color = textureLoad(colorImage, pixelInImage, 0);

    // See: https://www.w3.org/TR/webgpu/#coordinate-systems
    // These coordinate are in NDC (normalized device coordinates)
    let pointInClip = vec4f(
        2 * (vec2f(pixelInImage) / vec2f(cu.resolution)) - 1,
        depth,
        1
    );
    let pointInCamera = cu.clipToCamera * pointInClip;

    // "points" is really a 2D array, but WGSL does not support 2D arrays with dynamic sizing
    // This "flattens" (maps) a 2D index to a 1D index
    let flatIndex = pixelInImage.y * cu.resolution.x + pixelInImage.x;
    points[flatIndex].xyz = pointInCamera.xyz / pointInCamera.w;
    points[flatIndex].rgb = pack(color);
    points[flatIndex].normalXyz = vec3(0, 0, 0);
    points[flatIndex].curvature = 0;
}
