// An array of points is eventually copied to CPU memory
// As such it must follow the layout defined in "point.hpp"
struct Point {
    float x, y, z;
    float rgb; // 3 rgb bytes packed into a 32-bit float
    float nx, ny, nz;
    float curvature;
};

kernel void fillPointCloud(
        float16 clipToCamera,
        int2 resolution,
        read_only image2d_t rgbaImage,
        read_only image2d_t depthImage,
        global struct Point* points) {
    int2 pixel = (int2) (get_global_id(0), get_global_id(1));

    float depth = read_imagef(depthImage, pixel).x;
    float4 rgba = read_imagef(rgbaImage, pixel);
    float rgbaPacked = 0;

    float4 pointInClip = (float4) (2 * ((float) pixel.x / (float) resolution.x) - 1,
                                   2 * ((float) pixel.y / (float) resolution.y) - 1,
                                   2 * depth - 1,
                                   1);

    // Note(quintin):
    // This is just pointInCamera = clipToCamera * pointInClip
    // OpenCL does not seem to have an intrinsic for this
    float4 pointInCamera = (float4) (dot(clipToCamera.s0123, pointInClip),
                                     dot(clipToCamera.s4567, pointInClip),
                                     dot(clipToCamera.s89AB, pointInClip),
                                     dot(clipToCamera.sCDEF, pointInClip));
    pointInCamera /= pointInCamera.w;

    int flatIndex = (resolution.y - pixel.y) * resolution.x + pixel.x;
    points[flatIndex].x = pointInCamera.x;
    points[flatIndex].y = pointInCamera.y;
    points[flatIndex].z = pointInCamera.z;
    points[flatIndex].rgb = rgbaPacked;
    points[flatIndex].nx = 0.0f;
    points[flatIndex].ny = 0.0f;
    points[flatIndex].nz = 0.0f;
    points[flatIndex].curvature = 0.0f;
}