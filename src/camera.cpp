/*
The Camera class has the following member variables:

* fx_, fy_, cx_, cy_: the intrinsic parameters of the camera
* baseline_: the baseline between the stereo cameras
* pose_: the extrinsic parameters of the stereo camera
* pose_inv_: the inverse of the extrinsic parameters


The Camera class has the following member functions:

* Camera(): default constructor
* world2camera(const Vec3 &p_w, const SE3 &T_c_w): converts a point from world coordinates 
to camera coordinates given the camera's extrinsic parameters
* camera2world(const Vec3 &p_c, const SE3 &T_c_w): converts a point from camera coordinates 
to world coordinates given the camera's extrinsic parameters
* camera2pixel(const Vec3 &p_c): converts a point from camera coordinates to pixel coordinates 
given the camera's intrinsic parameters
* pixel2camera(const Vec2 &p_p, double depth): converts a point from pixel coordinates to 
camera coordinates given the camera's intrinsic parameters and depth
* world2pixel(const Vec3 &p_w, const SE3 &T_c_w): converts a point from world coordinates 
to pixel coordinates given the camera's extrinsic and intrinsic parameters
* pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth): converts a point from pixel 
coordinates to world coordinates given the camera's extrinsic and intrinsic parameters and depth.
*/
#include "myslam/camera.h"

namespace myslam {

// Default constructor
Camera::Camera() {
}

// World to camera coordinate transform
Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w) {
    // The point is first transformed into the coordinate system of the camera and then the camera's pose is applied
    return pose_ * T_c_w * p_w;
}

// Camera to world coordinate transform
Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w) {
    // The point is first transformed into the coordinate system of the stereo camera and then transformed to the world coordinate system
    return T_c_w.inverse() * pose_inv_ * p_c;
}

// Camera to pixel coordinate transform
Vec2 Camera::camera2pixel(const Vec3 &p_c) {
    // Apply camera intrinsics to the camera coordinates to get pixel coordinates
    return Vec2(
            fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
            fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

// Pixel to camera coordinate transform
Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth) {
    // Apply the inverse of camera intrinsics to pixel coordinates to get camera coordinates and set depth
    return Vec3(
            (p_p(0, 0) - cx_) * depth / fx_,
            (p_p(1, 0) - cy_) * depth / fy_,
            depth
    );
}

// World to pixel coordinate transform
Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w) {
    // Apply world to camera and camera to pixel coordinate transforms to get pixel coordinates
    return camera2pixel(world2camera(p_w, T_c_w));
}

// Pixel to world coordinate transform
Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth) {
    // Apply pixel to camera and camera to world coordinate transforms to get world coordinates
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

}  // namespace myslam