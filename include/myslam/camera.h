#pragma once

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "myslam/common_include.h"

namespace myslam {

// Pinhole stereo camera model
class Camera {
public:
    // macro to align the class members to the cache line of CPU, which can improve the performance of accessing these members.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // Define shared pointer type
    typedef std::shared_ptr<Camera> Ptr;

    // Camera intrinsics
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;
    SE3 pose_;             // extrinsic, from stereo camera to single camera
    SE3 pose_inv_;         // inverse of extrinsics

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline,
           const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
        // Compute inverse of extrinsics
        pose_inv_ = pose_.inverse();
    }

    SE3 pose() const { return pose_; }

    // Compute and return the intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }
    
    // Coordinate transformations

    // Convert a 3D point from world coordinates to camera coordinates
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    // Convert a 3D point from camera coordinates to world coordinates
    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    // Convert a 3D point from camera coordinates to pixel coordinates
    Vec2 camera2pixel(const Vec3 &p_c);

    // Convert a 2D point from pixel coordinates to camera coordinates, with
    // an optional depth value
    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    // Convert a 2D point from pixel coordinates to world coordinates, with
    // an optional depth value
    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    // Convert a 3D point from world coordinates to pixel coordinates
    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
};

}  // namespace myslam

#endif  // MYSLAM_CAMERA_H