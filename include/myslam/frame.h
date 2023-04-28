#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam {

// Forward declare MapPoint and Feature structs
struct MapPoint;
struct Feature;

/**
 * Frame
 * Each frame is assigned an independent ID, and the key frame is assigned a key frame ID
 */
struct Frame {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;           // ID of this frame
    unsigned long keyframe_id_ = 0;  // ID of key frame
    bool is_keyframe_ = false;       // Whether this frame is a keyframe
    double time_stamp_;              // Timestamp, not used yet
    SE3 pose_;                       // Pose of the camera in this frame, represented as Tcw
    std::mutex pose_mutex_;          // Mutex for locking the pose data
    cv::Mat left_img_, right_img_;   // Left and right stereo images

    // Extracted features in the left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // Corresponding features in the right image, set to nullptr if no corresponding feature is found
    std::vector<std::shared_ptr<Feature>> features_right_;

public:
    // Default constructor
    Frame() {}

    // Constructor
    Frame( long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right );

    // Get the pose of the camera in this frame
    SE3 Pose() {
        std::unique_lock<std::mutex> lck( pose_mutex_ );
        return pose_;
    }

    // Set the pose of the camera in this frame
    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck( pose_mutex_ );
        pose_ = pose;
    }

    // Set this frame as a keyframe and assign a keyframe ID
    void SetKeyFrame();

    // Factory build mode, create a new frame and assign an ID to it
    static std::shared_ptr<Frame> CreateFrame();
};

}  // namespace myslam

#endif  // MYSLAM_FRAME_H
