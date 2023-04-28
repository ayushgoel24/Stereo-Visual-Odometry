#pragma once
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {

/**
 * @brief Map class
 * 
 * This class is responsible for maintaining the map, including adding new frames and map points,
 * keeping track of active landmarks and keyframes, removing old keyframes, and cleaning up the map.
 * 
 * The front-end can add new keyframes and map points by calling InsertKeyFrame() and InsertMapPoint(),
 * respectively. The back-end maintains the structure of the map, determines outlier/elimination, etc.
 */
class Map {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() {}

    // Add a new keyframe to the map
    void InsertKeyFrame(Frame::Ptr frame);
    // Add a new map point to the map
    void InsertMapPoint(MapPoint::Ptr map_point);

    // Get all landmarks in the map
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    
    // Get all keyframes in the map
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    // Get active landmarks in the map
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    // Get active keyframes in the map
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    // Remove map points with no observations
    void CleanMap();

private:
    
    // Make old keyframes inactive
    void RemoveOldKeyframe();

    std::mutex data_mutex_;          // Mutex for thread-safe access to data
    LandmarksType landmarks_;        // All landmarks in the map
    LandmarksType active_landmarks_; // Active landmarks in the map
    KeyframesType keyframes_;        // All keyframes in the map
    KeyframesType active_keyframes_; // Active keyframes in the map

    Frame::Ptr current_frame_ = nullptr;

    int num_active_keyframes_ = 7;  // Number of active keyframes
};
}  // namespace myslam

#endif  // MAP_H
