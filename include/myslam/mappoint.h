#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {

// Forward declarations for Frame and Feature classes
struct Frame;
struct Feature;

/**
 * MapPoint class represents a landmark point that is observed by one or more cameras in a scene.
 * This class stores information about the 3D position of the point, its unique ID, and the list of
 * features that have observed it.
 */
struct MapPoint {
   
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long id_ = 0;          // Unique ID for the MapPoint
    bool is_outlier_ = false;       // Indicates if the MapPoint is an outlier or not
    Vec3 pos_ = Vec3::Zero();       // Position of the MapPoint in the world
    std::mutex data_mutex_;         // Mutex for thread-safe access to MapPoint data
    int observed_times_ = 0;        // Number of times the MapPoint has been observed
    std::list<std::weak_ptr<Feature>> observations_;  // List of features that observe the MapPoint

    // Default constructor for MapPoint
    MapPoint() {}

    // Constructor for MapPoint that initializes its unique ID and 3D position
    MapPoint( long id, Vec3 position );

    // Returns the position of the MapPoint in the world
    Vec3 Pos() {
        std::unique_lock<std::mutex> lck( data_mutex_ );
        return pos_;
    }

    // Sets the position of the MapPoint in the world
    void SetPos( const Vec3 &pos ) {
        std::unique_lock<std::mutex> lck( data_mutex_ );
        pos_ = pos;
    };

    // Adds a feature observation of the MapPoint
    void AddObservation( std::shared_ptr<Feature> feature ) {
        std::unique_lock<std::mutex> lck( data_mutex_ );
        observations_.push_back( feature );
        observed_times_++;
    }

    // Removes a feature observation of the MapPoint
    void RemoveObservation( std::shared_ptr<Feature> feat );

    // Returns a list of features that observe the MapPoint
    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    // Factory function for creating a new MapPoint
    static MapPoint::Ptr CreateNewMappoint();
};
}  // namespace myslam

#endif  // MYSLAM_MAPPOINT_H
