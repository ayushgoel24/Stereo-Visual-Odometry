#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam {

// Constructor of MapPoint class, takes an id and a 3D position vector
MapPoint::MapPoint( long id, Vec3 position ) : id_(id), pos_(position) {}

// Factory method to create a new MapPoint
MapPoint::Ptr MapPoint::CreateNewMappoint() {
    // Use a static variable to keep track of factory_id across method calls
    static long factory_id = 0;
    // Create a new MapPoint using the constructor
    MapPoint::Ptr new_mappoint( new MapPoint );
    // Assign a unique id to the new MapPoint and increment the factory_id
    new_mappoint->id_ = factory_id++;
    // Return the new MapPoint
    return new_mappoint;
}

// Method to remove an observation from the MapPoint
void MapPoint::RemoveObservation( std::shared_ptr<Feature> feat ) {
    // Lock the data mutex to prevent concurrent access
    std::unique_lock<std::mutex> lck( data_mutex_ );
    // Iterate through all observations of the MapPoint
    for ( auto iter = observations_.begin(); iter != observations_.end(); iter++ ) {
        // Check if the observation is the one we want to remove
        if ( iter->lock() == feat ) {
            // If so, remove it from the list of observations
            observations_.erase( iter );
            // Reset the map_point_ field of the Feature to null
            feat->map_point_.reset();
            // Decrement the observed_times_ counter
            observed_times_--;
            break;
        }
    }
}

}  // namespace myslam
