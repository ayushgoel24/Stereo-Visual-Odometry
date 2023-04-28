#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam {

// Inserts a keyframe into the map
void Map::InsertKeyFrame( Frame::Ptr frame ) {
    current_frame_ = frame;
    
    // Check if the keyframe is already in the map
    if ( keyframes_.find( frame->keyframe_id_ ) == keyframes_.end() ) {
        // Add the keyframe to the map
        keyframes_.insert( make_pair( frame->keyframe_id_, frame ) );
        active_keyframes_.insert( make_pair( frame->keyframe_id_, frame ) );
    } else {
        // Update the existing keyframe in the map
        keyframes_[ frame->keyframe_id_ ] = frame;
        active_keyframes_[ frame->keyframe_id_ ] = frame;
    }

    // Check if the number of active keyframes exceeds a certain threshold
    if ( active_keyframes_.size() > num_active_keyframes_ ) {
        // Remove the oldest keyframe if the threshold is exceeded
        RemoveOldKeyframe();
    }
}

// Inserts a landmark into the map
void Map::InsertMapPoint( MapPoint::Ptr map_point ) {
    // Check if the landmark is already in the map
    if ( landmarks_.find( map_point->id_ ) == landmarks_.end() ) {
        // Add the landmark to the map
        landmarks_.insert( make_pair( map_point->id_, map_point ) );
        active_landmarks_.insert( make_pair( map_point->id_, map_point ) );
    } else {
        // Update the existing landmark in the map
        landmarks_[ map_point->id_ ] = map_point;
        active_landmarks_[ map_point->id_ ] = map_point;
    }
}

// Removes the oldest keyframe from the map
void Map::RemoveOldKeyframe() {
    if ( current_frame_ == nullptr ) return;
    
    // Find the two closest and farthest keyframes to the current frame
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->Pose().inverse();
    
    for ( auto& kf : active_keyframes_ ) {
        
        if (kf.second == current_frame_) continue;
        
        auto dis = (kf.second->Pose() * Twc).log().norm();
        
        if ( dis > max_dis ) {
            max_dis = dis;
            max_kf_id = kf.first;
        }

        if ( dis < min_dis ) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    // Define a recent threshold for removing keyframes
    const double min_dis_th = 0.2;
    
    Frame::Ptr frame_to_remove = nullptr;
    if ( min_dis < min_dis_th ) {
        // If there are very close frames, delete the most recent one first
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        // delete the furthest
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
    
    // remove keyframe and landmark observation
    active_keyframes_.erase( frame_to_remove->keyframe_id_ );
    for ( auto feat : frame_to_remove->features_left_ ) {
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }

    for (auto feat : frame_to_remove->features_right_) {
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }

    CleanMap();
}

// Member function of the Map class that removes landmarks that are not being observed by any frames
void Map::CleanMap() {
    int cnt_landmark_removed = 0;   // Counter variable to keep track of how many landmarks are removed

    for ( auto iter = active_landmarks_.begin(); iter != active_landmarks_.end(); ) {
        if ( iter->second->observed_times_ == 0 ) {   // If landmark is not being observed by any frames
            iter = active_landmarks_.erase( iter );   // Remove landmark from the active_landmarks_ map
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
    
    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
}

}  // namespace myslam
