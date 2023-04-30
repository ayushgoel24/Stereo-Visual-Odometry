#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {
class Map;

/**
 * @brief Backend class for performing optimization in a separate thread
 * 
 * There is a separate optimization thread, and the optimization is started when the Map is updated
 * Map updates are triggered by the frontend
*/
class Backend {
   
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    // Constructor that starts the optimization thread
    Backend();

    // Set the left and right purpose cameras to obtain internal and external parameters
    void SetCameras( Camera::Ptr left, Camera::Ptr right ) {
        cam_left_ = left;
        cam_right_ = right;
    }

    // Set the map object for the backend to work with
    void SetMap( std::shared_ptr<Map> map ) { map_ = map; }

    // Trigger map update and start optimization
    void UpdateMap();

    // Stop the backend thread
    void Stop();

private:
    
    // Backend thread function
    void BackendLoop();

    // Function to perform optimization for a given set of keyframes and landmarks
    void Optimize( Map::KeyframesType& keyframes, Map::LandmarksType& landmarks );

    std::shared_ptr<Map> map_;      // Shared pointer to the Map object
    std::thread backend_thread_;    // Thread object for the backend thread
    std::mutex data_mutex_;         // Mutex object for thread safety

    /**
     * Condition variable that the backend thread waits on until it is notified of a map update.
     * When the map update is triggered, the backend thread takes the activated keyframes and map points 
     * from the map and performs optimization:
     */
    std::condition_variable map_update_;
    
    // Atomic boolean to indicate whether the backend thread is running or not
    std::atomic<bool> backend_running_;

    // Shared pointers to the left and right cameras
    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace myslam

#endif  // MYSLAM_BACKEND_H