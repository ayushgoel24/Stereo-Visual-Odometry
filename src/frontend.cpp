#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam {

Frontend::Frontend() {

    // Initialize GFTT detector with parameters from config file
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    // Initialize member variables with parameters from config file
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    // Set current frame to be the one provided
    current_frame_ = frame;

    // Based on the current status of the frontend, call the appropriate method
    switch (status_) {
        case FrontendStatus::INITING:
            // Perform stereo initialization
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            // Perform frame tracking
            Track();
            break;
        case FrontendStatus::LOST:
            // If lost, reset the frontend
            Reset();
            break;
    }

    // Update last frame to be the current frame
    last_frame_ = current_frame_;
    return true;
}

// Used to detect new features in the current frame's left image using the GFTT feature detector.
int Frontend::DetectFeatures() {
    // create a mask to avoid re-detection of existing features
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    /**
     * This loop iterates over each feature in the `current_frame_->features_left_ vector` and draws 
     * a rectangle around it in the mask image. The rectangle is defined by subtracting and adding 
     * a 10 pixel offset to the feature's position. The `CV_FILLED` flag is used to fill the rectangle 
     * with 0 (black) color. This is done to prevent detecting features that have already been detected 
     * in previous frames. By masking out these features, the detector can focus on detecting new 
     * features in the current frame.
    */
    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    // detect new keypoints using the GFTT feature detector
    std::vector<cv::KeyPoint> keypoints;
    /**
     * The GFTT algorithm is a corner detection algorithm that identifies interesting points in the 
     * image where the intensity changes rapidly in more than one direction. The function takes in 
     * the current frame's left image, the output vector of keypoints, and a mask specifying which 
     * regions of the image to exclude from the feature detection.
    */
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    
    // create features for the detected keypoints
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

// Find corresponding points in the right image using LK flow method.
int Frontend::FindFeaturesInRight() {
    // create a vector of left keypoints and corresponding right keypoints
    std::vector<cv::Point2f> kps_left, kps_right;
    
    for (auto &kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();
        if (mp) {
            // use projected points as initial guess
            auto px =
                camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left iamge
            kps_right.push_back(kp->position_.pt);
        }
    }

    // estimate corresponding points in the right image using LK flow method
    std::vector<uchar> status;
    Mat error;

    /**
     * The `cv::calcOpticalFlowPyrLK` function is used to calculate the optical flow between two images. 
     * It takes as input two consecutive images (current_frame_->left_img_ and current_frame_->right_img_), 
     * along with the corresponding keypoints in these images (kps_left and kps_right), and computes the 
     * optical flow between them.
     * 
     * The status and error variables are used to store the status and error values of the optical flow 
     * calculation for each keypoint. The `cv::Size(11, 11)` parameter specifies the size of the window 
     * used for the optical flow calculation, and 3 is the maximum pyramid level used for the calculation. 
     * The `cv::TermCriteria` parameter specifies the termination criteria for the iterative algorithm 
     * used to compute the optical flow, and `cv::OPTFLOW_USE_INITIAL_FLOW` specifies that an initial flow 
     * approximation should be used as a starting point for the calculation.
    */
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    // create features for the corresponding keypoints in the right image
    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            // creates a new cv::KeyPoint object named kp using the coordinates of the i-th keypoint in 
            // kps_right as its center, and a size of 7 pixels
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}

/**
 * loops through each feature point in the left image of the current frame, and for each valid 
 * feature point (i.e., with a corresponding feature point in the right image), it triangulates 
 * the feature points to get a 3D world point, creates a new map point from the triangulated 3D 
 * point, adds the observations of the feature points to the new map point, associates the map 
 * point with the feature points, and adds the new map point to the map. Finally, it sets the 
 * current frame as a keyframe and adds it to the map, updates the map using the backend, and 
 * logs the number of initialized map points.
*/
bool Frontend::BuildInitMap() {
    // Get the poses of the left and right cameras
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    // Loop through each feature point in the left image of the current frame
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        
        // If there is no corresponding feature point in the right image, skip this point
        if (current_frame_->features_right_[i] == nullptr) continue;
        
        // Triangulate the feature points to get a 3D world point
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};
        Vec3 pworld = Vec3::Zero();

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            // Create a new map point from the triangulated 3D point
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            
            // Add the observations of the feature points to the new map point
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            
            // Associate the map point with the feature points
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            
            // Increment the number of initialized map points
            cnt_init_landmarks++;
            
            // Add the new map point to the map
            map_->InsertMapPoint(new_map_point);
        }
    }
    
    // Set the current frame as a keyframe and add it to the map
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    
    // Update the map using the backend
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

// This function initializes the stereo camera system
bool Frontend::StereoInit() {
    // Detect features in the left image and count the number of features found
    int num_features_left = DetectFeatures();
    // Find features in the right image that correspond to the features found in the left image
    int num_coor_features = FindFeaturesInRight();
    // If the number of corresponding features found is less than the minimum required number, return false
    if (num_coor_features < num_features_init_) {
        return false;
    }

    // If enough corresponding features are found, build the initial map
    bool build_map_success = BuildInitMap();
    // If the map is successfully built, set the status to tracking good and update the viewer
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }

    // If the map is not successfully built, return false
    return false;
}

/**
 * Track features in the last frame using optical flow (LK algorithm) and add new features 
 * to the current frame
 * 
 * @return number of good points tracked in the last frame
*/
int Frontend::TrackLastFrame() {
    // Create vectors to store keypoints in the last and current frames
    std::vector<cv::Point2f> kps_last, kps_current;
    // Iterate through all the features in the last frame
    for (auto &kp : last_frame_->features_left_) {
        if (kp->map_point_.lock()) {
            // If a map point exists for the feature, use it to project
            // the 3D point onto the current frame
            auto mp = kp->map_point_.lock();
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // If no map point exists, simply use the feature's current position
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    // Apply optical flow to estimate the new locations of the keypoints in the current frame
    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    // Iterate through the tracked keypoints and add them to the current frame
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            // If the keypoint was successfully tracked, create a new feature
            // and add it to the current frame
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}

/**
 * This function estimates the current pose of the camera based on the features 
 * detected in the current frame and their corresponding 3D points. It uses g2o 
 * to perform optimization and estimates the pose using a sparse set of 3D points 
 * in the current frame that are observed in both the current and the previous frames.
 * 
 * @return number of inliers
*/
int Frontend::EstimateCurrentPose() {
    // Set up the g2o optimizer
    // Uses a dense linear solver and Levenberg-Marquardt optimization algorithm
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Create a vertex for the camera pose and adds to the optimizer.
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // Get the camera intrinsic matrix
    Mat33 K = camera_left_->K();

    // For each feature in the current frame that has a corresponding map point, 
    // an edge is created between the camera vertex and the 3D point. The edge 
    // is added to the optimizer, and the feature is added to a list of features 
    // for outlier detection. The measurement of the edge is set to the 2D position 
    // of the feature in the current frame, and the information matrix is set to 
    // the identity matrix.
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp) {
            // If the feature has a corresponding 3D point, add it to the optimization
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // Estimate the pose to determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    
    // The optimization is run for a fixed number of iterations, and outliers are 
    // detected based on the error of each edge. If the error is above a threshold, 
    // the corresponding feature is marked as an outlier, and the edge is assigned 
    // a higher level to indicate that it should be considered less important in 
    // subsequent iterations.
    for (int iteration = 0; iteration < 4; ++iteration) {
        // Set the initial estimate for the camera pose
        vertex_pose->setEstimate(current_frame_->Pose());

        // Optimize the graph
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // Count the outliers and mark them as such
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            // Disable robust kernel on the final iteration
            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    
    // Set the estimated pose and mark the outliers
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    // Removes the outliers from the features
    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }

    return features.size() - cnt_outlier;
}

/**
 * Add observations of a new keyframe to the corresponding map points.
*/
void Frontend::SetObservationsForKeyFrame() {
    // For each feature in the left image of the current frame
    for (auto &feat : current_frame_->features_left_) {
        // Retrieve the corresponding map point (if it exists)
        auto mp = feat->map_point_.lock();
        if (mp) {
            // Add the feature as an observation of the map point
            mp->AddObservation(feat);
        }
    }
}

/**
 * Triangulate new map points based on feature matches in left and right images
 * 
 * @returns number of new map points that were triangulated
*/
int Frontend::TriangulateNewPoints() {
    // Get camera poses
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    // Get current frame's pose in world coordinates
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    
    // Initialize counter for number of new map points
    int cnt_triangulated_pts = 0;
    
    // Loop through all left features in current frame
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        // Check if this feature has no associated map point in the left image
        // and has a matching feature in the right image
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) {
            
            // Triangulate the 3D point from the left and right image features
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                // Create a new map point and add it to the map
                auto new_map_point = MapPoint::CreateNewMappoint();
                // Transform the 3D point from current camera frame to world frame
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                
                // Add observations of this map point from both left and right images
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                // Associate the map point with the left and right image features
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                
                // Insert the new map point into the map
                map_->InsertMapPoint(new_map_point);
                
                cnt_triangulated_pts++;
            }
        }
    }

    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

/**
 * This function is responsible for inserting a new keyframe into the map if the 
 * current frame has lost too many feature points to track properly.
 * 
 * @returns true, if a new keyframe was inserted, false, otherwise.
*/
bool Frontend::InsertKeyframe() {

    // Check if we still have enough inliers to continue tracking without inserting a new keyframe.
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // We still have enough inliers, so we don't need to insert a new keyframe.
        return false;
    }
    
    // Otherwise, we need to insert a new keyframe.
    // Set the current frame as a new keyframe and add it to the map.
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    // Update the observations for all map points seen in the new keyframe.
    SetObservationsForKeyFrame();

    // Detect new features in the current frame.
    DetectFeatures();

    // Find corresponding features in the right image of the stereo pair, if available.
    FindFeaturesInRight();

    // Triangulate the map points seen in the new keyframe.
    TriangulateNewPoints();

    // Update the backend with the new keyframe.
    backend_->UpdateMap();

    if (viewer_) viewer_->UpdateMap();

    // Indicate that a new keyframe was successfully inserted.
    return true;
}

/**
 * Track the current frame by matching features with the last frame, estimate the current 
 * camera pose and update the tracking status.
 * 
 * @return true if tracking is successful
*/
bool Frontend::Track() {
    // Set current frame pose relative to the last frame
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    // Track features from the last frame
    int num_track_last = TrackLastFrame();
    
    // Estimate the current camera pose using the tracked features
    tracking_inliers_ = EstimateCurrentPose();

    // Update tracking status based on the number of inliers
    if (tracking_inliers_ > num_features_tracking_) {
        // Tracking is good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // Tracking is bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // Tracking is lost
        status_ = FrontendStatus::LOST;
    }

    // Insert the current frame as a keyframe if necessary
    InsertKeyframe();
    // Update the relative motion between the current and last frames
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    // Visualize the current frame if a viewer is available
    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    
    return true;
}

bool Frontend::Reset() {
    LOG(INFO) << "Reset is not implemented. ";
    return true;
}

}  // namespace myslam