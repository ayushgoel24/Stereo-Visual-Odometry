// Including necessary headers
#include "myslam/dataset.h"
#include "myslam/frame.h"
#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;

namespace myslam {

    // Constructor for the Dataset class
    Dataset::Dataset( const std::string& dataset_path )
        : dataset_path_(dataset_path) {}

    // Function to read camera intrinsics and extrinsics from the calibration file and initialize the camera vector
    bool Dataset::Init() {
        
        // Read camera intrinsics and extrinsics from the calibration file
        ifstream fin( dataset_path_ + "/calib.txt" );
        if ( !fin ) {
            LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
            return false;
        }

        for ( int i = 0; i < 4; ++i ) {
            
            char camera_name[3];
            for ( int k = 0; k < 3; ++k ) {
                fin >> camera_name[k];
            }

            double projection_data[12];
            for (int k = 0; k < 12; ++k) {
                fin >> projection_data[k];
            }

            // Extract camera intrinsics and extrinsics from the projection data
            Mat33 K;
            K << projection_data[0], projection_data[1], projection_data[2],
                projection_data[4], projection_data[5], projection_data[6],
                projection_data[8], projection_data[9], projection_data[10];
            
            Vec3 t;
            t << projection_data[3], projection_data[7], projection_data[11];
            
            /**
             * This transformation is performed to convert the extrinsic parameters from the camera coordinate 
             * system to the world coordinate system.
            */
            t = K.inverse() * t;

            /**
             * The reason for doing this is to make the initial depth estimates more accurate. 
             * Specifically, it helps to ensure that the initial triangulation of feature correspondences between 
             * stereo image pairs results in points that are not too far away from the camera, which can make the 
             * subsequent estimation of the camera poses and scene structure more stable and accurate.
            */
            K = K * 0.5;
            
            /**
             * Create a new camera with the extracted intrinsics and extrinsics and add it to the camera vector
             * `SO3()` creates an identity rotation matrix
             * `SE3(SO3(), t)` creates a rigid transformation with no rotation and the given translation vector t.
            */
            Camera::Ptr new_camera( new Camera( K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(), SE3( SO3(), t ) ) );
            cameras_.push_back(new_camera);
            
            // Print the extrinsics of the camera
            LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
        }

        fin.close();
        // Set the current image index to 0 and return true
        current_image_index_ = 0;
        return true;
    }

    // Function to read the next frame from the dataset
    Frame::Ptr Dataset::NextFrame() {
        
        // Define the file format for the left and right images
        boost::format fmt( "%s/image_%d/%06d.png" );
        
        // Read the left and right images
        cv::Mat image_left, image_right;
        
        image_left = cv::imread( ( fmt % dataset_path_ % 0 % current_image_index_ ).str(), cv::IMREAD_GRAYSCALE );
        image_right = cv::imread( ( fmt % dataset_path_ % 1 % current_image_index_ ).str(), cv::IMREAD_GRAYSCALE );

        // Check if the images were read successfully, if not return nullptr
        if ( image_left.data == nullptr || image_right.data == nullptr ) {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }

        // Resize the left and right images
        cv::Mat image_left_resized, image_right_resized;
        // `cv::INTER_NEAREST` is the interpolation method, which is a nearest-neighbor interpolation method.
        cv::resize( image_left, image_left_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
        cv::resize( image_right, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );

        // Create a new frame with the resized left and right images and increment the current image
        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = image_left_resized;
        new_frame->right_img_ = image_right_resized;
        current_image_index_++;
        return new_frame;

    }

}  // namespace myslam