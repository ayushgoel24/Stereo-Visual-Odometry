/*
The Dataset class has the following member variables:

* dataset_path_: a string that holds the path to the dataset directory.
* current_image_index_: an integer that holds the index of the current image being processed.
* cameras_: a vector of Camera::Ptr shared pointers that holds all the cameras in the dataset.


The Dataset class has the following member functions:

* Dataset(const std::string& dataset_path): the constructor that takes a string argument representing the path to the dataset directory.
* bool Init(): initializes the dataset and returns whether the initialization was successful.
* Frame::Ptr NextFrame(): returns the next frame containing the stereo images.
* Camera::Ptr GetCamera(int camera_id) const: returns the camera with the specified ID.

========================================================================================================================*/

#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam {

/**
 * This class represents a dataset of images and provides methods to
 * access the images and their associated cameras.
 */
class Dataset {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // A shared pointer to the Dataset class
    typedef std::shared_ptr<Dataset> Ptr;

    /**
     * Constructs a Dataset object given the path to the dataset configuration file.
     *
     * @param dataset_path The path to the dataset configuration file.
     */
    Dataset(const std::string& dataset_path);

    /**
     * Initializes the dataset and returns whether the initialization was successful.
     *
     * @return `true` if the initialization was successful, `false` otherwise.
     */
    bool Init();

    /**
     * Returns the next frame containing the stereo images.
     *
     * @return A shared pointer to the Frame object containing the stereo images.
     */
    Frame::Ptr NextFrame();

    /**
     * Returns the camera with the specified ID.
     *
     * @param camera_id The ID of the camera.
     * @return A shared pointer to the Camera object.
     */
    Camera::Ptr GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

private:
    std::string dataset_path_; // The path to the dataset directory
    int current_image_index_ = 0; // The index of the current image being processed
    std::vector<Camera::Ptr> cameras_; // A vector containing all the cameras in the dataset

};

}  // namespace myslam

#endif