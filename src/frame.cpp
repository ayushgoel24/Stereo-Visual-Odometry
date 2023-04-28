#include "myslam/frame.h"

namespace myslam {

// Constructor with initialization list to set member variables
Frame::Frame( long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right )
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

// Static method to create a new frame with a unique ID
Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0; // Initialize factory_id only once
    Frame::Ptr new_frame( new Frame ); // Create a new Frame object using smart pointer
    new_frame->id_ = factory_id++; // Set the ID of the new frame
    return new_frame; // Return the new frame
}

// Method to mark the frame as a keyframe and set its keyframe ID
void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0; // Initialize keyframe_factory_id only once
    is_keyframe_ = true; // Mark the frame as a keyframe
    keyframe_id_ = keyframe_factory_id++; // Set the keyframe ID of the frame
}

} // namespace myslam