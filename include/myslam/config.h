#pragma once

#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h" // Include necessary headers

namespace myslam {

/**
 * Configuration class, used to read parameter values from a configuration file
 * using the singleton pattern.
 */
class Config {
private:
    static std::shared_ptr<Config> config_; // Static member variable for singleton pattern
    cv::FileStorage file_; // cv::FileStorage object to read parameter values

    Config() {} // Private constructor for singleton pattern

public:
    ~Config(); // Destructor to release the cv::FileStorage resource

    // Set the configuration file to be used for reading parameter values
    static bool SetParameterFile(const std::string& filename);

    // Get the value of a parameter specified by its key
    template<typename T>
    static T Get(const std::string& key) {
        return T(Config::config_->file_[key]); // Return the value of the specified parameter
    }
};

} // namespace myslam

#endif // MYSLAM_CONFIG_H
