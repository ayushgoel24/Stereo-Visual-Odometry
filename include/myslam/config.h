#pragma once

#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

/**
 * Configuration class, used to read parameter values from a configuration file
 * using the singleton pattern.
 */
class Config {

private:
    // Static member variable for singleton pattern
    static std::shared_ptr<Config> config_;
    // cv::FileStorage object to read parameter values
    cv::FileStorage file_;

    // Private constructor for singleton pattern
    Config() {} 

public:
    // Destructor to release the cv::FileStorage resource
    ~Config(); 

    // Set the configuration file to be used for reading parameter values
    static bool SetParameterFile( const std::string& filename );

    // Get the value of a parameter specified by its key
    template<typename T>
    static T Get( const std::string& key ) {
        // Return the value of the specified parameter
        return T( Config::config_->file_[ key ] );
    }
};

} // namespace myslam

#endif // MYSLAM_CONFIG_H
