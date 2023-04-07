#include "myslam/config.h"

namespace myslam
{

    std::shared_ptr<Config> Config::config_ = nullptr; // Static member variable for singleton pattern

    bool Config::SetParameterFile(const std::string &filename)
    {
        /**
         * Checks whether the `config_` pointer is `nullptr` (i.e., it points to nothing) and if so, it creates a new instance of the `Config` class by calling its default constructor with `new Config` and assigns a `std::shared_ptr` to it.
        */
        if (config_ == nullptr)
            config_ = std::shared_ptr<Config>(new Config);

        /**
         * Uses the `cv::FileStorage` class from the OpenCV library to read data from a file and store it in the `file_` member variable of the object pointed to by `config_`.
         * The second argument to the constructor, `cv::FileStorage::READ`, specifies that the file should be opened in read mode.
         * Assuming that the file was successfully opened, the `file_` member variable now contains the data read from the file, which can be accessed and used as needed.
        */
        config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);

        /**
         * Checks whether the `file_` member variable of the object pointed to by `config_` was successfully opened and, if not, logs an error message and releases the resource associated with the `file_` member variable before returning false.
        */
        if (config_->file_.isOpened() == false)
        {
            LOG(ERROR) << "parameter file " << filename << " does not exist.";
            config_->file_.release(); // Release the file resource
            return false;
        }

        return true;
    }

    Config::~Config()
    {
        // Release the file resource if it is currently open
        if (file_.isOpened())
            file_.release();
    }

} // namespace myslam
