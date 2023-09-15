#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"

namespace myslam
{

    /**
     * Constructor to store the configuration file path.
     * @param config_path The path to the configuration file.
     */
    VisualOdometry::VisualOdometry( std::string &config_path )
        : config_file_path_( config_path ) {}

    /**
     * Initializes the Visual Odometry by reading the configuration file,
     * creating the required components, and linking them together.
     * @return True if initialization was successful, false otherwise.
     */
    bool VisualOdometry::Init() {
        // read from config file
        if ( Config::SetParameterFile( config_file_path_ ) == false ) {
            return false;
        }

        // initialize dataset
        dataset_ = Dataset::Ptr( new Dataset( Config::Get<std::string>( "dataset.path" ) ) );
        
        // The `CHECK_EQ()` macro is typically used for testing whether a condition is true, and if it is not, it will terminate the program with an error message.
        CHECK_EQ( dataset_->Init(), true );

        // create components and links
        frontend_ = Frontend::Ptr( new Frontend );
        backend_ = Backend::Ptr( new Backend );
        map_ = Map::Ptr( new Map );
        viewer_ = Viewer::Ptr( new Viewer );

        // link components
        frontend_->SetBackend( backend_ );
        frontend_->SetMap( map_ );
        frontend_->SetViewer( viewer_ );
        frontend_->SetCameras( dataset_->GetCamera(0), dataset_->GetCamera(1) );

        backend_->SetMap( map_ );
        backend_->SetCameras( dataset_->GetCamera(0), dataset_->GetCamera(1) );

        viewer_->SetMap( map_ );

        return true;
    }

    /**
     * Runs the Visual Odometry by processing each frame of the dataset
     * and updating the map.
     */
    void VisualOdometry::Run() {
        while (1) {
            LOG(INFO) << "VO is running";
            if ( Step() == false )  break;
        }

        backend_->Stop();
        viewer_->Close();

        LOG(INFO) << "VO exit";
    }

    /**
     * Processes the next frame of the dataset and updates the map.
     * @return True if the frame was successfully processed, false if there
     * are no more frames to process.
     */
    bool VisualOdometry::Step() {
        Frame::Ptr new_frame = dataset_->NextFrame();
        if ( new_frame == nullptr ) return false;

        // process frame and update map
        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return success;
    }

} // namespace myslam
