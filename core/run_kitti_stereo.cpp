#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

// All DEFINE macros take the same three arguments: the name of the flag, its default value, and a 'help' string that describes its use
DEFINE_string( config_file, "./configurations/application.yml", "config file path" );

int main(int argc, char **argv) {

    /** 
     * Commandline flags are flags that users specify on the command line when they run an executable.
     * https://gflags.github.io/gflags/
     */
    google::ParseCommandLineFlags( &argc, &argv, true );

    /**
     * creates a pointer to an instance of the `myslam::VisualOdometry` class, and initializes it with the constructor that takes a `FLAGS_config_file` argument
    */
    myslam::VisualOdometry::Ptr vo( new myslam::VisualOdometry( FLAGS_config_file ) );
    
    /**
     * Asserts if the vo object was initialised correctly.
     * It returns false, in case it is unable to read the configuration file defined by `FLAGS_config_file` argument.
    */
    assert( vo->Init() == true );

    /**
     * Runs the `myslam::VisualOdometry::Run()` method, which is responsible for the main loop of the application.
    */
    vo->Run();

    return 0;
}
