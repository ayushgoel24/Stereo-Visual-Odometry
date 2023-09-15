# Stereo Visual Odometry using Kitti Dataset

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![GitHub Issues](https://img.shields.io/github/issues/ayushgoel24/Stereo-Visual-Odometry.svg)](https://github.com/ayushgoel24/Stereo-Visual-Odometry/issues)
[![Contributions welcome](https://img.shields.io/badge/Contributions-welcome-orange.svg)](https://github.com/ayushgoel24/Stereo-Visual-Odometry)

This repository contains code for implementing Visual Odometry using stereo images from the Kitti dataset. The aim of this project is to estimate the camera motion and generate a trajectory using stereo images.

## Table of Contents
- [Overview](#overview)
- [Pipeline](#pipeline)
- [Key Concepts Used](#overview)
- [Edge Cases](#overview)
- [Scope of Improvements](#overview)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [Results](#results)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

## Pipeline
![Pipeline](./static/images/pipeline.png)

## Key Concepts Used

1. Lucas-Kanade Optical Flow: Optical flow is used for feature tracking in the stereo images. The Lucas-Kanade algorithm is employed to estimate the motion of features between consecutive frames.

2. GFTT (Good Features to Track): The GFTT algorithm is used for feature extraction. It identifies key points in the image that can be tracked across frames.

3. Multi-Threading and Mutexes: Multi-threading and mutexes are utilized to optimize the performance of the algorithm. This allows for parallel processing and efficient synchronization of data.

## Edge Cases

The following edge cases have been handled in the implementation:

- [x] Handling the case where the number of active keyframes exceeds a predefined limit.
- [ ] Adaptation to changes in brightness settings.
- [ ] Recovery when tracking is lost.

## Scope of Improvements

There are several areas where the code can be improved and extended:

1. Comparison of Feature Extraction Algorithms: In addition to GFTT, it would be beneficial to compare other feature extraction algorithms such as ORB and SIFT. This can help determine the most suitable algorithm for feature detection in different scenarios.

2. Parameterized Optimization Methods: The performance of the odometry algorithm can be further improved by experimenting with different optimization methods and parameters. This would involve comparing the results achieved by varying optimization techniques.

3. Comparison with Other Feature Tracking Algorithms: Apart from Lucas-Kanade, there are various other feature tracking algorithms available. Comparing the performance of different algorithms can provide insights into their strengths and weaknesses.

4. Quantification of Results: To evaluate the accuracy and reliability of the visual odometry system, it would be helpful to quantify the results. This can be achieved by comparing the estimated trajectory with ground truth data and calculating metrics such as error metrics (e.g., RMSE) and trajectory similarity measures (e.g., Dynamic Time Warping).

---

## Running the Code

To build the code, follow these steps:

1. Navigate to the root directory of the project:
   ```
   cd ~/ws
   ```

2. Create a build directory:
   ```
   mkdir build
   ```

3. Enter the build directory:
   ```
   cd build
   ```

4. Generate the build files using CMake:
   ```
   cmake ..
   ```

5. Compile the code:
   ```
   make
   ```

To run the code on the KITTI dataset, execute the following command:

```
cd ~/ws
bin/run_kitti_stereo
```

Ensure that the KITTI dataset is properly configured and accessible in the expected directory structure for the code to run successfully.

## Results

### Output will be like

```
0204 14:58:25.402386 29886 visual_odometry.cpp:41] VO is running
I0204 14:58:25.413861 29886 frontend.cpp:265] Find 130 in the last image.
I0204 14:58:25.415143 29886 frontend.cpp:211] Outlier/Inlier in pose estimating: 16/88
I0204 14:58:25.415163 29886 frontend.cpp:216] Current Pose = 
000.559344 0-0.798614 000.222148 00-25.5681
000.825838 000.560017 0-0.066126 00-302.885
-0.0715975 000.220446 000.972768 00-356.362
0000000000 0000000000 0000000000 0000000001
I0204 14:58:25.420295 29886 visual_odometry.cpp:62] VO cost time: 0.00845767 seconds.
```

### Output GIF of the visual odometry
![Output](./static/gif/result.gif)

## Things to take care of
1. Use 3rd party libraries for version compatibility
2. May need to run commands (or add in .bashrc):
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7
```

## License

This project is licensed under the [MIT License](LICENSE).

## Contact

For any inquiries or questions, please contact [ayush.goel2427@gmail.com].

Happy coding!