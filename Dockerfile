# GCC support can be specified at major, minor, or micro version
# (e.g. 8, 8.2 or 8.2.0).
# See https://hub.docker.com/r/library/gcc/ for all supported GCC
# tags from Docker Hub.
# See https://docs.docker.com/samples/library/gcc/ for more on how to use this image
# FROM gcc:latest
FROM ubuntu:20.04

ENV TZ="America/New_York" \
    DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y --no-install-recommends install \
    build-essential \
    git \
    clang \
    gdb \
    wget \
    libgoogle-glog-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libglew-dev \
    libsuitesparse-dev \
    libfmt-dev \
    libboost-all-dev \
    pkg-config \
    unzip

RUN git config --global http.sslVerify false

RUN wget https://github.com/Kitware/CMake/releases/download/v3.27.4/cmake-3.27.4-linux-aarch64.sh --no-check-certificate
RUN chmod +x cmake-3.27.4-linux-aarch64.sh
RUN ./cmake-3.27.4-linux-aarch64.sh --skip-license --prefix=/usr/local
ENV PATH="/usr/local/bin:${PATH}"
RUN rm -rf cmake-3.27.4-linux-aarch64.sh

RUN cmake --version

RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.4.0.zip --no-check-certificate
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.4.0.zip --no-check-certificate
RUN unzip opencv.zip
RUN unzip opencv_contrib.zip

RUN cd opencv-4.4.0 && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.4.0/modules \
    -D BUILD_EXAMPLES=ON .. && \
    make -j4 && \
    make install

RUN rm -rf opencv.zip opencv_contrib.zip opencv-4.4.0/ opencv_contrib-4.4.0/

RUN git clone -b 3.4 --single-branch https://gitlab.com/libeigen/eigen.git && \
    cd eigen && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install
RUN rm -rf eigen/

RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi

RUN cmake --version

WORKDIR /usr/src/
RUN git clone https://github.com/ayushgoel24/Stereo-Visual-Odometry.git 
WORKDIR /usr/src/Stereo-Visual-Odometry

RUN git submodule init && \
    git submodule update

RUN cd 3rdparty/ceres-solver && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install

# install DBow3
RUN cd 3rdparty/DBow3 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install

# install g2o
RUN cd 3rdparty/g2o && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install

# install googletest
RUN cd 3rdparty/googletest && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install

# install sophus
RUN cd 3rdparty/Sophus && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install

# # install pangolin
RUN cd 3rdparty/Pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install

# # This command compiles your app using GCC, adjust for your source code
# RUN g++ -o myapp core/run_kitti_stereo.cpp

# # This command runs your application, comment out this line to compile only
# CMD ["./myapp"]

RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install

RUN ldconfig

CMD ["./bin/run_kitti_stereo"]

LABEL Name=stereovisualodometry Version=0.0.1
