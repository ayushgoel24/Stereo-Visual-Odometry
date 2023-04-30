# GCC support can be specified at major, minor, or micro version
# (e.g. 8, 8.2 or 8.2.0).
# See https://hub.docker.com/r/library/gcc/ for all supported GCC
# tags from Docker Hub.
# See https://docs.docker.com/samples/library/gcc/ for more on how to use this image
# FROM gcc:latest
FROM ubuntu:20.04

WORKDIR /usr/src/

RUN apt-get update && apt-get -y --no-install-recommends install \
    build-essential \
    git \
    clang \
    cmake \
    gdb \
    wget \
    libgoogle-glog-dev

RUN cmake --version

RUN git config --global http.sslVerify false

RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi

RUN git clone https://github.com/ayushgoel24/Stereo-Visual-Odometry.git 

WORKDIR /usr/src/Stereo-Visual-Odometry

RUN ls -alh

RUN git submodule init && \
    git submodule update

RUN cd 3rdparty/ceres-solver && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install && \
    # install DBow3
    cd ../../DBow3 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install && \
    # install g2o
    cd ../../g2o && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install && \
    # install googletest
    cd ../../googletest && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install && \
    # install sophus
    cd ../../sophus && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install && \
    # install pangolin
    cd ../../pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(USE_PROC) && \
    make install

# This command compiles your app using GCC, adjust for your source code
RUN g++ -o myapp core/run_kitti_stereo.cpp

# This command runs your application, comment out this line to compile only
CMD ["./myapp"]

LABEL Name=stereovisualodometry Version=0.0.1
