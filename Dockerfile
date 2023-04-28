# GCC support can be specified at major, minor, or micro version
# (e.g. 8, 8.2 or 8.2.0).
# See https://hub.docker.com/r/library/gcc/ for all supported GCC
# tags from Docker Hub.
# See https://docs.docker.com/samples/library/gcc/ for more on how to use this image
# FROM gcc:latest
FROM ubuntu:18.04
LABEL Description="Build environment"

ENV HOME /root

SHELL ["/bin/bash", "-c"]

# These commands copy your files into the specified directory in the image
# and set that as the working location
COPY . /usr/src/myapp
WORKDIR /usr/src/myapp

RUN apt-get update && apt-get -y --no-install-recommends install \
    build-essential \
    clang \
    cmake \
    gdb \
    wget

# This command compiles your app using GCC, adjust for your source code
RUN g++ -o myapp core/run_kitti_stereo.cpp

# This command runs your application, comment out this line to compile only
CMD ["./myapp"]

LABEL Name=stereovisualodometry Version=0.0.1
