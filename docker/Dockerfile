#FROM ubuntu:xenial

## setup environment
#RUN locale-gen en_US.UTF-8
#ENV LANG en_US.UTF-8

## setup keys
#RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

## setup sources.list
#RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

## install bootstrap tools
#RUN apt-get update && apt-get install --no-install-recommends -y \
    #python-rosdep \
    #python-rosinstall \
    #python-vcstools \
    #&& rm -rf /var/lib/apt/lists/*

## bootstrap rosdep
#RUN rosdep init && rosdep update

## install ros-core
#ENV ROS_DISTRO kinetic
#RUN apt-get update && apt-get install -y \
    #ros-kinetic-ros-core=1.3.0-0* \
    #&& rm -rf /var/lib/apt/lists/*

## install ros-base
#RUN apt-get update && apt-get install -y \
    #ros-kinetic-ros-base=1.3.0-0* \
    #&& rm -rf /var/lib/apt/lists/*

## install ros-desktop
#RUN apt-get update && apt-get install -y \
    #ros-kinetic-desktop=1.3.0-0* \
    #&& rm -rf /var/lib/apt/lists/*

## install ros-desktop-full
#RUN apt-get update && apt-get install -y \
    #ros-kinetic-desktop-full=1.3.0-0* \
    #&& rm -rf /var/lib/apt/lists/*

# Use ROS base instead of Tensorflow, 
# because downloading ROS packages is super slow. 
FROM ros:kinetic-ros-base

# Init catkin workspace.
RUN . /opt/ros/kinetic/setup.sh && \
    mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    catkin_init_workspace && \
    cd /catkin_ws && \
    catkin_make && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> ~/.bashrc

# Install threexml and shared_serial from local .deb packages.
COPY threexml_pkg /threexml_pkg

# Install threexml with dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        libftdipp1-dev \
        libmuparser-dev \
        libreadline-dev \
        libtinyxml-dev && \
    dpkg -i /threexml_pkg/ros-kinetic-shared-serial_0.2.1-0xenial_amd64.deb && \
    dpkg -i /threexml_pkg/ros-kinetic-threemxl_0.2.0-0xenial_amd64.deb && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /threexml_pkg/

### Tensorflow installation ###
# Pick up some TF dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        curl \
        libfreetype6-dev \
        libpng12-dev \
        libzmq3-dev \
        pkg-config \
        python \
        python-dev \
        rsync \
        software-properties-common \
        unzip \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN curl -O https://bootstrap.pypa.io/get-pip.py && \
    python get-pip.py && \
    rm get-pip.py

RUN pip --no-cache-dir install \
        ipykernel \
        jupyter \
        matplotlib \
        numpy \
        ptpython \
        scipy \
        sklearn \
        && \
    python -m ipykernel.kernelspec

ENV TENSORFLOW_VERSION 0.11.0rc1

# --- DO NOT EDIT OR DELETE BETWEEN THE LINES --- #
# These lines will be edited automatically by parameterized_docker_build.sh. #
# COPY _PIP_FILE_ /
# RUN pip --no-cache-dir install /_PIP_FILE_
# RUN rm -f /_PIP_FILE_

# Install TensorFlow CPU version from central repo
RUN pip --no-cache-dir install \
    http://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-${TENSORFLOW_VERSION}-cp27-none-linux_x86_64.whl
# --- ~ DO NOT EDIT OR DELETE BETWEEN THE LINES --- #

# Copy sample notebooks.
COPY notebooks /notebooks

# Set up our notebook config.
COPY jupyter_notebook_config.py /root/.jupyter/

# Jupyter has issues with being run directly:
#   https://github.com/ipython/ipython/issues/7062
# We just add a little wrapper script.
COPY run_jupyter.sh /

# TensorBoard
EXPOSE 6006
# IPython
EXPOSE 8888
### END Tensorflow installation ###

### OpenCV installation ###
# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake unzip wget \
        python-dev python-tk python-numpy \
        zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev \
        libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev \
        libtbb-dev libeigen3-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
        
# Download OpenCV 3.1.0 source
RUN wget https://github.com/opencv/opencv/archive/3.1.0.tar.gz && \
    tar -xzf 3.1.0.tar.gz && \
    rm 3.1.0.tar.gz

# Build from source and tnstall OpenCV 3.1.0
RUN mkdir -p /opencv-3.1.0/build && \
    cd /opencv-3.1.0/build && \
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DWITH_OPENGL=ON -DWITH_TBB=ON -DWITH_EIGEN=ON \
        -DBUILD_DOCS=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF .. && \
    make -j4 && \
    make install && \
    ldconfig && \
    rm -rf /opencv-3.1.0
### END OpenCV installation ###

WORKDIR "/notebooks"
CMD ["/run_jupyter.sh"]
