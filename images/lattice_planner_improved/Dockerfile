FROM nvidia/cudagl:10.1-base-ubuntu18.04

# If you don't need GPU support, use this:
# FROM ubuntu:18.04

# ---------- Set up environment variables ----------
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV JVM_LIB=/usr/lib/jvm/java-8-openjdk-amd64/jre/lib/amd64/server/
ENV ROS_DISTRO=melodic
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Stockholm
ENV QT_X11_NO_MITSHM=1

# ---------- Timezone Setup ----------
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ---------- Setup sources ----------
# Replace mirror with a more stable one
RUN sed -i 's/http:\/\/ftp.lysator.liu.se\/ubuntu\//http:\/\/archive.ubuntu.com\/ubuntu\//' /etc/apt/sources.list

# Install packages needed to set up all sources
RUN apt-get clean && apt-get update \
    && apt-get install -q -y --no-install-recommends \
    software-properties-common \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ROS sources
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# Latest GCC and G++
RUN apt-add-repository ppa:ubuntu-toolchain-r/test

# Catkin tools repository
RUN curl http://packages.ros.org/ros.key | apt-key add -

# Source for newer version of Gazebo
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu bionic main" > /etc/apt/sources.list.d/gazebo-latest.list
RUN curl http://packages.osrfoundation.org/gazebo.key | apt-key add -

# Accept the Microsoft core fonts license (msttcorefonts)
RUN echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections

# ---------- Install packages using apt-get ----------
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
    wget \
    dirmngr \
    gnupg2 \
    lsb-release \
    # ROS bootstrap tools
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    # ROS Melodic
    ros-melodic-desktop-full \
    # Basic packages
    screen \
    tree \
    sudo \
    synaptic \
    nano \
    bash-completion \
    git \
    git-lfs \
    software-properties-common \
    build-essential \
    python-pip \
    python3-pip \
    python3-yaml \
    iproute2 \
    # ACADO and other tools
    gcc-8 \
    g++-8 \
    gcc-9 \
    g++-9 \
    cmake \
    gnuplot \
    doxygen \
    graphviz \
    ccache \
    # More ROS packages
    ros-melodic-rqt-graph \
    ros-melodic-rqt-gui \
    ros-melodic-rqt-plot \
    ros-melodic-kobuki-core \
    ros-melodic-kobuki-dock-drive \
    ros-melodic-kobuki-driver \
    ros-melodic-kobuki-ftdi \
    ros-melodic-kobuki-msgs \
    ros-melodic-mav-msgs \
    ros-melodic-mav-planning-msgs \
    ros-melodic-nmea-msgs \
    ros-melodic-roscpp-tutorials \
    ros-melodic-octomap-* \
    python-catkin-tools \
    # Unspecified packages
    tmux \
    vim \
    mc \
    fish \
    zsh \
    mesa-utils \
    x11-apps \
    jstest-gtk \
    htop \
    psmisc \
    mlocate \
    gdb \
    gedit \
    ffmpeg \
    valgrind \
    kcachegrind \
    python-scipy \
    ipython3 \
    python3-numpy \
    python3-scipy \
    python3-sympy \
    python3-matplotlib \
    libblas-dev \
    liblapack-dev \
    liblapacke-dev \
    libeigen3-dev \
    libgsl-dev \
    daemontools \
    libudev-dev \
    libiw-dev \
    freeglut3-dev \
    libdc1394-22 \
    libdc1394-22-dev \
    libjansson-dev \
    libboost-dev \
    imagemagick \
    libtinyxml-dev \
    mercurial \
    cmake \
    libgts-dev \
    libsfml-dev \
    libcanberra-gtk-module \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# ---------- Install CMake (version >= 3.14.5) ----------
ENV CMAKE_VERSION=3.14.5
RUN mkdir -p /opt \
    && curl -SL https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.tar.gz \
    | tar -xzC /opt
ENV PATH=/opt/cmake-${CMAKE_VERSION}-Linux-x86_64/bin:${PATH}
