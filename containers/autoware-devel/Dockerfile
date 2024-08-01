ARG IMAGE_NAME=localhost/autoware
ARG AUTOWARE_VERSION=latest

FROM docker.io/library/ubuntu:22.04 AS autoware-base
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

# Metadata
LABEL \
org.opencontainers.image.vendor="tr.edu.bogazici.cmpe.bounverif" \
org.opencontainers.image.version=${AUTOWARE_VERSION} \
org.opencontainers.image.authors="Bogazici University System Verification Group" \
org.opencontainers.image.source="https://github.com/bounverif/autoware" \
org.opencontainers.image.title="Autoware" \
org.opencontainers.image.description="Autoware container images" \
org.opencontainers.image.licenses="Apache-2.0" \
org.opencontainers.image.ref.name="autoware" \
org.opencontainers.image.base.name="docker.io/library/ubuntu"

# Debian containers save no deb archives by default. 
# The following command disables clean-up actions to enable
#   --mount=type=cache,target=/var/cache/apt 
# optimizations. Otherwise, apt caching does not work.
RUN rm -f /etc/apt/apt.conf.d/docker-clean

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -qq && \
    apt-get install -qy --no-install-recommends \
        sudo \
        wget \
        gnupg2 \
        ca-certificates \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Build arguments
ARG \
AUTOWARE_VERSION=latest \
CUDA_ARCH=x86_64 \
CUDA_DISTRO=ubuntu2204 \
CUDA_KEYRING_PACKAGE=cuda-keyring_1.1-1_all.deb \ 
CUDA_TOOLKIT_REPOSITORY_URL=https://developer.download.nvidia.com/compute/cuda/repos \
CUDA_TOOLKIT_VERSION_MAJOR=12 \
CUDA_TOOLKIT_VERSION_MINOR=4 \
ROS_DISTRO=humble   

# Environment variables
ENV \
AUTOWARE_BUILD_DIR=/tmp/autoware/build \
AUTOWARE_DATA_DIR=/opt/autoware/data \
AUTOWARE_INSTALL_DIR=/opt/autoware \
AUTOWARE_SOURCE_DIR=/tmp/autoware/src \
AUTOWARE_VERSION=${AUTOWARE_VERSION:-latest} \
CUDA_TOOLKIT_VERSION=${CUDA_TOOLKIT_VERSION_MAJOR}.${CUDA_TOOLKIT_VERSION_MINOR} \
CUDA_TOOLKIT_VERSION_DASHED=${CUDA_TOOLKIT_VERSION_MAJOR}-${CUDA_TOOLKIT_VERSION_MINOR} \
ROS_DISTRO=${ROS_DISTRO:-humble}

# Repository management
RUN set -eux;\
    wget -qO- "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" | gpg --dearmour -o /usr/share/keyrings/ros-archive-keyring.gpg;\
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list;\
    export CUDA_KEYRING_FILEPATH=${CUDA_TOOLKIT_REPOSITORY_URL}/${CUDA_DISTRO}/${CUDA_ARCH}/${CUDA_KEYRING_PACKAGE};\
    wget -q ${CUDA_KEYRING_FILEPATH} && \
    dpkg -i ${CUDA_KEYRING_PACKAGE} && \
    rm ${CUDA_KEYRING_PACKAGE} && \
    unset CUDA_KEYRING_FILEPATH

# User management 
RUN groupadd bounverif -g 1000 && \
    useradd -lms /bin/bash bounverif -g bounverif -u 1000 && \
    printf "%s ALL= NOPASSWD: ALL\\n" bounverif >> /etc/sudoers

FROM ${IMAGE_NAME}:${AUTOWARE_VERSION}-base AS autoware-source
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -qq && \
    apt-get install -qqy --no-install-recommends \
        git \
        python3-minimal \
        python3-vcstool \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source=etc/autoware/autoware.repos.yml,target=/etc/autoware/autoware.repos.yml \
    mkdir -p ${AUTOWARE_SOURCE_DIR} && \
    vcs import --shallow ${AUTOWARE_SOURCE_DIR} < /etc/autoware/autoware.repos.yml

FROM ${IMAGE_NAME}:${AUTOWARE_VERSION}-base AS autoware-builder
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -qq && \
    apt-get install -qqy --no-install-recommends \
        build-essential \
        git \
        cmake \
        ccache \
        ninja-build \
        python3-minimal \
        python3-vcstool \
        python3-rosdep \
        python3-colcon-core \
        python3-colcon-common-extensions \
        ansible-core \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,from=autoware-source,source=${AUTOWARE_SOURCE_DIR},target=${AUTOWARE_SOURCE_DIR} \
    --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    mkdir -p /etc/autoware && \
    vcs export --exact ${AUTOWARE_SOURCE_DIR} > /etc/autoware/autoware.repos.${AUTOWARE_VERSION}.yml && \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    rosdep init && \
    rosdep update -q && \
    rosdep install -qy \
        --from-paths ${AUTOWARE_SOURCE_DIR} \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/* 

# CUDA Toolkit Installation
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -qq && \
    apt-get install -qy --no-install-recommends \
        cuda-minimal-build-${CUDA_TOOLKIT_VERSION_DASHED} \
        libcublas-dev-${CUDA_TOOLKIT_VERSION_DASHED} \
        libcurand-dev-${CUDA_TOOLKIT_VERSION_DASHED} \
        libcusolver-dev-${CUDA_TOOLKIT_VERSION_DASHED} \
        libnvinfer-dev=8.* \
        libnvinfer-headers-dev=8.* \
        libnvinfer-plugin-dev=8.* \
        libnvinfer-headers-plugin-dev=8.* \
        libnvparsers-dev=8.* \
        libnvonnxparsers-dev=8.* \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --set cuda /usr/local/cuda-${CUDA_TOOLKIT_VERSION}

# Environment management
ENV \
CCACHE_DIR=/var/cache/ccache \
CCACHE_BASEDIR=/tmp \
CMAKE_BUILD_TYPE=Release \
CMAKE_C_COMPILER_LAUNCHER=ccache \
CMAKE_CXX_COMPILER_LAUNCHER=ccache \
CMAKE_CUDA_COMPILER_LAUNCHER=ccache \
CMAKE_EXPORT_COMPILE_COMMANDS=ON \
CMAKE_GENERATOR=Ninja

# Ccache configuration
# RUN mkdir -p ${CCACHE_DIR} && chmod 777 ${CCACHE_DIR}

FROM ${IMAGE_NAME}:${AUTOWARE_VERSION}-builder AS autoware-builder-with-cache
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT
#
# This build is only for producing a build cache to be saved in the image.
# Hence, we do not want build artifacts and logs.
#
RUN --mount=type=bind,from=autoware-source,source=${AUTOWARE_SOURCE_DIR},target=/tmp/autoware/src \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon --log-base /dev/null build \
        --base-paths /tmp/autoware/src \
        --build-base /tmp/autoware/build \
        --install-base /tmp/autoware/install \
        --packages-up-to autoware_launch \
        --event-handlers \
            console_direct- \
            console_stderr+ \
            console_cohesion- \
            console_start_end- \
            console_package_list- \
            status- \
            summary+ \
            desktop_notification- \
        --cmake-args \
            " -Wno-dev" \
            " --no-warn-unused-cli" \
    && rm -rf /tmp/autoware

FROM ${IMAGE_NAME}:${AUTOWARE_VERSION}-builder-with-cache AS autoware-devel
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT
