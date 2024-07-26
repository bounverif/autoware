FROM docker.io/library/ubuntu:22.04 AS autoware-base

ARG TARGETARCH
ARG TARGETOS
ARG TARGETPLATFORM
ARG TARGETVARIANT

# Metadata
LABEL org.opencontainers.image.vendor="tr.edu.bogazici.cmpe.bounverif"
LABEL org.opencontainers.image.version="0.1.0"
LABEL org.opencontainers.image.authors="Bogazici University System Verification Group"
LABEL org.opencontainers.image.source="https://github.com/bounverif/autoware"
LABEL org.opencontainers.image.title="Autoware"

# Autoware variables
ARG AUTOWARE_VERSION=latest
ENV AUTOWARE_VERSION=${AUTOWARE_VERSION}
ENV AUTOWARE_SOURCE_DIR=/tmp/src
ENV AUTOWARE_BUILD_DIR=/tmp/build
ENV AUTOWARE_INSTALL_DIR=/opt/autoware

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
        tini \
        wget \
        gnupg2 \
        ca-certificates \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# User management 
ARG USER=bounverif
ARG USERGROUP=${USER}
ARG UID=1000
ARG GID=${UID}

RUN groupadd ${USERGROUP} -g ${GID} && \
    useradd -ms /bin/bash ${USER} -g ${USERGROUP} -u ${UID} && \
    printf "${USER} ALL= NOPASSWD: ALL\\n" >> /etc/sudoers

# ROS repository management
ARG ROS_DISTRO=humble   
ENV ROS_DISTRO=${ROS_DISTRO}
RUN wget -qO- "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" | gpg --dearmour -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

FROM autoware-base AS autoware-source

ARG TARGETARCH
ARG TARGETOS
ARG TARGETPLATFORM
ARG TARGETVARIANT

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
        git \
        python3-minimal \
        python3-vcstool \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

COPY autoware.repos.yml /var/lib/autoware/autoware.repos.${AUTOWARE_VERSION}.yml

RUN mkdir -p ${AUTOWARE_SOURCE_DIR} && vcs import --shallow ${AUTOWARE_SOURCE_DIR} < /var/lib/autoware/autoware.repos.${AUTOWARE_VERSION}.yml

FROM autoware-base AS autoware-builder

ARG TARGETARCH
ARG TARGETOS
ARG TARGETPLATFORM
ARG TARGETVARIANT

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
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
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,from=autoware-source,source=${AUTOWARE_SOURCE_DIR},target=${AUTOWARE_SOURCE_DIR} \
    --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt update && \
    rosdep init && \
    rosdep update -q && \
    rosdep install -qy \
        --from-paths ${AUTOWARE_SOURCE_DIR} \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/* 

ENV CMAKE_GENERATOR=Ninja
ENV CMAKE_C_COMPILER_LAUNCHER=ccache
ENV CMAKE_CXX_COMPILER_LAUNCHER=ccache
ENV CMAKE_CUDA_COMPILER_LAUNCHER=ccache
ENV CCACHE_DIR=/var/cache/ccache

# Set environment variables for all users 
RUN printf "CMAKE_C_COMPILER_LAUNCHER=${CMAKE_C_COMPILER_LAUNCHER}\\n" >> /etc/environment && \
    printf "CMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}\\n" >> /etc/environment && \
    printf "CMAKE_CUDA_COMPILER_LAUNCHER=${CMAKE_CUDA_COMPILER_LAUNCHER}\\n" >> /etc/environment && \
    printf "CCACHE_DIR=${CCACHE_DIR}\\n" >> /etc/environment

RUN mkdir -p ${CCACHE_DIR} && chmod 777 ${CCACHE_DIR}

# CUDA Toolkit Installation
ARG CUDA_ARCH=x86_64
ARG CUDA_DISTRO=ubuntu2204
ARG CUDA_KEYRING_PACKAGE=cuda-keyring_1.1-1_all.deb
ARG CUDA_KEYRING_FILEPATH=https://developer.download.nvidia.com/compute/cuda/repos/${CUDA_DISTRO}/${CUDA_ARCH}/${CUDA_KEYRING_PACKAGE}

ARG CUDA_TOOLKIT_VERSION_MAJOR=12
ARG CUDA_TOOLKIT_VERSION_MINOR=4
ENV CUDA_TOOLKIT_VERSION_DASHED=${CUDA_TOOLKIT_VERSION_MAJOR}-${CUDA_TOOLKIT_VERSION_MINOR}
ENV CUDA_TOOLKIT_VERSION=${CUDA_TOOLKIT_VERSION_MAJOR}.${CUDA_TOOLKIT_VERSION_MINOR}

RUN wget -q ${CUDA_KEYRING_FILEPATH} && dpkg -i ${CUDA_KEYRING_PACKAGE} && rm ${CUDA_KEYRING_PACKAGE}

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

FROM autoware-builder AS autoware-builder-with-cache
#
# This build is only for producing a build cache to be saved in the image.
# Hence, we do not want build artifacts and logs.
#
RUN --mount=type=bind,from=autoware-source,source=${AUTOWARE_SOURCE_DIR},target=/tmp/src \
    export CCACHE_BASEDIR=/tmp && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon --log-base /dev/null build \
        --base-paths /tmp/src \
        --build-base /tmp/build \
        --install-base /tmp/install \
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
            -DCMAKE_BUILD_TYPE=Release \
            " -Wno-dev" \
            " --no-warn-unused-cli" \
    && rm -rf /tmp/build \
    && rm -rf /tmp/install \
    && du -h --max-depth=0 ${CCACHE_DIR}
