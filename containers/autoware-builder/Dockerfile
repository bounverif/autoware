ARG BASE_IMAGE=localhost/autoware
ARG AUTOWARE_VERSION=latest

FROM ${BASE_IMAGE}:${AUTOWARE_VERSION}-base AS autoware-source
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -qq && \
    apt-get install -qy --no-install-recommends \
        git \
        python3-minimal \
        python3-vcstool \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source=etc/autoware/autoware.repos.yml,target=/etc/autoware/autoware.repos.yml \
    mkdir -p ${AUTOWARE_SOURCE_DIR} && vcs import --shallow ${AUTOWARE_SOURCE_DIR} < /etc/autoware/autoware.repos.yml

FROM ${BASE_IMAGE}:${AUTOWARE_VERSION}-base AS autoware-builder
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update -qq && \
    apt-get install -qy --no-install-recommends \
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
    mkdir -p /etc/autoware && vcs export --exact-with-tags ${AUTOWARE_SOURCE_DIR} > /etc/autoware/autoware.repos.yml && \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
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

# Set environment variables for all users 
RUN printf "CMAKE_GENERATOR=%s\\n" ${CMAKE_GENERATOR} >> /etc/environment && \
    printf "CMAKE_C_COMPILER_LAUNCHER=%s\\n" ${CMAKE_C_COMPILER_LAUNCHER} >> /etc/environment && \
    printf "CMAKE_CXX_COMPILER_LAUNCHER=%s\\n" ${CMAKE_CXX_COMPILER_LAUNCHER} >> /etc/environment && \
    printf "CMAKE_CUDA_COMPILER_LAUNCHER=%s\\n" ${CMAKE_CUDA_COMPILER_LAUNCHER} >> /etc/environment

# Ccache configuration
COPY etc/ccache.conf /etc/ccache.conf
RUN export CCACHE_DIR=$(ccache --get-config cache_dir) && \ 
    mkdir -p ${CCACHE_DIR} && \
    chmod 777 ${CCACHE_DIR}

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

FROM ${BASE_IMAGE}:${AUTOWARE_VERSION}-builder AS autoware-builder-with-cache
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT
#
# This build is only for producing a build cache to be saved in the image.
# Hence, we do not want build artifacts and logs.
#
RUN --mount=type=bind,from=autoware-source,source=${AUTOWARE_SOURCE_DIR},target=/tmp/src \
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
    && rm -rf /tmp/install
