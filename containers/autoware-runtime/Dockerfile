ARG IMAGE_NAME=localhost/autoware
ARG AUTOWARE_VERSION=latest

FROM ${IMAGE_NAME}:latest-builder-with-cache AS autoware-prebuilt
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

ARG AUTOWARE_SOURCE_REPO=https://github.com/autowarefoundation/autoware
ARG AUTOWARE_PACKAGES_UP_TO=autoware_launch

WORKDIR ${AUTOWARE_DEVEL_ROOT}

RUN git clone ${AUTOWARE_SOURCE_REPO} --depth=1 ${AUTOWARE_DEVEL_ROOT} \
    ; mkdir -p ${AUTOWARE_DEVEL_SOURCE_DIR} && \
        vcs import ${AUTOWARE_DEVEL_SOURCE_DIR}\
        --shallow \
        --input ${AUTOWARE_DEVEL_ROOT}/autoware.repos \
    ; . /opt/ros/${ROS_DISTRO}/setup.sh \
    ; ccache --show-config \
    ; ccache --zero-stats \
    ; colcon --log-base /dev/null build \
        --packages-up-to ${AUTOWARE_PACKAGES_UP_TO} \
        --parallel-workers 4 \
        --event-handlers \
            console_direct- \
            console_stderr+ \
            console_cohesion- \
            console_start_end- \
            console_package_list+ \
            status- \
            summary+ \
            desktop_notification- \
    ; ccache -v --show-stats

FROM ${IMAGE_NAME}:latest-base AS autoware-runtime
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

# This is not complete. It is just a placeholder for the final image.
ARG CUDA_TOOLKIT_VERSION_MAJOR=12 
ARG CUDA_TOOLKIT_VERSION_MINOR=6
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt set -eux \
    ; export DEBIAN_FRONTEND=noninteractive \
    ; apt-get -qq update && apt-get install -qqy --no-install-recommends \
        libcublas-${CUDA_TOOLKIT_VERSION_MAJOR}-${CUDA_TOOLKIT_VERSION_MINOR} \
        libcurand-${CUDA_TOOLKIT_VERSION_MAJOR}-${CUDA_TOOLKIT_VERSION_MINOR} \
        libnvinfer8 \
        libnvinfer-plugin8 \
        libnvparsers8 \
        libnvonnxparsers8 \
    ; apt-get autoremove -y \
    ; rm -rf /var/lib/apt/lists/* 

ENV AUTOWARE_INSTALL_DIR=/opt/autoware
COPY --from=autoware-prebuilt ${AUTOWARE_DEVEL_ROOT}/install ${AUTOWARE_INSTALL_DIR}

ARG AUTOWARE_VERSION
ENV AUTOWARE_RUNTIME_VERSION=${AUTOWARE_VERSION:-latest}
RUN echo "AUTOWARE_RUNTIME_VERSION=${AUTOWARE_RUNTIME_VERSION}" >> /etc/environment

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
