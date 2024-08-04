ARG IMAGE_NAME=localhost/autoware
ARG AUTOWARE_VERSION=latest

FROM ${IMAGE_NAME}:latest-builder-with-cache AS autoware-prebuilt
# ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

WORKDIR ${AUTOWARE_ROOT}

RUN --mount=type=bind,source=etc/autoware/autoware.repos.yml,target=/etc/autoware/autoware.repos.yml set -ex \
    ; mkdir -p src \
    ; vcs import --shallow src --input /etc/autoware/autoware.repos.yml \
    ; export CCACHE_BASEDIR=${AUTOWARE_ROOT} \
    ; ccache --show-config \
    ; ccache --zero-stats \
    ; . /opt/ros/${ROS_DISTRO}/setup.sh \
    ; colcon --log-base /dev/null build \
        --packages-up-to autoware_launch \
        --parallel-workers 4 \
        --event-handlers \
            console_direct- \
            console_stderr+ \
            console_cohesion- \
            console_start_end- \
            console_package_list- \
            status- \
            summary+ \
            desktop_notification- \
    ; rm -rf ${AUTOWARE_ROOT} \
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

COPY --from=autoware-prebuilt ${AUTOWARE_ROOT}/install ${AUTOWARE_INSTALL_DIR}

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

ARG AUTOWARE_VERSION
ENV AUTOWARE_RUNTIME_VERSION=${AUTOWARE_VERSION:-latest}
RUN echo "AUTOWARE_RUNTIME_VERSION=${AUTOWARE_RUNTIME_VERSION}" >> /etc/environment
