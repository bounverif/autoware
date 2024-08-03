ARG IMAGE_NAME=localhost/autoware
ARG BUILDER_VERSION=latest
ARG AUTOWARE_VERSION=latest

FROM ${IMAGE_NAME}:${BUILDER_VERSION}-builder-with-cache AS autoware-prebuilt
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

RUN echo "${AUTOWARE_VERSION}"

RUN --mount=type=bind,source=etc/autoware/autoware.repos.yml,target=/etc/autoware/autoware.repos.yml set -ex \
    ; set -ex \
    ; mkdir -p ${AUTOWARE_SOURCE_DIR} \
    ; vcs import --shallow ${AUTOWARE_SOURCE_DIR} < /etc/autoware/autoware.repos.yml \
    ; ccache --get-config base_dir \
    ; ccache --zero-stats \
    ; . /opt/ros/${ROS_DISTRO}/setup.sh \
    ; colcon --log-base /dev/null build \
        --base-paths ${AUTOWARE_SOURCE_DIR} \
        --build-base ${AUTOWARE_BUILD_DIR} \
        --install-base ${AUTOWARE_INSTALL_DIR} \
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
    ; rm -rf ${AUTOWARE_SOURCE_DIR} \
    ; rm -rf ${AUTOWARE_BUILD_DIR} \
    ; ccache -v --show-stats

FROM ${IMAGE_NAME}:${BUILDER_VERSION}-base AS autoware-runtime
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

# This is not complete. It is just a placeholder for the final image.

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt set -eux \
    ; export DEBIAN_FRONTEND=noninteractive \
    ; apt-get -qq update && apt-get install -qqy --no-install-recommends \
        libcublas-12-4 \
        libcurand-12-4 \
        libnvinfer8 \
        libnvinfer-plugin8 \
        libnvparsers8 \
        libnvonnxparsers8 \
    ; apt-get autoremove -y \
    ; rm -rf /var/lib/apt/lists/* 

COPY --from=autoware-prebuilt ${AUTOWARE_INSTALL_DIR} ${AUTOWARE_INSTALL_DIR}

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
