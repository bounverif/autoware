ARG IMAGE_NAME=localhost/autoware
ARG AUTOWARE_VERSION=latest

FROM ${IMAGE_NAME}:${AUTOWARE_VERSION}-builder-with-cache AS autoware-prebuilt
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

RUN --mount=type=bind,source=etc/autoware/autoware.repos.yml,target=/etc/autoware/autoware.repos.yml \
    mkdir -p ${AUTOWARE_SOURCE_DIR} \
    && vcs import --shallow ${AUTOWARE_SOURCE_DIR} < /etc/autoware/autoware.repos.yml && \
    ccache --zero-stats && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon --log-base /dev/null build \
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
            -DCMAKE_BUILD_TYPE=Release \
            " -Wno-dev" \
            " --no-warn-unused-cli" \
    && rm -rf ${AUTOWARE_SOURCE_DIR} \
    && rm -rf ${AUTOWARE_BUILD_DIR} \
    && ccache -v --show-stats

FROM ${IMAGE_NAME}:${AUTOWARE_VERSION}-base AS autoware-runtime
ARG TARGETARCH TARGETOS TARGETPLATFORM TARGETVARIANT

# This is not complete. It is just a placeholder for the final image.

COPY --from=autoware-prebuilt ${AUTOWARE_INSTALL_DIR} ${AUTOWARE_INSTALL_DIR}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${TARGETPLATFORM}/var/cache/apt \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && apt-get install -y --no-install-recommends \
        libcublas-12-4 \
        libcurand-12-4 \
        libnvinfer8 \
        libnvinfer-plugin8 \
        libnvparsers8 \
        libnvonnxparsers8 \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/* 

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
