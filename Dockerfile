ARG AUTOWARE_VERSION=latest
FROM ghcr.io/bounverif/autoware:cache-${AUTOWARE_VERSION} as autoware-buildcache

FROM ubuntu:22.04 as autoware-meta

ARG USER=bounverif
ARG USERGROUP=${USER}
ARG UID=1000
ARG GID=${UID}

LABEL org.opencontainers.image.vendor ="tr.edu.bogazici.cmpe.bounverif"
LABEL org.opencontainers.image.version="0.1.0"
LABEL org.opencontainers.image.authors="Bogazici University Verification Group"
LABEL org.opencontainers.image.url=""
LABEL org.opencontainers.image.documentation=""
LABEL org.opencontainers.image.source=""
LABEL org.opencontainers.image.title="Autoware"

RUN groupadd ${USERGROUP} -g ${GID} && \
    useradd -ms /bin/bash ${USER} -g ${USERGROUP} -u ${UID} && \
    printf "${USER} ALL= NOPASSWD: ALL\\n" >> /etc/sudoers

FROM ubuntu:22.04 as autoware-base

ARG TARGETARCH
ENV ID=ubuntu
ENV VERSION_ID=22.04
ENV CACHEMOUNT_PREFIX=/${TARGETARCH}/${ID}${VERSION_ID}

ARG USER=bounverif
ARG USERGROUP=${USER}
ARG UID=1000
ARG GID=${UID}

ARG AUTOWARE_VERSION=latest
ENV AUTOWARE_VERSION=${AUTOWARE_VERSION}

ENV DEBIAN_FRONTEND=noninteractive

# Debian containers save no deb archives by default. 
# The following command disables clean-up actions to enable
#   --mount=type=cache,target=/var/cache/apt 
# optimizations. Otherwise, apt caching does not work.
RUN rm -f /etc/apt/apt.conf.d/docker-clean

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${CACHEMOUNT_PREFIX}/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        tini \
        wget \
        gnupg2 \
        ca-certificates \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN groupadd ${USERGROUP} -g ${GID} && \
    useradd -ms /bin/bash ${USER} -g ${USERGROUP} -u ${UID} && \
    printf "${USER} ALL= NOPASSWD: ALL\\n" >> /etc/sudoers

RUN wget -qO- "https://keyserver.ubuntu.com/pks/lookup?fingerprint=on&op=get&search=0x6125E2A8C77F2818FB7BD15B93C4A3FD7BB9C367" | gpg --dearmour -o /usr/share/keyrings/ansible-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ansible-archive-keyring.gpg] http://ppa.launchpad.net/ansible/ansible/ubuntu jammy main" | tee /etc/apt/sources.list.d/ansible.list && \
    wget -qO- "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" | gpg --dearmour -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ jammy main" > /etc/apt/sources.list.d/autonomoustuff-public.list

ARG CUDA_ARCH=x86_64
ARG CUDA_DISTRO=ubuntu2204
ARG CUDA_KEYRING_PACKAGE=cuda-keyring_1.1-1_all.deb
ARG CUDA_KEYRING_FILEPATH=https://developer.download.nvidia.com/compute/cuda/repos/${CUDA_DISTRO}/${CUDA_ARCH}/${CUDA_KEYRING_PACKAGE}
    
RUN wget -q ${CUDA_KEYRING_FILEPATH} && dpkg -i ${CUDA_KEYRING_PACKAGE} && rm ${CUDA_KEYRING_PACKAGE}

FROM autoware-base as autoware-builder-nocuda

ENV AUTOWARE_SOURCE_DIR=/usr/local/src/autoware
ENV AUTOWARE_BUILD_DIR=/tmp/build/autoware
ENV AUTOWARE_INSTALL_DIR=/opt/autoware/humble

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${CACHEMOUNT_PREFIX}/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
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

COPY autoware.repos.yml /var/lib/autoware/autoware.repos.${AUTOWARE_VERSION}.yml

RUN --mount=type=cache,target=${AUTOWARE_SOURCE_DIR},id=autoware-src-${AUTOWARE_VERSION} \
    mkdir -p ${AUTOWARE_SOURCE_DIR} && vcs import --shallow ${AUTOWARE_SOURCE_DIR} < /var/lib/autoware/autoware.repos.${AUTOWARE_VERSION}.yml \
        || { rm -rf ${AUTOWARE_SOURCE_DIR}/* && vcs import --shallow ${AUTOWARE_SOURCE_DIR} < /var/lib/autoware/autoware.repos.${AUTOWARE_VERSION}.yml; }

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${CACHEMOUNT_PREFIX}/var/cache/apt \
    --mount=type=cache,target=${AUTOWARE_SOURCE_DIR},id=autoware-src-${AUTOWARE_VERSION},readonly \
    rosdep init && apt update && rosdep update && \
    rosdep install -y \
        --from-paths ${AUTOWARE_SOURCE_DIR} \
        --ignore-src \
        --rosdistro humble \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/* 

ENV CMAKE_GENERATOR=Ninja

ENV CMAKE_C_COMPILER_LAUNCHER=ccache
ENV CMAKE_CXX_COMPILER_LAUNCHER=ccache

ENV CCACHE_DIR=/var/cache/ccache

RUN mkdir -p ${CCACHE_DIR} && chmod 777 ${CCACHE_DIR}

FROM autoware-builder-nocuda as autoware-builder

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${CACHEMOUNT_PREFIX}/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
        cuda-minimal-build-12-4 \
        libcublas-dev-12-4 \
        libcurand-dev-12-4 \
        libcusolver-dev-12-4 \
        libnvinfer-dev=8.* \
        libnvinfer-headers-dev=8.* \
        libnvinfer-plugin-dev=8.* \
        libnvinfer-headers-plugin-dev=8.* \
        libnvparsers-dev=8.* \
        libnvonnxparsers-dev=8.* \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/* 

ENV PATH="/usr/local/cuda-12.4/bin:${PATH}"

# FROM autoware-builder as autoware-cache-prebuilt

# RUN --mount=type=cache,target=${CCACHE_DIR},id=autoware-cache-${AUTOWARE_VERSION} \
#     --mount=type=cache,target=${AUTOWARE_SOURCE_DIR},id=autoware-src-${AUTOWARE_VERSION},readonly \
#     ccache --zero-stats && \
#     . /opt/ros/humble/setup.sh && \
#     colcon build \
#         --base-paths ${AUTOWARE_SOURCE_DIR} \
#         --build-base ${AUTOWARE_BUILD_DIR} \
#         --install-base ${AUTOWARE_INSTALL_DIR} \
#         --packages-up-to autoware_launch \
#         --event-handlers console_cohesion+ console_package_list+ desktop_notification-\
#         --cmake-args \
#             -DCMAKE_BUILD_TYPE=Release \
#             " -Wno-dev" \
#             " --no-warn-unused-cli" \
#     && ccache -v --show-stats

# FROM autoware-base as autoware-cache

# ENV CCACHE_DIR=/var/cache/ccache
# RUN --mount=type=cache,target=${CCACHE_DIR},id=autoware-cache-${AUTOWARE_VERSION},readonly \
#     cp -r ${CCACHE_DIR} /usr/share/ccache

FROM autoware-builder as autoware-builder-with-cache

RUN --mount=type=cache,target=${AUTOWARE_SOURCE_DIR},id=autoware-src-${AUTOWARE_VERSION},readonly \
    --mount=type=cache,target=${AUTOWARE_BUILD_DIR},id=autoware-build-${AUTOWARE_VERSION} \
    ccache --zero-stats && \
    . /opt/ros/humble/setup.sh && \
    colcon build \
        --base-paths ${AUTOWARE_SOURCE_DIR} \
        --build-base ${AUTOWARE_BUILD_DIR} \
        --packages-up-to autoware_launch \
        --event-handlers console_cohesion+ console_package_list+ desktop_notification-\
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            " -Wno-dev" \
            " --no-warn-unused-cli" \
    && ccache -v --show-stats

FROM autoware-builder-with-cache as autoware-prebuilt

RUN --mount=type=cache,target=${AUTOWARE_SOURCE_DIR},id=autoware-src-${AUTOWARE_VERSION},readonly \
    --mount=type=cache,target=${AUTOWARE_BUILD_DIR},id=autoware-build-${AUTOWARE_VERSION} \
    ccache --zero-stats && \
    . /opt/ros/humble/setup.sh && \
    colcon build \
        --base-paths ${AUTOWARE_SOURCE_DIR} \
        --build-base ${AUTOWARE_BUILD_DIR} \
        --install-base ${AUTOWARE_INSTALL_DIR} \
        --packages-up-to autoware_launch \
        --event-handlers console_cohesion+ console_package_list+ desktop_notification-\
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            " -Wno-dev" \
            " --no-warn-unused-cli" \
    && ccache -v --show-stats

FROM autoware-base as autoware-runtime

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=${CACHEMOUNT_PREFIX}/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
        libcublas-12-4 \
        libcurand-12-4 \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/* 

COPY --from=autoware-prebuilt ${AUTOWARE_INSTALL_DIR} ${AUTOWARE_INSTALL_DIR}

FROM autoware-builder as autoware-devel

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
