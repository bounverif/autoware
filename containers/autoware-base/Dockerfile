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

# Autoware version
ARG AUTOWARE_VERSION
ENV AUTOWARE_VERSION=${AUTOWARE_VERSION:-latest}

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

# Autoware default directories
ENV AUTOWARE_SOURCE_DIR=/tmp/src
ENV AUTOWARE_BUILD_DIR=/tmp/build
ENV AUTOWARE_INSTALL_DIR=/opt/autoware

# CUDA Toolkit configuaration
ARG CUDA_ARCH=x86_64
ARG CUDA_DISTRO=ubuntu2204
ARG CUDA_KEYRING_PACKAGE=cuda-keyring_1.1-1_all.deb
ARG CUDA_KEYRING_FILEPATH=https://developer.download.nvidia.com/compute/cuda/repos/${CUDA_DISTRO}/${CUDA_ARCH}/${CUDA_KEYRING_PACKAGE}
ARG CUDA_TOOLKIT_VERSION_MAJOR=12
ARG CUDA_TOOLKIT_VERSION_MINOR=4

ENV CUDA_TOOLKIT_VERSION=${CUDA_TOOLKIT_VERSION_MAJOR}.${CUDA_TOOLKIT_VERSION_MINOR}
ENV CUDA_TOOLKIT_VERSION_DASHED=${CUDA_TOOLKIT_VERSION_MAJOR}-${CUDA_TOOLKIT_VERSION_MINOR}

# ROS configuration
ARG ROS_DISTRO=humble   
ENV ROS_DISTRO=${ROS_DISTRO}

# Repository management
RUN <<END
set -eux

wget -qO- "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key" | gpg --dearmour -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

wget -q ${CUDA_KEYRING_FILEPATH} 
dpkg -i ${CUDA_KEYRING_PACKAGE}
rm ${CUDA_KEYRING_PACKAGE}
END

# User management 
ARG USER=bounverif
ARG USERGROUP=${USER}
ARG UID=1000
ARG GID=${UID}

RUN groupadd ${USERGROUP} -g ${GID} && \
    useradd -lms /bin/bash ${USER} -g ${USERGROUP} -u ${UID} && \
    printf "%s ALL= NOPASSWD: ALL\\n" ${USER} >> /etc/sudoers
