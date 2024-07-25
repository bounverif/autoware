FROM ghcr.io/bounverif/autoware:latest-builder-with-cache AS autoware-devel

ARG TARGETARCH
ARG TARGETOS
ARG TARGETPLATFORM
ARG TARGETVARIANT

RUN wget -qO- "https://keyserver.ubuntu.com/pks/lookup?fingerprint=on&op=get&search=0x6125E2A8C77F2818FB7BD15B93C4A3FD7BB9C367" | gpg --dearmour -o /usr/share/keyrings/ansible-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ansible-archive-keyring.gpg] http://ppa.launchpad.net/ansible/ansible/ubuntu jammy main" | tee /etc/apt/sources.list.d/ansible.list

RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && apt-get install -qy --no-install-recommends \
        ansible-core \
        ansible \
    && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

USER bounverif
WORKDIR /home/bounverif

ENV AUTOWARE_VERSION=devel
ENV AUTOWARE_SOURCE_DIR=/home/bounverif/autoware/src
ENV AUTOWARE_BUILD_DIR=/tmp/build/autoware
ENV AUTOWARE_INSTALL_DIR=/home/bounverif/autoware/install

ENV AUTOWARE_REPOSITORY_URL=https://github.com/autowarefoundation/autoware.git
ENV AUTOWARE_CORE_REPOSITORY_URL=https://github.com/autowarefoundation/autoware.core.git
ENV AUTOWARE_UNIVERSE_REPOSITORY_URL=https://github.com/autowarefoundation/autoware.universe.git

