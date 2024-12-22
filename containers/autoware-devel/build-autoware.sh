#!/bin/sh -e

git clone "${AUTOWARE_SOURCE_REPO}" --depth=1 "${AUTOWARE_DEVEL_ROOT}"
mkdir -p "${AUTOWARE_DEVEL_SOURCE_DIR}"
vcs import "${AUTOWARE_DEVEL_SOURCE_DIR}" --shallow --input "${AUTOWARE_DEVEL_ROOT}/autoware.repos"
. /opt/ros/humble/setup.sh 
ccache --show-config 
ccache --zero-stats
colcon --log-base /dev/null build \
    --packages-up-to "${AUTOWARE_BUILD_PACKAGES_UP_TO}" \
    --base-paths "${AUTOWARE_DEVEL_SOURCE_DIR}" \
    --build-base "${AUTOWARE_DEVEL_BUILD_DIR}" \
    --install-base "${AUTOWARE_DEVEL_INSTALL_DIR}" \
    --parallel-workers "${AUTOWARE_BUILD_PARALLEL_WORKERS:-4}" \
    --merge-install \
    --event-handlers \
        console_cohesion+ \
    --cmake-args \
        " -Wno-dev" \
        " --no-warn-unused-cli" \
ccache -v --show-stats