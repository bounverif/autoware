#!/bin/bash
set -e

export AUTOWARE_ROOT="${HOME}/autoware"
export AUTOWARE_SOURCE_DIR="${AUTOWARE_ROOT}/src"
export AUTOWARE_BUILD_DIR="${AUTOWARE_ROOT}/build"
export AUTOWARE_INSTALL_DIR="${AUTOWARE_ROOT}/install"
export AUTOWARE_DATA_DIR="${AUTOWARE_ROOT}/data"

exec "$@"