#!/bin/bash

# Build script for ORB_SLAM3
# This script configures and builds ORB_SLAM3 with all dependencies

set -e  # Exit on error

echo "Building ORB_SLAM3..."

# Configure CMake
echo "Configuring build..."

# Install prefix precedence:
# - If CMAKE_INSTALL_PREFIX is set, it should win (explicit user intent).
# - Otherwise, if CONDA_PREFIX is set, install into the active conda env.
INSTALL_PREFIX_ARG=""
if [[ -n "${CMAKE_INSTALL_PREFIX:-}" ]]; then
    INSTALL_PREFIX_ARG="-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
elif [[ -n "${CONDA_PREFIX:-}" ]]; then
    INSTALL_PREFIX_ARG="-DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX}"
fi

cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    ${INSTALL_PREFIX_ARG}

# Build
echo "Building..."
cmake --build build -j8

echo "Build complete!"
