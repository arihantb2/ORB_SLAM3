#!/bin/bash

# Build script for ORB_SLAM3
# This script configures and builds ORB_SLAM3 with all dependencies

set -e  # Exit on error

echo "Building ORB_SLAM3..."

# Configure CMake
echo "Configuring build..."
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    ${CMAKE_INSTALL_PREFIX:+-DCMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX} \
    ${CONDA_PREFIX:+-DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX}

# Build
echo "Building..."
cmake --build build -j8

echo "Build complete!"
