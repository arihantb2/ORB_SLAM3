#!/bin/bash

# Build script for ORB_SLAM3
# This script configures and builds ORB_SLAM3 with all dependencies

set -e  # Exit on error

echo "Building ORB_SLAM3..."

# Configure CMake
echo "Configuring build..."

BUILD_TYPE=${BUILD_TYPE:-Release}

cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_INSTALL_PREFIX=install/${BUILD_TYPE}

# Build
echo "Building..."
cmake --build build -j8

echo "Build complete!"

echo "Installing..."
cmake --build build --target install

echo "Installation complete!"