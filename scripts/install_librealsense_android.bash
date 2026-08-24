#!/bin/bash
#
# Builds librealsense2 for Android and installs it where RTAB-Map's Android build
# looks for its dependencies, so that the "RealSense (USB)" camera driver of the
# Android app gets enabled (see app/android/jni/CameraMobileRealSense.cpp).
#
# The docker recipe does the same thing, see docker/noble/android/Dockerfile. Use
# this script instead when you build RTAB-Map for Android on the host.
#
# Usage:
#   ./scripts/install_librealsense_android.bash <librealsense_source_dir> [install_prefix] [abi] [api_level]
#
# Example:
#   export ANDROID_NDK=$HOME/Android/Sdk/ndk/25.1.8937393
#   ./scripts/install_librealsense_android.bash ../librealsense /opt/android arm64-v8a 24
#
# Then configure RTAB-Map for Android as usual, CMake picks librealsense up
# through CMAKE_FIND_ROOT_PATH:
#   cmake -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
#         -DANDROID_ABI=arm64-v8a -DANDROID_NATIVE_API_LEVEL=24 ... \
#         -DCMAKE_FIND_ROOT_PATH="/opt/android/arm64-v8a/bin;/opt/android/arm64-v8a;/opt/android/arm64-v8a/share" ..
# and "With RealSense = YES" should appear in the "Android build info" summary.

set -e

if [ $# -lt 1 ]; then
    echo "Usage: $0 <librealsense_source_dir> [install_prefix] [abi] [api_level]"
    exit 1
fi

SRC=$(cd "$1" && pwd)
PREFIX=${2:-/opt/android}
ABI=${3:-arm64-v8a}
API=${4:-24}

if [ -z "$ANDROID_NDK" ]; then
    echo "ANDROID_NDK is not set (e.g. export ANDROID_NDK=\$HOME/Android/Sdk/ndk/25.1.8937393)"
    exit 1
fi

if [ ! -f "$SRC/CMake/android_config.cmake" ]; then
    echo "$SRC doesn't look like a librealsense source directory"
    exit 1
fi

BUILD="$SRC/build-android-$ABI-$API"
mkdir -p "$BUILD"

# * FORCE_RSUSB_BACKEND is mandatory on Android: the SDK cannot access /dev/bus/usb,
#   the USB file descriptor is handed over from the java side (DeviceWatcher.java).
# * BUILD_ROSBAG2=OFF: its fastcdr dependency is not installed, so it would be left
#   dangling in realsense2Targets.cmake and break find_package(realsense2) downstream.
# * BUILD_RS2_ALL=OFF: the bundled static library is not installed anyway, and its
#   output path is broken when CMAKE_ARCHIVE_OUTPUT_DIRECTORY is not set.
# * -include cerrno: the vendored nlohmann/json uses errno without including it,
#   which older NDK libc++ (r21) does not pull in transitively.
cmake -S "$SRC" -B "$BUILD" \
    -DCMAKE_TOOLCHAIN_FILE="$ANDROID_NDK/build/cmake/android.toolchain.cmake" \
    -DANDROID_ABI="$ABI" \
    -DANDROID_NDK="$ANDROID_NDK" \
    -DANDROID_NATIVE_API_LEVEL="$API" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DBUILD_ROSBAG2=OFF \
    -DBUILD_WITH_DDS=OFF \
    -DBUILD_RS2_ALL=OFF \
    -DENABLE_CCACHE=OFF \
    -DCMAKE_CXX_FLAGS="-include cerrno" \
    -DCMAKE_INSTALL_PREFIX="$PREFIX/$ABI" \
    -DCMAKE_FIND_ROOT_PATH="$PREFIX/$ABI/bin;$PREFIX/$ABI;$PREFIX/$ABI/share"

cmake --build "$BUILD" -j"$(nproc)"

# The static libraries are huge (~400 MB) with debug symbols.
STRIP=$(find "$ANDROID_NDK/toolchains/llvm/prebuilt" -name "llvm-strip" | head -1)
if [ -n "$STRIP" ]; then
    find "$BUILD" -type f -name "*.a" -print0 | xargs -0 "$STRIP" -g -S -d --strip-debug
fi

cmake --install "$BUILD"

echo ""
echo "librealsense2 installed in $PREFIX/$ABI"
