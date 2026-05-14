#!/usr/bin/env bash
# Build script for capture_session C++ executable

set -e

mkdir -p build
cd build
cmake ..
make -j$(nproc)

echo ""
echo "✅ Build complete: ./build/capture_session"
echo ""
echo "Usage:"
echo "  ./build/capture_session [--video /dev/video0] [--rtk /dev/ttyACM0] [--out ./recordings]"
echo ""
