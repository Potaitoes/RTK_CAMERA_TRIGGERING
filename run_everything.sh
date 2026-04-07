#!/usr/bin/env bash

BASE_DIR="/home/vitom/recordSample"
SESSION_NAME="$(date +%Y%m%d_%H%M%S)"
SESSION_DIR="${BASE_DIR}/${SESSION_NAME}"

# If folder already exists, append a suffix
if [ -d "${SESSION_DIR}" ]; then
    i=1
    while [ -d "${SESSION_DIR}_${i}" ]; do
        i=$((i + 1))
    done
    SESSION_DIR="${SESSION_DIR}_${i}"
fi

mkdir -p "${SESSION_DIR}"
echo "[run_everything] Session folder: ${SESSION_DIR}"

# # Stop PTP service
# sudo systemctl stop enable_ptp.service
# sudo systemctl daemon-reload
# echo "[run_everything] PTP service stopped"

#sleep 2  # Give it a moment to ensure PTP is fully stopped

# Clean up background processes on exit
cleanup() {
    echo "[run_everything] Stopping background processes..."
    kill $LAUNCH_PID $BAG_PID 2>/dev/null
    wait $LAUNCH_PID $BAG_PID 2>/dev/null
    echo "[run_everything] Done."
}
trap cleanup EXIT INT TERM

ros2 launch motiv lidar.launch.py &
LAUNCH_PID=$!

sleep 10  # Wait for the launch to initialize

ros2 bag record /imu/data /imu/data_raw /lidar_points --max-bag-duration 180 -o "${SESSION_DIR}" &
BAG_PID=$!

sleep 5  # Ensure the bag recording has started

./build/capture_session --video /dev/video0 --rtk /dev/ttyACM0 --out "${SESSION_DIR}"