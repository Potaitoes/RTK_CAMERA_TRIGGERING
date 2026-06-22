#!/usr/bin/env python3
import os
import sys
import subprocess
import time

# ===== CONFIG =====
HID_PATH = "/dev/hidraw3"   #HID address of the camera (stable udev symlink).
BUFFER_LENGTH = 65
VIDEO_DEV = "/dev/video1"
EXPOSURE_TIME = 167  # in 100µs units (100 = 10ms)

# ===== PROTOCOL =====
CAMERA_CONTROL = 0xA8 ##define CAMERA_CONTROL_24CUG 0xA8

# Commands
SET_STREAM_MODE_CU135 = 0x1C ##define SET_STREAM_MADE_24CUG                0x1C
SET_TO_DEFAULT_CU135 = 0xFF ##define SET_TO_DEFAULT_24CUG                 0xFF

# Values
STREAM_TRIGGER = 0x01 # inferred from qtCam // same for 24ug camera.

# Response
SET_SUCCESS = 0x01   # inferred from qtCam
SET_FAIL = 0x00


# ===== CORE FUNCTION =====
def send_cmd(cmd, value=0x00, verbose=True):
    if not os.path.exists(HID_PATH):
        print(f"❌ HID device not found: {HID_PATH}")
        return None

    try:
        # Open non-blocking so reads don't hang indefinitely
        fd = os.open(HID_PATH, os.O_RDWR | os.O_NONBLOCK)
    except FileNotFoundError:
        print(f"❌ HID device not found: {HID_PATH}")
        return None
    except PermissionError:
        print(f"❌ Permission denied opening {HID_PATH}; try running as root or adjust udev rules")
        return None

    out_buf = bytearray(BUFFER_LENGTH)
    out_buf[0] = CAMERA_CONTROL
    out_buf[1] = cmd
    out_buf[2] = value

    try:
        os.write(fd, out_buf)
    except OSError as e:
        print(f"❌ Failed to write to HID device: {e}")
        os.close(fd)
        return None

    # Wait for a response with a short timeout
    resp = bytes()
    try:
        import select

        rlist, _, _ = select.select([fd], [], [], 1.0)
        if rlist:
            try:
                resp = os.read(fd, BUFFER_LENGTH)
            except BlockingIOError:
                resp = bytes()
            except OSError as e:
                print(f"❌ Error reading HID device: {e}")
                resp = bytes()
    finally:
        try:
            os.close(fd)
        except OSError:
            pass

    if verbose:
        try:
            print("Response:", list(resp[:10]))
        except Exception:
            print("Response: <unreadable>")

    # Basic validation
    if len(resp) >= 7:
        if resp[0] == CAMERA_CONTROL and resp[1] == cmd:
            if resp[6] == SET_SUCCESS:
                print("✅ Success")
                return True
            elif resp[6] == SET_FAIL:
                print("❌ Command failed")
                return False

    print("⚠️ No/invalid response (may still have worked)")
    return None


def set_effect_sketch():
    return send_cmd(SET_STREAM_MODE_CU135, STREAM_TRIGGER)


def set_exposure(dev=VIDEO_DEV, exposure=EXPOSURE_TIME):
    """Set manual exposure via v4l2-ctl."""
    try:
        subprocess.run(
            ["v4l2-ctl", "-d", dev, "--set-ctrl=auto_exposure=1"],
            check=True
        )
        print(f"✅ auto_exposure set to manual (1)")
        time.sleep(1)
        subprocess.run(
            ["v4l2-ctl", "-d", dev, f"--set-ctrl=exposure_time_absolute={exposure}"],
            check=True
        )
        print(f"✅ exposure_time_absolute set to {exposure}")
    except FileNotFoundError:
        print("❌ v4l2-ctl not found — install v4l-utils")
    except subprocess.CalledProcessError as e:
        print(f"❌ v4l2-ctl failed: {e}")


def set_default_mode():
    return send_cmd(SET_TO_DEFAULT_CU135, 0x00)


# ===== CLI =====
def main():
    if len(sys.argv) != 2:
        print("Usage: {} <mode>\n  mode: 0 = default values, 1 = stream trigger mode".format(sys.argv[0]))
        sys.exit(1)

    try:
        mode = int(sys.argv[1])
    except ValueError:
        print("Invalid mode: must be 0 or 1")
        sys.exit(1)

    if mode == 0:
        print("Setting camera to default values")
        set_default_mode()
    elif mode == 1:
        print("Setting camera to stream trigger mode")
        set_effect_sketch()
        #print("Setting manual exposure...")
       # set_exposure()
    else:
        print("Invalid mode: must be 0 or 1")
        sys.exit(1)


if __name__ == "__main__":
    main()
