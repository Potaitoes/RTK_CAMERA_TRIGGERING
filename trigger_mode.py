#!/usr/bin/env python3
import os
import sys

# ===== CONFIG =====
HID_PATH = "/dev/hidraw0"   #HID address of the camera.
BUFFER_LENGTH = 65

# ===== PROTOCOL =====
CAMERA_CONTROL = 0x81

# Commands
SET_STREAM_MODE_CU135 = 0x11
SET_TO_DEFAULT_CU135 = 0x12

# Values

STREAM_TRIGGER = 0x01 # inferred from qtCam

# Response
SET_SUCCESS = 0x01   # inferred from qtCam
SET_FAIL = 0x00


# ===== CORE FUNCTION =====
def send_cmd(cmd, value=0x00, verbose=True):
    fd = os.open(HID_PATH, os.O_RDWR)

    out_buf = bytearray(BUFFER_LENGTH)
    out_buf[0] = CAMERA_CONTROL     # 0x81
    out_buf[1] = cmd
    out_buf[2] = value

    os.write(fd, out_buf)

    try:
        resp = os.read(fd, BUFFER_LENGTH)
    except BlockingIOError:
        resp = bytes()

    os.close(fd)

    if verbose:
        print("Response:", list(resp[:10]))

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
    else:
        print("Invalid mode: must be 0 or 1")
        sys.exit(1)


if __name__ == "__main__":
    main()
