"""
Raspberry Pi 4B Camera Diagnostic Tool
Identifies whether a USB Webcam or CSI Camera Module is connected and working.

Run on Raspberry Pi:
    python3 diagnose_camera.py
"""

import os
import sys
import glob
import subprocess
import cv2

print("=" * 60)
print("  RASPBERRY PI CAMERA DIAGNOSTIC TOOL")
print("=" * 60)

# 1. Check USB Devices (lsusb)
print("\n[1/5] Checking connected USB devices (lsusb)...")
try:
    res = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5)
    usb_lines = res.stdout.strip().split("\n")
    camera_found = False
    for line in usb_lines:
        print(f"    {line}")
        if any(w in line.lower() for w in ["cam", "video", "webcam", "uvc"]):
            camera_found = True
    if camera_found:
        print("  --> [✓] Potential USB Camera detected in USB bus.")
    else:
        print("  --> [!] No obvious USB webcam string in lsusb. Check physical connection.")
except Exception as e:
    print(f"    Could not run lsusb: {e}")

# 2. Check Raspberry Pi CSI Camera Module (libcamera / rpicam)
print("\n[2/5] Checking Raspberry Pi CSI Camera Module (libcamera / rpicam)...")
csi_found = False
for cmd in ["rpicam-hello", "libcamera-hello"]:
    try:
        res = subprocess.run([cmd, "--list-cameras"], capture_output=True, text=True, timeout=5)
        out = (res.stdout + res.stderr).strip()
        if out:
            print(f"    Output from {cmd} --list-cameras:")
            for l in out.split("\n"):
                print(f"      {l}")
            if "available" in out.lower() or "ov5647" in out.lower() or "imx" in out.lower():
                csi_found = True
                print("  --> [✓] Raspberry Pi CSI Camera Module detected!")
                break
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"    Error running {cmd}: {e}")

if not csi_found:
    print("  --> [!] No Raspberry Pi CSI Camera detected via libcamera/rpicam.")

# 3. Check V4L2 Device Nodes
print("\n[3/5] Listing /dev/video* devices...")
nodes = sorted(glob.glob("/dev/video*"))
print(f"    Found nodes: {nodes}")
try:
    res = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True, timeout=5)
    print("    v4l2-ctl device mappings:")
    for l in res.stdout.strip().split("\n"):
        print(f"      {l}")
except FileNotFoundError:
    print("    (v4l2-ctl not installed. Install with: sudo apt-get install v4l-utils)")
except Exception as e:
    print(f"    v4l2-ctl error: {e}")

# 4. Test Picamera2
print("\n[4/5] Testing Picamera2 Python library...")
try:
    from picamera2 import Picamera2
    print("    Picamera2 is installed.")
    try:
        p2 = Picamera2()
        cfg = p2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
        p2.configure(cfg)
        p2.start()
        frame = p2.capture_array()
        p2.stop()
        p2.close()
        print(f"  --> [✓] Picamera2 captured frame successfully! Shape: {frame.shape}")
    except Exception as e:
        print(f"    Picamera2 capture error: {e}")
except ImportError:
    print("    Picamera2 is NOT installed in this Python environment.")

# 5. Test OpenCV VideoCapture
print("\n[5/5] Testing OpenCV cv2.VideoCapture on detected nodes...")
working_indices = []
for node in nodes:
    try:
        idx = int(node.replace("/dev/video", ""))
        if idx > 5:
            continue
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                print(f"  --> [✓] Index {idx} ({node}) works! Frame shape: {frame.shape}")
                working_indices.append(idx)
            cap.release()
    except Exception:
        pass

print("\n" + "=" * 60)
print("  DIAGNOSTIC SUMMARY & RECOMMENDATION")
print("=" * 60)
if working_indices:
    print(f"[✓] Working OpenCV camera indices: {working_indices}")
    print(f"👉 Recommended command: python3 headless.py --cam {working_indices[0]}")
elif csi_found:
    print("[!] A Raspberry Pi CSI Camera Module was detected, but requires libcamera bridge.")
    print("👉 Recommended command: libcamerify python3 headless.py")
else:
    print("[!] No working camera detected.")
    print("1. If using a USB webcam: Ensure it is plugged into a blue USB 3.0 port and powered.")
    print("2. If using a Raspberry Pi CSI Camera: Ensure ribbon cable is seated firmly and enabled in raspi-config.")
    print("3. Try running: libcamerify python3 headless.py")
print("=" * 60 + "\n")
