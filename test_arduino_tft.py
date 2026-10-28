"""
Diagnostic utility to test serial communication between Raspberry Pi and Arduino Uno TFT shield.
Run this directly on your Raspberry Pi:
    python3 test_arduino_tft.py
"""

import sys
import time
import glob
import serial

print("=" * 60)
print("  ARDUINO UNO TFT SHIELD DIAGNOSTIC TOOL")
print("=" * 60)

# Discover serial ports
ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/cu.usb*"))
print(f"[*] Detected serial devices: {ports}")

if not ports:
    print("[!] ERROR: No Arduino serial device found!")
    print("    Check your USB cable connection between Pi and Arduino Uno.")
    sys.exit(1)

port = ports[0]
print(f"[*] Connecting to: {port} @ 115200 baud...")

try:
    # Explicitly disable DTR toggle to prevent continuous reset loops
    ser = serial.Serial(port, 115200, timeout=2)
    ser.dtr = False
    ser.rts = False
    
    print("[*] Waiting 2.5 seconds for Arduino initialization...")
    time.sleep(2.5)
    
    # Read any startup banners from Arduino
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"    [Arduino Startup]: {line}")

    # Test 0: Handshake PING
    print("\n[0/3] Sending Handshake PING...")
    ser.write(b"PING\n")
    ser.flush()
    time.sleep(0.5)
    while ser.in_waiting:
        print(f"    [Arduino Response]: {ser.readline().decode('utf-8', errors='ignore').strip()}")

    # Test 1: Idle Screen
    print("\n[1/3] Sending IDLE:0...")
    ser.write(b"IDLE:0\n")
    ser.flush()
    time.sleep(1)
    while ser.in_waiting:
        print(f"    [Arduino Response]: {ser.readline().decode('utf-8', errors='ignore').strip()}")

    # Test 2: Green Checkmark
    print("\n[2/3] Sending GREEN CHECKMARK (OK:Niyanth_B:AM.SC.U4AIE24163)...")
    ser.write(b"OK:Niyanth_B:AM.SC.U4AIE24163\n")
    ser.flush()
    time.sleep(1)
    while ser.in_waiting:
        print(f"    [Arduino Response]: {ser.readline().decode('utf-8', errors='ignore').strip()}")

    print("\n[*] Check your Arduino screen now! Did it show the Green Checkmark?")
    time.sleep(4)

    # Test 3: Red X Mark
    print("\n[3/3] Sending RED X MARK (UNKNOWN)...")
    ser.write(b"UNKNOWN\n")
    ser.flush()
    time.sleep(1)
    while ser.in_waiting:
        print(f"    [Arduino Response]: {ser.readline().decode('utf-8', errors='ignore').strip()}")

    print("\n[*] Did it show the Red X Mark?")
    time.sleep(3)

    ser.write(b"IDLE:1\n")
    ser.flush()
    ser.close()
    print("\n[✓] Diagnostic finished.")

except Exception as e:
    print(f"[!] Serial Communication Error: {e}")
