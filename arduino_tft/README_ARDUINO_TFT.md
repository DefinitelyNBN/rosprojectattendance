# Arduino Uno TFT LCD Shield Guide

This guide explains how to use an **Arduino Uno with a 2.4" or 2.8" TFT LCD Shield** (MCUFRIEND / SPFD5408 / ILI9341 8-bit parallel shield) as the visual feedback display for the Face Recognition Attendance Robot.

---

## 📐 Architecture & Connection Overview

When using an Arduino Uno TFT Shield:
1. The **TFT Shield plugs directly on top of the Arduino Uno** (it uses the standard Uno shield pin headers; no jumper wires needed between Uno and TFT).
2. The **Arduino Uno connects to any USB port on the Raspberry Pi 4B** via a standard USB Type-A to Type-B cable.
3. The Raspberry Pi communicates with the Arduino over USB Serial at **`115200 baud`**:
   - Sends `OK:<NAME>:<ID>` when a face in the database is verified ➔ Arduino draws a large **Green Checkmark (✔)**, student name, and ID.
   - Sends `UNKNOWN` when an unrecognized face is detected ➔ Arduino draws a large **Red X Mark (✖)** and displays **Access Denied**.
   - Sends `IDLE:<COUNT>` when scanning ➔ Arduino displays the **Radar Reticle** and current attendance count.

```
+-------------------+           USB Cable          +---------------+
|  Raspberry Pi 4B  |==============================|  Arduino Uno  |
|  (Face Recog /    |      (/dev/ttyACM0 or        |  (Receives    |
|   ROS 2 Engine)   |       /dev/ttyUSB0)          |   Serial Cmd) |
+-------------------+                              +-------+-------+
                                                           | (Plugs on top)
                                                    +---------------+
                                                    |  TFT Shield   |
                                                    |  2.4"/2.8" LCD|
                                                    |  ✔ Green / ✖ Red|
                                                    +---------------+
```

---

## 🛠️ Step 1: Install Arduino Libraries

Open the **Arduino IDE** on your computer and install the following libraries via **Tools ➔ Manage Libraries...**:

1. **Adafruit GFX Library** by Adafruit (Core graphics library)
2. **MCUFRIEND_kbv** by David Prentice (Driver for 99% of 2.4"/2.8" Uno TFT shields)

---

## 💻 Step 2: Upload the Arduino Sketch

1. Connect the Arduino Uno to your computer with the USB cable.
2. In the Arduino IDE, open:
   ```text
   rosprojectattendance/arduino_tft/arduino_tft.ino
   ```
3. Select your board and port:
   - **Board**: `Arduino Uno`
   - **Port**: Select the active COM / `/dev/cu.usbmodem` port.
4. Click **Upload** (Ctrl + U or ⌘ + U).
5. Once uploaded, the screen will initialize in landscape mode (320x240) showing:
   - Top Header: `SMILE PLEASE :)`
   - Center: Animated cyan radar reticle
   - Bottom: `ATTENDED: 0`

---

## 🔌 Step 3: Connect Arduino to Raspberry Pi

1. Unplug the Arduino from your computer.
2. Plug the USB cable directly into one of the **Raspberry Pi 4B's USB ports**.
3. On the Raspberry Pi, verify that the Arduino is recognized:
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB*
   ```
   *(It will typically show as `/dev/ttyACM0`)*.

---

## 🧪 Step 4: Test Serial Commands (Optional)

You can test the screen directly from the Raspberry Pi terminal using Python:

```python
import serial, time

# Connect to Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2) # Wait for Arduino bootloader reset

# Test Green Checkmark
ser.write(b"OK:Niyanth_B:AM.SC.U4AIE24163\n")
time.sleep(4)

# Test Red X Mark
ser.write(b"UNKNOWN\n")
time.sleep(4)

# Return to Idle
ser.write(b"IDLE:5\n")
ser.close()
```

---

## 🚀 Step 5: Run the Complete System

The Raspberry Pi software automatically detects and connects to the Arduino Uno on launch:

```bash
python3 main.py
```

Whenever anyone stands in front of the camera:
- If **Verified**: The Arduino screen immediately flashes the **Green Checkmark (✔)** with their name and roll number, ROS 2 halts the robot for 5 seconds, and Excel logs the entry.
- If **Unknown**: The Arduino screen immediately flashes the **Red X Mark (✖)** and displays `ACCESS DENIED / UNREGISTERED`.
