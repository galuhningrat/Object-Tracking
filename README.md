# Object Tracking System with Servo Control
## Raspberry Pi 5 + OpenCV 4.11.0 + PCA9685

[![Python Version](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/downloads/)
[![OpenCV Version](https://img.shields.io/badge/opencv-4.11.0-green.svg)](https://opencv.org/)
[![License](https://img.shields.io/badge/license-MIT-orange.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-red.svg)](https://www.raspberrypi.com/)

A real-time computer vision system that tracks objects and controls pan-tilt servos automatically using OpenCV on Raspberry Pi 5. Features PID control, video recording, and manual calibration mode.

![System Demo](docs/images/demo.gif)
*Real-time object tracking with automatic servo control*

---

## 🌟 Features

- **Real-time Object Tracking** using OpenCV trackers (CSRT, KCF, MOSSE, MIL)
- **PID Control System** for smooth and precise servo movements
- **Video Recording** in MP4 format with live preview
- **Digital Zoom** for focusing on specific areas
- **Manual Control Mode** for servo calibration
- **Anti-Drift Protection** with dead zone and integral windup prevention
- **Low Latency** (~40ms) for responsive tracking
- **Modular Architecture** for easy customization

---

## 📋 Table of Contents

- [Hardware Requirements](#-hardware-requirements)
- [Software Requirements](#-software-requirements)
- [Installation](#-installation)
  - [Quick Start](#quick-start)
  - [Manual Installation](#manual-installation)
- [Hardware Setup](#-hardware-setup)
- [Configuration](#-configuration)
- [Usage](#-usage)
- [Troubleshooting](#-troubleshooting)
- [Performance](#-performance)
- [Advanced Topics](#-advanced-topics)
- [Contributing](#-contributing)
- [License](#-license)
- [Acknowledgments](#-acknowledgments)

---

## 🔧 Hardware Requirements

| Component | Specification | Quantity | Notes |
|-----------|--------------|----------|-------|
| **Raspberry Pi 5** | 4GB/8GB RAM, ARMv8 @ 2.4GHz | 1 | Recommended for optimal performance |
| **Camera Module** | OV5647 5MP CSI camera | 1 | Compatible with Picamera2 |
| **PCA9685 HAT** | 16-channel PWM driver, I2C | 1 | Waveshare recommended |
| **Servo Motors** | DS3218MG or similar (digital) | 2 | Pan + Tilt |
| **Pan-Tilt Bracket** | Aluminum or plastic | 1 | For mounting servos |
| **Power Supply (RPi)** | 5V/3A USB-C | 1 | Official PSU recommended |
| **Power Supply (Servo)** | 5-6V/3-5A DC | 1 | **CRITICAL: External power required** |
| **MicroSD Card** | 64GB Class 10 or better | 1 | For OS and data storage |
| **Cooling** | Heatsink + Fan | 1 | Prevents thermal throttling |

### ⚠️ Critical Power Requirements

**Servos MUST have external power supply!** GPIO pins cannot provide sufficient current (max ~50mA vs servo's 2.5A peak).

**Symptoms of inadequate power:**
- Servo jittering or not moving
- Raspberry Pi random restarts
- I2C communication errors

**Solution:** Connect 5V/3A+ external PSU to PCA9685 VIN/GND pins.

---

## 💻 Software Requirements

- **OS:** Raspberry Pi OS (64-bit, Debian Trixie)
- **Python:** 3.11+
- **OpenCV:** 4.11.0 (compiled from source)
- **Picamera2:** Latest version
- **Libraries:** numpy, smbus2, adafruit-circuitpython-pca9685 (optional)

---

## 📦 Installation

### Quick Start

Run the automated installation script:

```bash
# Clone repository
git clone https://github.com/yourusername/object-tracking-servo.git
cd object-tracking-servo

# Run installation script (takes 2-3 hours)
chmod +x scripts/install_opencv_rpi5.sh
./scripts/install_opencv_rpi5.sh

# Install Python dependencies
pip3 install -r requirements.txt

# Enable I2C and Camera
sudo raspi-config
# Navigate: Interface Options -> I2C (Enable)
# Navigate: Interface Options -> Legacy Camera (Disable - use libcamera)

# Reboot
sudo reboot
```

### Manual Installation

<details>
<summary>Click to expand detailed installation steps</summary>

#### 1. System Update

```bash
sudo apt update && sudo apt upgrade -y
```

#### 2. Create Swap File (4GB)

```bash
sudo swapoff -a
sudo rm -f /swapfile
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

#### 3. Install Dependencies

```bash
sudo apt install -y \
    build-essential cmake git unzip pkg-config \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev v4l-utils libxvidcore-dev libx264-dev \
    libgtk-3-dev libcanberra-gtk3-module \
    libopenblas-dev liblapack-dev gfortran \
    python3-dev python3-numpy libtbb-dev \
    libdc1394-dev qtbase5-dev liblapacke-dev \
    i2c-tools python3-smbus2 python3-picamera2
```

#### 4. Compile OpenCV 4.11.0

```bash
cd ~
git clone -b 4.11.0 https://github.com/opencv/opencv.git
git clone -b 4.11.0 https://github.com/opencv/opencv_contrib.git

mkdir -p ~/opencv/build && cd ~/opencv/build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D WITH_OPENMP=ON \
    -D WITH_OPENCL=OFF \
    -D WITH_FFMPEG=ON \
    -D WITH_TBB=ON \
    -D WITH_GTK=ON \
    -D WITH_V4L=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_EXAMPLES=OFF ..

make -j$(nproc)  # Takes 2-3 hours
sudo make install
sudo ldconfig
```

#### 5. Verify Installation

```bash
python3 -c "import cv2; print(cv2.__version__)"
# Expected output: 4.11.0
```

</details>

---

## 🔌 Hardware Setup

### Wiring Diagram

```
Raspberry Pi 5 GPIO:
┌─────────────────────────────┐
│ Pin 3 (SDA) ──────► PCA9685 SDA
│ Pin 5 (SCL) ──────► PCA9685 SCL
│ Pin 6 (GND) ──────► PCA9685 GND (signal reference)
└─────────────────────────────┘

PCA9685 Connections:
┌─────────────────────────────┐
│ VIN ◄────── External PSU (+5V)
│ GND ◄────── External PSU (GND) + RPi GND
│ 
│ Channel 0 ──► Servo Pan
│ Channel 1 ──► Servo Tilt
└─────────────────────────────┘

⚠️ CRITICAL: Remove V+ jumper on PCA9685!
```

### Camera Connection

Connect OV5647 camera module to CSI port using ribbon cable (contacts facing inward toward Ethernet port).

### Verification

```bash
# Check I2C devices
i2cdetect -y 1
# Should show: 40 (PCA9685 address)

# Test camera
libcamera-hello --list-cameras
# Should detect: ov5647
```

---

## ⚙️ Configuration

Edit `config.json` or modify `CONFIG` dictionary in source code:

```python
CONFIG = {
    "camera": {
        "width": 640,        # Resolution (320/640/1280)
        "height": 480,       # Resolution (240/480/720)
        "flip": False        # Mirror image horizontally
    },
    "servo": {
        "pan_channel": 0,    # PCA9685 channel for pan
        "tilt_channel": 1,   # PCA9685 channel for tilt
        "pan_center_pulse": 1500,   # Center position (µs)
        "tilt_center_pulse": 1500,
        "pan_min_pulse": 750,       # Safety limits
        "pan_max_pulse": 2250,
        "tilt_min_pulse": 800,
        "tilt_max_pulse": 2200,
        "pan_inversion": 1,         # Direction: 1 or -1
        "tilt_inversion": -1,
        "manual_step_pulse": 30     # Manual control step size
    },
    "pid_pan": {
        "kp": 0.05,          # Proportional gain (0.05-0.20)
        "ki": 0.0,           # Integral gain (0 = anti-drift)
        "kd": 0.02,          # Derivative gain (0.01-0.08)
        "max_integral": 0,   # Anti-windup limit
        "dead_zone": 5       # Pixels ±5 tolerance
    },
    "pid_tilt": {
        "kp": 0.06,
        "ki": 0.0,
        "kd": 0.025,
        "max_integral": 0,
        "dead_zone": 5
    },
    "tracking": {
        "tracker_type": "csrt",      # csrt/kcf/mosse/mil
        "tolerance_ratio": 0.08      # Dead zone size (8%)
    }
}
```

### PID Tuning Guide

| Parameter | Effect When Increased | Recommended Range |
|-----------|----------------------|-------------------|
| **Kp** | Faster response, more overshoot | 0.05 - 0.20 |
| **Ki** | Eliminates steady-state error, causes drift | **0.0 (disabled)** |
| **Kd** | Better damping, noise sensitive | 0.01 - 0.08 |

**Anti-Drift Configuration:**
- Set `ki = 0.0` to prevent servo drift when object is centered
- Set `max_integral = 0` to disable integral accumulation
- Use `dead_zone` to create tolerance region (no movement if error < threshold)

---

## 🚀 Usage

### Main Tracking Program

```bash
python3 main5_all_in_4_record_video.py
```

**Keyboard Controls:**

| Key | Action |
|-----|--------|
| **S** | Select object ROI and start tracking |
| **I/K** | Manual tilt control (Up/Down) |
| **J/L** | Manual pan control (Left/Right) |
| **+/-** | Digital zoom in/out |
| **R** | Toggle video recording |
| **Q** | Quit program |

### Manual Calibration Mode

```bash
python3 manual_control.py
```

Use arrow keys or I/J/K/L to move servos manually. Find optimal pulse ranges and center positions.

### Test Scripts

```bash
# Test camera
python3 test_camera.py

# Test I2C communication
python3 test_i2c.py

# Test servo movement
python3 servo_test.py
```

---

## 🐛 Troubleshooting

### Servo Not Moving

**Problem:** Servo jittering, not moving, or Raspberry Pi restarting.

**Solution:**
1. Connect external 5V/3A PSU to PCA9685 VIN/GND
2. Remove V+ jumper on PCA9685
3. Verify voltage at servo pins: `multimeter VIN-GND = 5V`
4. Check I2C: `i2cdetect -y 1` (should show `40`)

### Tracking Lost Immediately

**Problem:** Bounding box disappears after 1-2 frames.

**Solution:**
- Select object with high contrast vs background
- Increase resolution to 640x480
- Try KCF tracker instead of CSRT
- Improve lighting conditions
- Slow down object movement

### Low Frame Rate (<15 fps)

**Problem:** Laggy video, unresponsive tracking.

**Solution:**
- Reduce resolution to 320x240
- Switch to KCF or MOSSE tracker
- Disable video recording
- Check CPU temperature: `vcgencmd measure_temp` (should be <70°C)
- Add cooling (heatsink + fan)

### Servo Drifting When Object Centered

**Problem:** Servo continues moving slowly even when object is at center.

**Solution:**
- Set `ki = 0.0` in PID config (disable integral)
- Increase `dead_zone` to 10-15 pixels
- Verify `tolerance_ratio` is adequate (0.08 = 8%)

### Camera Not Detected

**Problem:** `RuntimeError: Failed to create camera`

**Solution:**
```bash
# Check physical connection (ribbon cable)
libcamera-hello --list-cameras

# Enable camera in raspi-config
sudo raspi-config
# Interface Options -> Legacy Camera -> NO (disable)

# Reboot
sudo reboot
```

### ImportError: No module named 'cv2'

**Problem:** OpenCV not found by Python.

**Solution:**
```bash
# Verify installation
python3 -c "import cv2; print(cv2.__version__)"

# Reinstall if needed
cd ~/opencv/build
sudo make install
sudo ldconfig

# Check Python path
python3 -c "import sys; print(sys.path)"
```

---

## 📊 Performance

### Benchmark Results (Raspberry Pi 5, 4GB RAM)

| Resolution | Tracker | FPS | CPU Usage | Accuracy (IoU) |
|------------|---------|-----|-----------|----------------|
| 320×240 | MOSSE | 66 | 45% | 0.75 |
| 320×240 | KCF | 55 | 52% | 0.82 |
| 320×240 | CSRT | 45 | 58% | 0.87 |
| **640×480** | **CSRT** | **26** | **75%** | **0.87** |
| 1280×720 | CSRT | 12 | 92% | 0.89 |

**Recommended:** 640×480 with CSRT for balanced performance.

### Latency Analysis

| Component | Time (ms) | Percentage |
|-----------|-----------|------------|
| Frame capture | 8-10 | 25% |
| Tracker update | 18-22 | 55% |
| PID calculation | 0.5-1 | 2% |
| Servo command | 1-2 | 3% |
| Display & misc | 5-7 | 15% |
| **Total** | **33-42 ms** | **100%** |

**End-to-end latency:** ~40ms (highly responsive for human perception threshold of ~100ms)

---

## 🔬 Advanced Topics

### Adding Deep Learning Detection

Integrate YOLO for automatic object detection:

```python
# Install YOLOv8
pip3 install ultralytics

# Example integration
from ultralytics import YOLO
model = YOLO('yolov8n.pt')  # Nano model for speed

results = model(frame)
if len(results[0].boxes) > 0:
    bbox = results[0].boxes[0].xyxy[0].cpu().numpy()
    tracker.init(frame, bbox)
```

### Web Interface with Flask

Create remote control interface:

```python
from flask import Flask, Response
import cv2

app = Flask(__name__)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')
```

### Multi-Threading Optimization

Separate capture, tracking, and control threads:

```python
import threading

capture_thread = threading.Thread(target=capture_loop)
tracking_thread = threading.Thread(target=tracking_loop)
servo_thread = threading.Thread(target=servo_loop)

capture_thread.start()
tracking_thread.start()
servo_thread.start()
```

---

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create feature branch: `git checkout -b feature/amazing-feature`
3. Commit changes: `git commit -m 'Add amazing feature'`
4. Push to branch: `git push origin feature/amazing-feature`
5. Open Pull Request

### Development Setup

```bash
# Clone your fork
git clone https://github.com/yourusername/object-tracking-servo.git
cd object-tracking-servo

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dev dependencies
pip3 install -r requirements-dev.txt

# Run tests
pytest tests/
```

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **OpenCV Team** - Computer vision library
- **Raspberry Pi Foundation** - Hardware platform and documentation
- **Waveshare** - PCA9685 HAT driver library
- **Community Contributors** - Testing and feedback

### References

1. Lukežič, A., et al. (2017). "Discriminative Correlation Filter Tracker with Channel and Spatial Reliability." *CVPR 2017*.
2. Ziegler, J. G., & Nichols, N. B. (1942). "Optimum Settings for Automatic Controllers." *Trans. ASME*.
3. Bradski, G., & Kaehler, A. (2008). *Learning OpenCV: Computer Vision with the OpenCV Library*. O'Reilly Media.

---

## 📞 Support

- **Documentation:** [Full Technical Report](docs/FINAL.pdf)
- **Issues:** [GitHub Issues](https://github.com/yourusername/object-tracking-servo/issues)
- **Discussions:** [GitHub Discussions](https://github.com/yourusername/object-tracking-servo/discussions)

---

## 🗺️ Roadmap

- [x] Basic object tracking with CSRT
- [x] PID servo control
- [x] Video recording
- [x] Anti-drift protection
- [ ] YOLO integration for auto-detection
- [ ] Web interface with live streaming
- [ ] Kalman filter for prediction
- [ ] Multi-object tracking (SORT algorithm)
- [ ] ROS integration for robotics
- [ ] Mobile app control (Android/iOS)

---

<p align="center">
  Made with ❤️ by <a href="https://github.com/yourusername">Galuh Ningrat</a>
</p>

<p align="center">
  <a href="#-table-of-contents">⬆ Back to Top</a>
</p>

