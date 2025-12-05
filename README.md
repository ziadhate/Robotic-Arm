# FABRI CREATOR - PRO  
### Advanced Robot Arm Controller with GUI & Computer Vision  
Python • PySide6 • OpenCV • Serial Communication • 3D Kinematics

---

## 📌 Overview
This project provides a **full desktop application** to control a 6-DOF robotic arm using:

- **Python GUI (PySide6/QT)**
- **Real-time serial communication (USB / Bluetooth HC-06)**
- **3D visualization using Matplotlib**
- **Forward kinematics**
- **Motion sequencing + smooth interpolation**
- **Computer vision module (OpenCV)** for detecting red objects on a calibrated surface.

The system allows:
- Manual joint control  
- Recording and saving sequences  
- Smooth playback with custom speed  
- Real-time 3D arm visualization  
- CV-based zero-point detection and object following  
- Communication with Arduino/ATmega (angles: `a1,a2,a3,a4,a5,a6\n`)

---

## 📦 Features
### ✔ GUI Features  
- 6 sliders + spinboxes for joints  
- Connect/Disconnect to COM port  
- Live 3D arm plotting  
- Smooth/Non-smooth movement mode  
- Record steps / Save / Load JSON sequences  
- Play / Pause / Stop motion  
- Home and Reset positions  
- Throttled serial sending for stable communication  

### ✔ Computer Vision Features  
- Surface calibration (click 4 points)  
- Zero-position calibration  
- Real-time red-object detection  
- Mapping 2D camera coords → robot surface coords  
- Optional serial communication for robot tracking  

---

# 🛠️ Installation

### **1. Install Dependencies**
```bash
pip install PySide6 pyqtgraph matplotlib numpy opencv-python pyserial
