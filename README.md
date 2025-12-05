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

###flowchart TD

A[Start Program] --> B[Initialize GUI + Serial Manager]
B --> C[User Selects COM Port]
C -->|Connect| D[Open Serial Port]
C -->|Skip| E[Simulation Mode]

D --> F[Update Angles from Sliders/Spinboxes]
E --> F

F --> G[Update 3D Kinematic Plot]
G --> H{User Records Steps?}
H -->|Yes| I[Save Step to Sequence]
H -->|No| G

I --> J{User Plays Sequence?}
J -->|Yes| K[Interpolation + Send Angles Smoothly]
J -->|No| G

K --> L{Paused?}
L -->|Yes| L
L -->|No| K

K --> M{Stopped?}
M -->|Yes| G
M -->|No| K

%% COMPUTER VISION BRANCH
A --> CV1[Start CV Script]
CV1 --> CV2[Camera Calibrate Surface (4 Clicks)]
CV2 --> CV3[Click Zero Position]
CV3 --> CV4[Detect Red Object]
CV4 --> CV5[Map Object to Robot Coordinates]
CV5 --> CV6[Send Coordinates/Angles to Robot]
CV6 --> CV4





# 🛠️ Installation

### **1. Install Dependencies**
```bash
pip install PySide6 pyqtgraph matplotlib numpy opencv-python pyserial




