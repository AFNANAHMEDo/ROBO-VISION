ROBO VISION
AI-Powered Pan-Tilt Robotic Vision System

Institute Technical Summer Project (ITSP) — IIT Bombay

ROBO VISION is an AI + robotics system that detects a target in real time and automatically orients a pan-tilt mechanism towards it, forming a complete perception → control → actuation closed loop.

# Key Features
Live video streaming using ESP32-CAM over Wi-Fi
Custom-trained YOLOv8 for real-time object detection
Bounding-box center calculation for target localization
Conversion of target position into pan/tilt error
Adaptive proportional control for servo movement
Motion smoothing and anti-oscillation
ESP32-to-ESP32 USB Serial communication
Automatic pan-tilt target tracking
⚙️ System Pipeline
ESP32-CAM → Wi-Fi Video → YOLOv8 Detection
                         ↓
                  Target Center
                         ↓
                  Pan/Tilt Error
                         ↓
              Adaptive P Controller
                         ↓
                  USB Serial
                         ↓
              ESP32 Servo Control
                         ↓
                   Pan-Tilt
                         ↓
                     Target
# Tech Stack
Python, C++
OpenCV
YOLOv8 / Ultralytics
ESP32 & ESP32-CAM
Arduino IDE
USB Serial Communication
Servo Motors
Adaptive Proportional Control
📁Repository Structure
ROBO-VISION/
├── computer_vision/      # YOLO + tracking
├── esp32_cam/            # ESP32-CAM firmware
├── esp32_pan_tilt/       # Servo controller firmware
├── serial_communication/ # Serial utilities
├── hardware/             # Hardware documentation
├── assets/               # Images / demos
├── best.pt               # Trained YOLO model
├── requirements.txt
└── README.md



#Outcome

Successfully developed and demonstrated a working robotic vision prototype capable of detecting and automatically tracking a target using a pan-tilt mechanism.

The project involved hands-on work across computer vision, machine learning, embedded systems, robotics, control systems, and hardware-software integration.

# Team

Afnan Ahmed · Samarth Vishal Charhate · Faizan Shakeel · MS Fahad
Mentor: Khushi Agarwal
IIT Bombay — Institute Technical Summer Project (ITSP)

See. Decide. Move. 
