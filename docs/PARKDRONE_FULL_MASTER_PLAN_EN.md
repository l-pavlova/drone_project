# PARKDRONE – FULL MASTER PLAN
## Autonomous drone with Pixhawk + Raspberry Pi + Coral AI + Computer Vision

---

# 1. Project goal

Building an autonomous drone capable of:

- Flying a GPS route
- Using preloaded maps
- Avoiding obstacles in real time
- Recognizing objects via AI
- Reacting autonomously
- Returning automatically on failure
- Allowing future upgrades with LiDAR and additional sensors

---

# 2. Hardware architecture

## Flight Controller
Pixhawk

Responsible for:

- stabilization
- GPS positioning
- IMU
- barometer
- failsafe modes

---

## Companion Computer

Raspberry Pi 4 (4GB)

Runs:

- OpenCV
- Python
- AI logic
- MAVLink communication

---

## AI Accelerator

Google Coral USB

Used for:

- YOLO inference
- object detection
- obstacle detection

---

## Camera

IMX219 8MP CSI

Used for:

- video stream
- obstacle recognition
- visual navigation

---

# 3. Power

## Mandatory components

- 5V 5A BEC
- filtered power supply
- cooling

Diagram:

Battery
↓
Power Distribution
↓
BEC 5V
↓
Raspberry Pi + Coral

---

# 4. Software architecture

## Pixhawk

ArduPilot

or

PX4

---

## Raspberry Pi

OS:
- Raspberry Pi OS Lite

Packages:

- Python
- OpenCV
- NumPy
- pymavlink
- MAVSDK
- TensorFlow Lite
- Edge TPU Runtime

---

# 5. Communication

Pi ↔ Pixhawk

USB MAVLink

Baud:

115200

or

921600

---

# 6. Navigation

## Level 1

GPS Navigation

Drone
→ Waypoint 1
→ Waypoint 2
→ Waypoint 3

---

## Level 2

Path Planning

Algorithms:

- A*
- Dijkstra

---

## Level 3

Dynamic Replanning

On obstacle:

1. Detection
2. Generating a new route
3. Sending to Pixhawk

---

# 7. Maps

Source:

OpenStreetMap

Components:

- roads
- buildings
- parks
- water areas

---

## No-Fly Zones

Buildings:

virtual obstacles

Tall objects:

excluded from the route

---

# 8. Computer Vision

OpenCV Pipeline

Camera
↓
Frame
↓
Preprocessing
↓
AI Detection
↓
Obstacle Map
↓
Navigation

---

# 9. AI Detection

Models:

YOLOv5
YOLOv8
MobileNet SSD

Coral optimization:

TensorFlow Lite EdgeTPU

---

# 10. Types of obstacles

- people
- cars
- trees
- poles
- buildings
- animals

---

# 11. Obstacle Avoidance

Detection

↓

Risk calculation

↓

New trajectory

↓

MAVLink command

↓

Pixhawk executes

---

# 12. MAVLink Commands

Basic:

SET_MODE

ARM

TAKEOFF

LAND

RTL

---

Navigation:

SET_POSITION_TARGET_LOCAL_NED

SET_POSITION_TARGET_GLOBAL_INT

Velocity Control

---

# 13. Autonomous flight

Mission:

1. ARM
2. TAKEOFF
3. GPS Navigation
4. Obstacle Avoidance
5. Mission Complete
6. RTL

---

# 14. FailSafe

Loss of GPS

→ Hover
→ RTL

Loss of camera

→ GPS only

Loss of Pi

→ Pixhawk continues the mission

Low battery

→ RTL

---

# 15. SITL Simulation

Before real flight:

ArduPilot SITL

Gazebo

Mission Planner

MAVProxy

---

# 16. Development plan

Phase 1
- New RX
- RC tests

Phase 2
- Pi mounting

Phase 3
- Camera

Phase 4
- MAVLink

Phase 5
- OpenCV

Phase 6
- Coral

Phase 7
- AI Detection

Phase 8
- Obstacle Avoidance

Phase 9
- SITL

Phase 10
- Real autonomous flight

---

# 17. Future improvements

LiDAR

Optical Flow

Stereo Vision

Depth Camera

RTK GPS

Multiple Cameras

Swarm Coordination

SLAM

---

# 18. Expected characteristics

Maximum autonomy:
high

Processing:
real time

AI Detection:
20-100 FPS depending on the model

Reliability:
Pixhawk FailSafe + AI layer

---

# 19. Final architecture

IMX219
↓
Raspberry Pi
↓
Coral USB
↓
AI Detection
↓
Path Planner
↓
MAVLink
↓
Pixhawk
↓
ESC
↓
Motors

---

# 20. End goal

A fully autonomous drone that:

- uses maps
- plans a route
- detects obstacles
- changes its trajectory
- executes a mission
- returns home safely
