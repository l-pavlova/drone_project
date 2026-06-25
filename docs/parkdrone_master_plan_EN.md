# Project: Autonomous Drone with CV and AI

## 1. Hardware status
- Drone assembled: motors, ESC, Pixhawk, GPS, safety switch, buzzer
- RC: FlySky FS-i6 (RX temporarily defective, new IA6B on order)
- Camera: IMX219 8MP CSI
- Autonomy plan: Raspberry Pi 4 + Coral USB

## 2. Order list
1. Raspberry Pi 4 (4GB) + MicroSD 64GB + cooling (heatsink + fan)
2. 5V / 5A BEC
3. USB cable Pi ↔ Pixhawk
4. CSI ribbon cable for IMX219
5. Google Coral USB Accelerator
6. Mounting parts: standoffs, zip ties, vibration foam

## 3. Architecture
- Pixhawk: GPS + IMU + Barometer
- Raspberry Pi 4: OpenCV, AI logic, MAVLink
- Coral USB: inference acceleration
- IMX219: video stream for CV

## 4. Autonomy plan
1. Hardware setup
2. Software setup
3. OSM map and waypoints
4. A* / Dijkstra path planning
5. YOLO obstacle detection
6. MAVLink integration
7. SITL and real-world tests
8. Extensions: LiDAR, Optical Flow, Multi-camera

## 5. Navigation
### Global
- GPS route
- Waypoints
- No-fly zones

### Local
- Camera + AI
- Obstacle avoidance
- Real-time corrections

## 6. MAVLink control
- GUIDED mode
- SET_POSITION_TARGET_LOCAL_NED
- Velocity commands
- RTL fail-safe
- LAND fail-safe

## 7. Hardware recommendations
- Raspberry Pi 4 is sufficient
- Coral USB for real-time
- Mandatory 5V/5A BEC
- Active cooling required

## 8. Next steps
- New IA6B receiver
- Raspberry Pi 4
- Coral USB
- Installation and testing
