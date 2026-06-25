# Проект: Автономен дрон с CV и AI

## 1. Състояние на хардуера
- Дрон сглобен: мотори, ESC, Pixhawk, GPS, safety switch, buzzer
- RC: FlySky FS i6 (RX временно дефектен, поръчка на нов IA6B)
- Камера: IMX219 8MP CSI
- План за автономност: Raspberry Pi 4 + Coral USB

## 2. Списък за поръчка
1. Raspberry Pi 4 (4GB) + MicroSD 64GB + cooling (heatsink + fan)
2. 5V / 5A BEC
3. USB кабел Pi ↔ Pixhawk
4. CSI ribbon кабел за IMX219
5. Google Coral USB Accelerator
6. Монтажни елементи: стойки, zip ties, vibration foam

## 3. Архитектура
- Pixhawk: GPS + IMU + Barometer
- Raspberry Pi 4: OpenCV, AI логика, MAVLink
- Coral USB: ускорение на inference
- IMX219: видео поток за CV

## 4. План за автономност
1. Hardware setup
2. Software setup
3. OSM карта и waypoints
4. A* / Dijkstra path planning
5. YOLO obstacle detection
6. MAVLink интеграция
7. SITL и реални тестове
8. Разширения: LiDAR, Optical Flow, Multi-camera

## 5. Навигация
### Глобално
- GPS маршрут
- Waypoints
- No-fly зони

### Локално
- Камера + AI
- Избягване на препятствия
- Корекции в реално време

## 6. MAVLink управление
- GUIDED mode
- SET_POSITION_TARGET_LOCAL_NED
- Velocity commands
- RTL fail-safe
- LAND fail-safe

## 7. Хардуерни препоръки
- Raspberry Pi 4 е достатъчен
- Coral USB за реално време
- Задължителен 5V/5A BEC
- Активно охлаждане

## 8. Следващи стъпки
- Нов IA6B приемник
- Raspberry Pi 4
- Coral USB
- Инсталация и тестове
