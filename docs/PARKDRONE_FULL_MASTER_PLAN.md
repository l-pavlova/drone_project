# PARKDRONE – ПЪЛЕН MASTER PLAN
## Автономен дрон с Pixhawk + Raspberry Pi + Coral AI + Computer Vision

---

# 1. Цел на проекта

Изграждане на автономен дрон способен да:

- Лети по GPS маршрут
- Използва предварително заредени карти
- Избягва препятствия в реално време
- Разпознава обекти чрез AI
- Реагира автономно
- Връща се автоматично при проблем
- Позволява бъдещо надграждане с LiDAR и допълнителни сензори

---

# 2. Хардуерна архитектура

## Flight Controller
Pixhawk

Отговаря за:

- стабилизация
- GPS позициониране
- IMU
- барометър
- failsafe режими

---

## Companion Computer

Raspberry Pi 4 (4GB)

Изпълнява:

- OpenCV
- Python
- AI логика
- MAVLink комуникация

---

## AI Accelerator

Google Coral USB

Използва се за:

- YOLO inference
- object detection
- obstacle detection

---

## Camera

IMX219 8MP CSI

Използва се за:

- видео поток
- разпознаване на препятствия
- визуална навигация

---

# 3. Захранване

## Задължителни компоненти

- 5V 5A BEC
- филтрирано захранване
- охлаждане

Схема:

Battery
↓
Power Distribution
↓
BEC 5V
↓
Raspberry Pi + Coral

---

# 4. Софтуерна архитектура

## Pixhawk

ArduPilot

или

PX4

---

## Raspberry Pi

OS:
- Raspberry Pi OS Lite

Пакети:

- Python
- OpenCV
- NumPy
- pymavlink
- MAVSDK
- TensorFlow Lite
- Edge TPU Runtime

---

# 5. Комуникация

Pi ↔ Pixhawk

USB MAVLink

Baud:

115200

или

921600

---

# 6. Навигация

## Ниво 1

GPS Navigation

Drone
→ Waypoint 1
→ Waypoint 2
→ Waypoint 3

---

## Ниво 2

Path Planning

Алгоритми:

- A*
- Dijkstra

---

## Ниво 3

Dynamic Replanning

При препятствие:

1. Засичане
2. Генериране на нов маршрут
3. Изпращане към Pixhawk

---

# 7. Карти

Източник:

OpenStreetMap

Компоненти:

- пътища
- сгради
- паркове
- водни площи

---

## No-Fly Zones

Сгради:

виртуални препятствия

Високи обекти:

изключване от маршрута

---

# 8. Computer Vision

OpenCV Pipeline

Камера
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

Модели:

YOLOv5
YOLOv8
MobileNet SSD

Coral оптимизация:

TensorFlow Lite EdgeTPU

---

# 10. Видове препятствия

- хора
- автомобили
- дървета
- стълбове
- сгради
- животни

---

# 11. Obstacle Avoidance

Засичане

↓

Изчисляване на риск

↓

Нова траектория

↓

MAVLink команда

↓

Pixhawk изпълнява

---

# 12. MAVLink Команди

Основни:

SET_MODE

ARM

TAKEOFF

LAND

RTL

---

Навигация:

SET_POSITION_TARGET_LOCAL_NED

SET_POSITION_TARGET_GLOBAL_INT

Velocity Control

---

# 13. Автономен полет

Мисия:

1. ARM
2. TAKEOFF
3. GPS Navigation
4. Obstacle Avoidance
5. Mission Complete
6. RTL

---

# 14. FailSafe

Загуба на GPS

→ Hover
→ RTL

Загуба на камера

→ GPS only

Загуба на Pi

→ Pixhawk продължава мисията

Ниска батерия

→ RTL

---

# 15. SITL Симулация

Преди реален полет:

ArduPilot SITL

Gazebo

Mission Planner

MAVProxy

---

# 16. План за разработка

Фаза 1
- Нов RX
- RC тестове

Фаза 2
- Pi монтаж

Фаза 3
- Камера

Фаза 4
- MAVLink

Фаза 5
- OpenCV

Фаза 6
- Coral

Фаза 7
- AI Detection

Фаза 8
- Obstacle Avoidance

Фаза 9
- SITL

Фаза 10
- Реален автономен полет

---

# 17. Бъдещи подобрения

LiDAR

Optical Flow

Stereo Vision

Depth Camera

RTK GPS

Multiple Cameras

Swarm Coordination

SLAM

---

# 18. Очаквани характеристики

Максимална автономност:
висока

Обработка:
реално време

AI Detection:
20-100 FPS според модела

Надеждност:
Pixhawk FailSafe + AI слой

---

# 19. Финална архитектура

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

# 20. Крайна цел

Напълно автономен дрон който:

- използва карти
- планира маршрут
- открива препятствия
- променя траекторията
- изпълнява мисия
- прибира се безопасно
