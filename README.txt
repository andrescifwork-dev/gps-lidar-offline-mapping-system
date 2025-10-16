# Offline Mapping System with Obstacle Detection – GPS + LiDAR

This project implements an embedded system capable of rendering maps without an internet connection using OpenStreetMap data and a GPS module. It also integrates a LiDAR distance sensor to detect obstacles in real time and dynamically update the map. If a potential collision is detected, the system triggers an audible alarm.

## 🧠 Key Features

- 📍 Offline map rendering using OpenStreetMap
- 📡 Real-time coordinate tracking via GPS module
- 📏 Obstacle detection using LiDAR sensor
- 🔔 Audible alert when approaching critical proximity
- 🗺️ Dynamic map updates on screen

## 🔧 Technologies and Tools

- Microcontroller: ESP32 / STM32 (depending on version)
- GPS: NEO-6M
- Distance sensor: LiDAR Lite v3
- Libraries: TinyGPS++, Adafruit LiDAR, OSM parser
- Language: Python
- Display: Portable HDMI screen

## 📁 Code Structure

- `src/`: System source code
- `img/`: Prototype photos and screenshots

Have questions or suggestions? I'm open to collaborations!
