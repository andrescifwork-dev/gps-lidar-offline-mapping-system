# Sistema de Mapeo Offline con Detección de Obstáculos – GPS + LiDAR

Este proyecto implementa un sistema embebido capaz de renderizar mapas sin conexión a internet utilizando datos de OpenStreetMap y un módulo GPS. Además, integra un sensor de distancia LiDAR para detectar obstáculos en tiempo real y actualizar el mapa dinámicamente. Si se detecta una posible colisión, el sistema activa una alarma sonora.

## 🧠 Características principales

- 📍 Renderizado de mapas offline con OpenStreetMap
- 📡 Lectura de coordenadas en tiempo real vía módulo GPS
- 📏 Detección de obstáculos con sensor LiDAR
- 🔔 Alerta sonora ante proximidad crítica
- 🗺️ Actualización dinámica del mapa en pantalla

## 🔧 Tecnologías y herramientas

- Microcontrolador: ESP32 / STM32 (según versión)
- GPS: NEO-6M
- Sensor de distancia: LiDAR Lite v3
- Librerías: TinyGPS++, Adafruit LiDAR, OSM parser
- Lenguaje: Python
- Visualización: pantalla portátil HDMI

## 📁 Estructura del código

- `src/`: Código fuente del sistema
- `img/`: Imágenes del prototipo y capturas de pantalla


¿Tienes dudas o sugerencias? ¡Estoy abierto a colaboraciones!
