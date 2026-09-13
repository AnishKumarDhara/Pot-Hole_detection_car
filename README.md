<div align="center">

# 🚗 POTHOLE DETECTION RC CAR

### IoT-Based Experimental Road Surface Monitoring Platform

<img src="https://commons.wikimedia.org/wiki/Special:Redirect/file/Car_for_project.jpg" width="750">

<br>

**Detect • Analyze • Visualize • Experiment**

<br>

[![ESP32](https://img.shields.io/badge/ESP32-IoT%20Controller-000000?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)
[![C++](https://img.shields.io/badge/C%2B%2B-Embedded-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Arduino](https://img.shields.io/badge/Arduino-IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Blynk](https://img.shields.io/badge/Blynk-IoT-23C48E?style=for-the-badge)](https://blynk.io/)
[![GitHub](https://img.shields.io/badge/GitHub-Repository-181717?style=for-the-badge&logo=github)](https://github.com/IshanSamanta/Pot-Hole_detection)

<br>

![Status](https://img.shields.io/badge/STATUS-EXPERIMENTAL%20PROTOTYPE-orange?style=flat-square)
![Platform](https://img.shields.io/badge/PLATFORM-ESP32-blue?style=flat-square)
![Sensor](https://img.shields.io/badge/SENSOR-MPU6050-purple?style=flat-square)
![IoT](https://img.shields.io/badge/IoT-BLYNK-23C48E?style=flat-square)

</div>

---

## ⚠️ IMPORTANT — PROTOTYPE DISCLAIMER

> ### 🚧 THIS PROJECT IS AN EXPERIMENTAL PROTOTYPE
>
> This project has been developed for **educational, research, robotics, IoT, and concept-demonstration purposes only**.
>
> **It is NOT intended for commercial use, professional road inspection, autonomous driving, public-road deployment, or safety-critical applications.**
>
> The current pothole-detection method is experimental and has **not been professionally calibrated, certified, or validated for real-world road-safety applications.**
>
> **Do not use this system on public roads or in traffic.**
>
> Detection results may be affected by vehicle speed, chassis vibration, sensor orientation, sensor mounting, road conditions, motor vibration, and other environmental factors.
>
> **This repository represents a learning/research prototype, not a production-ready road-monitoring system.**

---

# 🌐 Overview

The **Pothole Detection RC Car** is an experimental robotic vehicle designed to investigate whether an inexpensive RC-car platform can be used to identify significant road-surface irregularities using an **MPU6050 inertial measurement unit**.

The vehicle uses an **ESP32** as its primary controller.

The MPU6050 continuously measures acceleration and gyroscopic motion. The ESP32 processes the acceleration data and derives a **Dynamic Z** value representing changes in the vertical acceleration component.

The processed data can then be transmitted through Wi-Fi to the **Blynk IoT platform**, allowing the sensor response to be observed remotely through a dashboard and chart.

---

# 🧠 System Architecture

```text
                         ROAD SURFACE
                              │
                              ▼
                     ┌─────────────────┐
                     │     MPU6050     │
                     │ Accelerometer + │
                     │    Gyroscope    │
                     └────────┬────────┘
                              │ I²C
                              ▼
                     ┌─────────────────┐
                     │      ESP32      │
                     │                 │
                     │ Sensor Reading  │
                     │ Signal Filter   │
                     │ Dynamic-Z       │
                     │ Detection Logic │
                     └───────┬─────────┘
                             │
                  ┌──────────┴──────────┐
                  │                     │
                  ▼                     ▼
          ┌──────────────┐       ┌──────────────┐
          │    L298N     │       │     Wi-Fi    │
          │ Motor Driver │       │ Connectivity │
          └──────┬───────┘       └──────┬───────┘
                 │                      │
                 ▼                      ▼
          ┌──────────────┐       ┌──────────────┐
          │ 4 × DC Motors│       │ Blynk Cloud  │
          └──────────────┘       └──────┬───────┘
                                        │
                                        ▼
                                 ┌──────────────┐
                                 │  Dashboard   │
                                 │  + Graph     │
                                 └──────────────┘
