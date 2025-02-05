# Autonomous Hovercraft

A fully autonomous hovercraft project powered by an Arduino Nano and various sensors, designed to navigate its environment by using distance measurements and motion control systems. This project combines IMU data, ultrasonic distance measurements, and motor control to create a self-navigating hovercraft.

---

## Project Overview
This project aims to build an autonomous hovercraft capable of maneuvering using sensor-based input and motor control. The hovercraft utilizes an IMU for orientation tracking, ultrasonic sensors for distance measurement, and a set of fans and servos for propulsion and directional control.

---

## In Action

https://github.com/user-attachments/assets/5a87e032-c3d5-4d5c-9fd3-dc1846e5d196

---

## Features
- Autonomous navigation using real-time obstacle detection.
- Orientation control using the MPU6050 IMU.
- Ultrasonic distance sensing for obstacle avoidance.
- PWM-based motor control for precise movements.
- Customizable PID control for steering adjustments.

---


## Code Overview

The Arduino sketch includes the following key components:

- **MPU6050 Initialization:** For orientation and stabilization using yaw, pitch, and roll calculations.
- **HC-SR04 Distance Sensors:** For obstacle detection and distance measurement.
- **PWM Control:** For adjusting servo positions and fan speeds.
- **Interrupt-Driven Distance Measurement:** Accurate real-time measurement without blocking the main control loop.

### Key Arduino Pins Used:
| Component       | Pin Assignment |
|-----------------|----------------|
| Lift Fan        | PD4 (P18)      |
| Propulsion Fan  | PB4 (P4)       |
| Servo Motor     | PB1 (P9)       |
| Ultrasonic Sensor 1 Trig | PB5   |
| Ultrasonic Sensor 1 Echo | PD3   |
| Ultrasonic Sensor 2 Trig | PB3   |
| Ultrasonic Sensor 2 Echo | PD2   |

---
