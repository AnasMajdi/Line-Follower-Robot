# Autonomous Line Follower & Maze Solving Robot 🤖🏁

A modular, high-performance firmware developed in C++ for an autonomous robot capable of high-speed line following and maze navigation.

## 🚀 Project Overview
This project features a robust Object-Oriented Programming (OOP) architecture, separating sensor reading, motor driving, and PID control logic. It is built using **PlatformIO** and optimized for embedded systems.

## ⚙️ Key Features
* **Dynamic PID Control:** Adaptive Gain Scheduling for smooth cornering and high-speed straight-line tracking.
* **Maze Navigation:** Implementation of the Left-Hand Rule algorithm for solving complex mazes.
* **Modular Architecture:** Clean C++ classes (`SensorManager`, `PIDController`, `MotorController`) for easy debugging and future scalability.

## 🛠️ Hardware Components
* **Microcontroller:** ESP32 (or specify your board)
* **Sensors:** QTR-8 Reflective Sensor Array
* **Motor Driver:** L298N (or specify your driver)
* **Motors:** High-RPM DC Gear Motors

## 📂 Software Structure
* `main.cpp`: Core loop managing system states and algorithm transitions.
* `SensorManager.cpp/.h`: Handles ADC readings, calibration, and line position logic.
* `PIDController.cpp/.h`: Computes positional error and dynamically calculates motor adjustments.
* `MotorController.cpp/.h`: Manages PWM signals and motor direction.

## 💻 Development Environment
Built and compiled using [PlatformIO](https://platformio.org/) in VS Code.
