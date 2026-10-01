/**
 * @file main.cpp
 * @author Anas Majdi - SpectrumChase
 * @brief Entry point for the Autonomous Procedural Line Follower.
 */

#include <Arduino.h>
#include "MotorController.h"
#include "SensorManager.h"
#include "PIDController.h"

// Define configuration structs instead of objects
MotorConfig myMotors;
SensorConfig mySensorConfig;
PIDConfig myPid;

const int dipRight = 23;

void setup() {
    Serial.begin(115200);
    pinMode(dipRight, INPUT_PULLUP);

    // 1. Hardware Initialization
    initMotors(myMotors);
    initSensors(mySensorConfig);

    // 2. Standby system until start signal is triggered (starts calibration)
    while (digitalRead(dipRight) == HIGH) {
        delay(10);
    }

    // 3. Execute Auto-Calibration
    // Robot will sweep left and right to read the line. Place on track before pressing.
    autoCalibrateSensors(myMotors);
    
    // One-second delay to remove hand and let the robot stabilize
    delay(1000);

    // Reset PID integral to prevent unexpected jumps
    myPid.integral = 0;
    myPid.lastError = 0;

    // 4. Initial kick to overcome static friction
    drive(150, 150, myMotors);
    delay(40);
}

void loop() {
    // Run main control loop continuously
    // Pass configurations and sensor state to the function
    calculatePID(myPid, myMotors, sensorState, mySensorConfig);
}