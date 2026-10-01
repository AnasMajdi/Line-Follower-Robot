/**
 * @file PIDController.h
 * @author Anas Majdi - SpectrumChase
 * @brief Procedural PID Controller with Gain Scheduling, Reverse Recovery, & Maze Logic.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "MotorController.h"
#include "SensorManager.h"

// Struct for PID profiles
struct PIDProfile {
    float kp; 
    float ki; 
    float kd;
};

// Maze algorithms
enum MazeAlgorithm { NONE, RIGHT_HAND_RULE, LEFT_HAND_RULE };

// Struct holding all PID states and variables
struct PIDConfig {
    // 1. Dynamic Gain Profiles
    PIDProfile straight = {0.15, 0.0001, 8.0}; 
    PIDProfile curve = {0.10, 0.0002, 5.0};    
    
    int lastError = 0;
    long integral = 0;
    
    int baseSpeed = 155;
    int pivotSpeed = 90;
    int reverseSpeed = -80; // Reverse speed when the line is lost

    unsigned long timeLost = 0;
    const unsigned long failsafeTimeout = 2000; 
    bool systemHalted = false;

    // Enable right-hand rule algorithm as default
    MazeAlgorithm activeMazeAlgo = RIGHT_HAND_RULE; 
};

// Main function replacing tick()
void calculatePID(PIDConfig &pid, const MotorConfig &motors, SensorState &sensorState, const SensorConfig &sensorCfg);

#endif