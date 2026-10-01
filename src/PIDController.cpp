/**
 * @file PIDController.cpp
 * @author Anas Majdi - SpectrumChase
 * @brief Implementation of the procedural PID control loop.
 */

#include "PIDController.h"

void calculatePID(PIDConfig &pid, const MotorConfig &motors, SensorState &sensorState, const SensorConfig &sensorCfg) {
    if (pid.systemHalted) {
        stopMotors(motors);
        return;
    }

    // Read line position 
    uint16_t pos = getLinePosition(sensorCfg, sensorState);
    
    // ==========================================
    // 1. Hardware Watchdog & Active Reverse Recovery
    // ==========================================
    if (sensorState.currentState != ON_LINE) {
        if (pid.timeLost == 0) pid.timeLost = millis();
        
        // If lost for more than the timeout, shut down motors
        if (millis() - pid.timeLost > pid.failsafeTimeout) {
            pid.systemHalted = true; 
            Serial.println("FAILSAFE TRIGGERED");
            return;
        }

        // Smart active reverse instead of stalling
        if (sensorState.currentState == LOST_LEFT) {
            drive(pid.reverseSpeed, pid.pivotSpeed, motors); // Reverse and pivot left
        } else if (sensorState.currentState == LOST_RIGHT) {
            drive(pid.pivotSpeed, pid.reverseSpeed, motors); // Reverse and pivot right
        }
        return; // Halt loop execution until line is found
    } else {
        pid.timeLost = 0; 
    }

    // ==========================================
    // 2. Maze Solving Heuristics and Corner Detection
    // ==========================================
    bool canGoRight = isCorner(true, sensorState);
    bool canGoLeft = isCorner(false, sensorState);

    if (canGoRight || canGoLeft) {
        if (pid.activeMazeAlgo == RIGHT_HAND_RULE) {
            // Priority: Right -> Left
            if (canGoRight) {
                drive(pid.pivotSpeed, -pid.pivotSpeed, motors); // Pivot right
                delay(200); return;
            } else if (canGoLeft) {
                drive(-pid.pivotSpeed, pid.pivotSpeed, motors); // Pivot left
                delay(200); return;
            }
        } 
        else if (pid.activeMazeAlgo == LEFT_HAND_RULE) {
            // Priority: Left -> Right
            if (canGoLeft) {
                drive(-pid.pivotSpeed, pid.pivotSpeed, motors);
                delay(200); return;
            } else if (canGoRight) {
                drive(pid.pivotSpeed, -pid.pivotSpeed, motors);
                delay(200); return;
            }
        } 
        else {
            // Normal mode (no maze - standard sharp turn)
            if (canGoRight) { drive(pid.pivotSpeed, 0, motors); delay(30); return; }
            if (canGoLeft)  { drive(0, pid.pivotSpeed, motors); delay(30); return; }
        }
    }

    // ==========================================
    // 3. Dynamic Gain Scheduling
    // ==========================================
    int error = pos - 3500;
    PIDProfile currentProfile;
    
    // Use smooth curve profile for large errors (sharp turns)
    if (abs(error) > 1500) {
        currentProfile = pid.curve;
    } else {
        // Use top speed profile for straight lines
        currentProfile = pid.straight;
    }

    pid.integral += error;
    int derivative = error - pid.lastError;
    
    // Calculation based on selected profile
    int correction = (currentProfile.kp * error) + (currentProfile.ki * pid.integral) + (currentProfile.kd * derivative);
    pid.lastError = error;

    drive(pid.baseSpeed + correction, pid.baseSpeed - correction, motors);
}