/**
 * @file SensorManager.h
 * @author Anas Majdi - SpectrumChase
 * @brief Procedural Sensor Manager with EMA Filter & Auto-Calibration.
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <QTRSensors.h>
#include "MotorController.h"

enum LineState { ON_LINE, LOST_RIGHT, LOST_LEFT };
enum ColorMode { BLACK_ON_WHITE, WHITE_ON_BLACK };

// Sensor configuration constants
struct SensorConfig {
    static const uint8_t numSensors = 8;
    uint8_t pins[8] = {13, 12, 14, 27, 26, 25, 33, 32};
    float emaAlpha = 0.7;    
    int noiseThreshold = 70;
    ColorMode currentMode = BLACK_ON_WHITE; // Added to toggle between track colors
};

// Dynamic sensor state variables
struct SensorState {
    LineState currentState = ON_LINE;
    uint16_t rawValues[8];
    float filteredValues[8] = {0}; 
    uint16_t lastPosition = 3500;
};

// Global instances for easy access
extern QTRSensors qtr;
extern SensorState sensorState;

// Core functions
void initSensors(const SensorConfig &config);
void autoCalibrateSensors(const MotorConfig &motors);
uint16_t getLinePosition(const SensorConfig &config, SensorState &state);
bool isCorner(bool checkRight, const SensorState &state);

#endif