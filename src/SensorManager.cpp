/**
 * @file SensorManager.cpp
 * @author Anas Majdi - SpectrumChase
 * @brief Implementation of Sensor Manager (Procedural).
 */

#include "SensorManager.h"

QTRSensors qtr;
SensorState sensorState;

void initSensors(const SensorConfig &config) {
    qtr.setTypeRC();
    qtr.setSensorPins(config.pins, config.numSensors);
}

// Auto-calibration function (robot sweeps left and right to read the line)
void autoCalibrateSensors(const MotorConfig &motors) {
    int calibSpeed = 100; // Calibration rotation speed
    
    // Sweep right
    drive(calibSpeed, -calibSpeed, motors);
    for (int i = 0; i < 100; i++) {
        qtr.calibrate();
        delay(10);
    }
    
    // Sweep left (longer duration to cover both sides)
    drive(-calibSpeed, calibSpeed, motors);
    for (int i = 0; i < 200; i++) {
        qtr.calibrate();
        delay(10);
    }
    
    // Return to center
    drive(calibSpeed, -calibSpeed, motors);
    for (int i = 0; i < 100; i++) {
        qtr.calibrate();
        delay(10);
    }
    
    stopMotors(motors); // Stop motors after calibration is complete
}

uint16_t getLinePosition(const SensorConfig &config, SensorState &state) {
    // Critical modification: using readCalibrated instead of read to apply calibration data
    qtr.readCalibrated(state.rawValues);
    
    long weightedSum = 0;
    long sum = 0;

    for (uint8_t i = 0; i < config.numSensors; i++) {
        long val = state.rawValues[i];

        // Color inversion feature
        if (config.currentMode == WHITE_ON_BLACK) {
            val = 1000 - val; // Calibrated values range from 0 to 1000
            if (val < 0) val = 0;
        }

        // Apply Exponential Moving Average (EMA) digital filter
        state.filteredValues[i] = (config.emaAlpha * val) + ((1.0 - config.emaAlpha) * state.filteredValues[i]);

        if (state.filteredValues[i] > config.noiseThreshold) {
            long cleanValue = state.filteredValues[i] - config.noiseThreshold;
            weightedSum += cleanValue * (i * 1000);
            sum += cleanValue;
        }
    }

    if (sum == 0) {
        state.currentState = (state.lastPosition < 3500) ? LOST_LEFT : LOST_RIGHT;
        return (state.lastPosition < 3500) ? 0 : 7000;
    }

    state.currentState = ON_LINE;
    state.lastPosition = weightedSum / sum;
    return state.lastPosition;
}

bool isCorner(bool checkRight, const SensorState &state) {
    if (checkRight) return (state.filteredValues[7] > 800 && state.filteredValues[0] < 500);
    else return (state.filteredValues[0] > 800 && state.filteredValues[7] < 500);
}