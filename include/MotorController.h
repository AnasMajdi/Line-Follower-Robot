/**
 * @file MotorController.h
 * @author Anas Majdi - SpectrumChase
 * @brief Procedural Motor Controller using Structs.
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

// Grouping motor pins in a single struct for organization
struct MotorConfig {
    uint8_t rIn1 = 19;
    uint8_t rIn2 = 18;
    uint8_t rPwm = 15;
    uint8_t rCh = 0;

    uint8_t lIn1 = 5;
    uint8_t lIn2 = 4;
    uint8_t lPwm = 2;
    uint8_t lCh = 1;
};

// Procedural function declarations
void initMotors(const MotorConfig &motors);
void drive(int leftSpeed, int rightSpeed, const MotorConfig &motors);
void stopMotors(const MotorConfig &motors);

#endif