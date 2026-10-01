/**
 * @file MotorController.cpp
 * @author Anas Majdi - SpectrumChase
 * @brief Implementation of hardware PWM control (Procedural).
 */

#include "MotorController.h"

void initMotors(const MotorConfig &motors) {
    pinMode(motors.rIn1, OUTPUT); 
    pinMode(motors.rIn2, OUTPUT);
    pinMode(motors.lIn1, OUTPUT); 
    pinMode(motors.lIn2, OUTPUT);

    ledcSetup(motors.rCh, 5000, 8); 
    ledcSetup(motors.lCh, 5000, 8);
    ledcAttachPin(motors.rPwm, motors.rCh); 
    ledcAttachPin(motors.lPwm, motors.lCh);
}

void drive(int leftSpeed, int rightSpeed, const MotorConfig &motors) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Right motor control
    digitalWrite(motors.rIn1, rightSpeed >= 0 ? LOW : HIGH);
    digitalWrite(motors.rIn2, rightSpeed >= 0 ? HIGH : LOW);
    ledcWrite(motors.rCh, abs(rightSpeed));

    // Left motor control
    digitalWrite(motors.lIn1, leftSpeed >= 0 ? LOW : HIGH);
    digitalWrite(motors.lIn2, leftSpeed >= 0 ? HIGH : LOW);
    ledcWrite(motors.lCh, abs(leftSpeed));
}

void stopMotors(const MotorConfig &motors) {
    drive(0, 0, motors);
}