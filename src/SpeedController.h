#pragma once
#include <Arduino.h>

/**
 * CONTROL LAYER – Geschwindigkeitscontroller
 *
 * Steuert Motor via PWM + Richtungspin (H-Brücke / ESC mit Dir-Pin).
 *
 * Fahrtrichtungen: FORWARD, REVERSE, STOP
 */

enum class DriveDirection { FORWARD, REVERSE, STOPPED };

struct SpeedConfig {
    uint8_t pwmPin      = 21;
    uint8_t dirPin      = 35;
    uint8_t dirForward  = LOW;   // LOW = Vorwärts am H-Brücken-Board
    uint8_t dirReverse  = HIGH;

    int     speedForward  = 22;  // PWM-Wert 0..255
    int     speedReverse  = 20;
};

class SpeedController {
public:
    void begin(const SpeedConfig& cfg);

    void forward();
    void reverse();
    void stop();
    void setForwardSpeed(int pwm);

    DriveDirection getDirection()    const { return _dir; }
    bool           isMoving()        const { return _dir != DriveDirection::STOPPED; }
    int            getForwardSpeed() const { return _cfg.speedForward; }

private:
    void setMotor(uint8_t dir, int pwm);

    SpeedConfig    _cfg;
    DriveDirection _dir = DriveDirection::STOPPED;
};
