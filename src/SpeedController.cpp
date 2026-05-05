/* --- Update 14.04.2026 ---
Code ist getestet und funktioniert. Vorwärts/Rückwärts fahren funktioniert, auch in Kombination mit der Lenkung.
*/
#include "SpeedController.h"

void SpeedController::begin(const SpeedConfig& cfg) {
    _cfg = cfg;
    pinMode(cfg.pwmPin, OUTPUT);
    pinMode(cfg.dirPin, OUTPUT);
    stop();
}

void SpeedController::forward() {
    setMotor(_cfg.dirForward, _cfg.speedForward);
    _dir = DriveDirection::FORWARD;
}

void SpeedController::reverse() {
    setMotor(_cfg.dirReverse, _cfg.speedReverse);
    _dir = DriveDirection::REVERSE;
}

void SpeedController::stop() {
    analogWrite(_cfg.pwmPin, 0);
    _dir = DriveDirection::STOPPED;
}

void SpeedController::setMotor(uint8_t dir, int pwm) {
    digitalWrite(_cfg.dirPin, dir);
    analogWrite(_cfg.pwmPin, pwm);
}

void SpeedController::setForwardSpeed(int pwm) {
    _cfg.speedForward = constrain(pwm, 0, 255);
}