/* --- Update 14.04.2026 ---
Code ist getestet und funktioniert. Räder lenken tatsächlich.
int angle = +30 -> rechts
int angle = -30 -> links
int angle =   0 -> geradeaus
*/
#include "SteeringController.h"

void SteeringController::begin(const SteeringConfig& cfg) {
    _cfg = cfg;
    _servo.setPeriodHertz(50);
    _servo.attach(cfg.servoPin, cfg.servoMin_us, cfg.servoMax_us);
    center();
}

void SteeringController::setAngle(float vehicleAngle_deg) {
    // Ratenfilter anwenden
    float target = applyRateFilter(vehicleAngle_deg);

    // Zeitintervall prüfen
    unsigned long now = millis();
    if (_cfg.minInterval_ms > 0 && (now - _lastUpdate_ms) < _cfg.minInterval_ms)
        return;
    _lastUpdate_ms = now;

    // Begrenzen
    target = constrain(target, -(float)_cfg.maxAngle_deg, (float)_cfg.maxAngle_deg);
    _currentAngle = target;

    int servoPos = angleToServo(target);
    _servo.write(servoPos);
}

void SteeringController::center() {
    _currentAngle = 0.0f;
    _servo.write(_cfg.servoCenter);
}

// ---- Private ----

float SteeringController::applyRateFilter(float target) {
    if (_cfg.maxRatePerCall <= 0.0f) return target;  // deaktiviert

    float delta = target - _currentAngle;
    if (delta >  _cfg.maxRatePerCall) delta =  _cfg.maxRatePerCall;
    if (delta < -_cfg.maxRatePerCall) delta = -_cfg.maxRatePerCall;
    return _currentAngle + delta;
}

int SteeringController::angleToServo(float angle) {
    // +angle = rechts → Servo sinkt unter Mitte; -angle = links → Servo steigt
    return (int)(_cfg.servoCenter - angle);
}
