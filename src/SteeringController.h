#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

/**
 * CONTROL LAYER – Lenkcontroller
 *
 * Input:  Zielwinkel in Grad (-max = voll links, 0 = geradeaus, +max = voll rechts)
 * Output: PWM-Signal an Lenkservo
 *
 * Mapping:
 *   vehicleAngle  -maxAngle → servoPos  (90 + maxAngle)   voll links
 *   vehicleAngle   0        → servoPos  90                geradeaus
 *   vehicleAngle  +maxAngle → servoPos  (90 - maxAngle)   voll rechts
 *
 * Optional: Ratenfilter verhindert zu abrupte Lenkbewegungen.
 */

struct SteeringConfig {
    uint8_t servoPin      = 19;
    int     servoMin_us   = 500;   // Servo-Pulsbreite Minimum (µs)
    int     servoMax_us   = 2500;  // Servo-Pulsbreite Maximum (µs)
    int     servoCenter   = 90;    // Servo-Winkel für Geradeausfahrt
    int     maxAngle_deg  = 30;    // Maximaler Lenkwinkel (mechanisch)

    // Servo-Trim: physischen Geradeaus-Versatz korrigieren.
    // Positiv = Neutral nach links  (korrigiert Rechtsdrall)
    // Negativ = Neutral nach rechts (korrigiert Linksdrall)
    // Faustregel: 1 Einheit ≈ 1° Servowinkel.  Typischer Bereich: −10 … +10.
    float   trimOffset_deg = 0.0f;

    // Ratenfilter: max. Winkeländerung pro loop()-Aufruf (0 = deaktiviert)
    float   maxRatePerCall = 0.0f;

    // Mindestzeit zwischen zwei Servo-Kommandos (ms), 0 = sofort
    uint32_t minInterval_ms = 0;
};

class SteeringController {
public:
    void begin(const SteeringConfig& cfg);
    void setAngle(float vehicleAngle_deg);   // -max..0..+max
    void center();                            // Geradeaus

    float getCurrentAngle() const { return _currentAngle; }

private:
    float applyRateFilter(float target);
    int   angleToServo(float angle);

    SteeringConfig _cfg;
    Servo          _servo;
    float          _currentAngle   = 0.0f;
    unsigned long  _lastUpdate_ms  = 0;
};
