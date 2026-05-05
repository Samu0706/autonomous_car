#include "ObstacleDetection.h"

// =============================
// BEGIN
// =============================
void ObstacleDetection::begin(const ObstacleConfig& cfg, VehicleStateMachine* vsm) {
    _cfg = cfg;
    _vsm = vsm;
}

// =============================
// PROCESS  –  einmal pro Scan
// =============================
void ObstacleDetection::process(const LidarPoint* points, int count) {

    // Zähler pro Sektor
    int cFrontMiddle     = 0;
    int cFrontSideLeft   = 0;
    int cFrontSideRight  = 0;

    int cRearMiddle      = 0;
    int cRearSideLeft    = 0;
    int cRearSideRight   = 0;

    // --- Punkte auswerten ---
    for (int i = 0; i < count; i++) {

        float a = points[i].angle;
        float d = points[i].distance;

        if (d <= 0) continue;   // ungültige Punkte überspringen

        // ==================
        // VORNE – Mitte  (160°–200°)
        // ==================
        if (inRange(a, _cfg.frontMiddleLeft, _cfg.frontMiddleRight)) {
            if (d < _cfg.limitMiddle) {
                cFrontMiddle++;
            }
        }

        // ==================
        // VORNE – Seite Links  (110°–170°)
        // ==================
        if (inRange(a, _cfg.frontSideLeftLeft, _cfg.frontSideLeftRight)) {
            if (d < _cfg.limitSide) {
                cFrontSideLeft++;
            }
        }

        // ==================
        // VORNE – Seite Rechts  (190°–250°)
        // ==================
        if (inRange(a, _cfg.frontSideRightLeft, _cfg.frontSideRightRight)) {
            if (d < _cfg.limitSide) {
                cFrontSideRight++;
            }
        }

        // ==================
        // HINTEN – Mitte  (340°–20°, wrap-around)
        // ==================
        if (inRange(a, _cfg.rearMiddleLeft, _cfg.rearMiddleRight)) {
            if (d < _cfg.limitReverseMiddle) {
                cRearMiddle++;
            }
        }

        // ==================
        // HINTEN – Seite Links  (290°–340°)
        // ==================
        if (inRange(a, _cfg.rearSideLeftLeft, _cfg.rearSideLeftRight)) {
            if (d < _cfg.limitReverseSide) {
                cRearSideLeft++;
            }
        }

        // ==================
        // HINTEN – Seite Rechts  (20°–70°)
        // ==================
        if (inRange(a, _cfg.rearSideRightLeft, _cfg.rearSideRightRight)) {
            if (d < _cfg.limitReverseSide) {
                cRearSideRight++;
            }
        }
    }

    // --- Flags setzen ---
    _obstacleAhead = (cFrontMiddle    > _cfg.counterLimit ||
                      cFrontSideLeft  > _cfg.counterLimit ||
                      cFrontSideRight > _cfg.counterLimit);

    _obstacleRear  = (cRearMiddle     > _cfg.counterLimit ||
                      cRearSideLeft   > _cfg.counterLimit ||
                      cRearSideRight  > _cfg.counterLimit);

    // --- Debug ---
    if (_obstacleAhead) {
        Serial.print("[OBS] VORNE –");
        if (cFrontMiddle    > _cfg.counterLimit) Serial.print(" Mitte");
        if (cFrontSideLeft  > _cfg.counterLimit) Serial.print(" Links");
        if (cFrontSideRight > _cfg.counterLimit) Serial.print(" Rechts");
        Serial.println();
    }
    if (_obstacleRear) {
        Serial.print("[OBS] HINTEN –");
        if (cRearMiddle     > _cfg.counterLimit) Serial.print(" Mitte");
        if (cRearSideLeft   > _cfg.counterLimit) Serial.print(" Links");
        if (cRearSideRight  > _cfg.counterLimit) Serial.print(" Rechts");
        Serial.println();
    }

    // --- VSM informieren ---
    // VSM schaltet Motor selbst: DRIVING→REVERSING→EMERGENCY
    _vsm->update(_obstacleAhead, _obstacleRear);
}

// =============================
// IN RANGE  –  mit wrap-around
// Normalfall:   left <= right  →  z.B. 160°–200°
// Wrap-around:  left >  right  →  z.B. 340°–20°
// =============================
bool ObstacleDetection::inRange(float angle, float left, float right) const {
    if (left <= right) {
        return angle >= left && angle <= right;
    } else {
        return angle >= left || angle <= right;
    }
}