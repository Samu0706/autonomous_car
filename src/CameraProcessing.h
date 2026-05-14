#pragma once
#include <Arduino.h>

struct AprilTagResult {
    int   id          = 0;
    float angle_deg   = 0.f;   // positiv = rechts, negativ = links
    float distance_cm = 0.f;
};

enum class CameraState {
    WAITING,    // noch kein <STARTUP> empfangen
    NO_TAG,     // läuft, kein Tag sichtbar
    TAG_FOUND   // läuft, Tag-Daten verfügbar
};
