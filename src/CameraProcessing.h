#pragma once
#include <Arduino.h>

// Normalisiertes Ergebnis eines HuskyLens-Frames.
// Unabhängig vom Algorithmus – nicht genutzte Felder bleiben 0.
struct CameraResult {
    uint16_t id      = 0;
    int16_t  xCenter = 0;   // Bildmitte X (px)
    int16_t  yCenter = 0;   // Bildmitte Y (px)
    int16_t  width   = 0;
    int16_t  height  = 0;
    float    pitch   = 0.f; // nur bei Gaze/Face
    float    yaw     = 0.f;
    char     name[32]{};
};

// Hilfsfunktion: gibt an ob das Objekt links / mittig / rechts im Bild ist.
// Bildbreite HuskyLens V2 = 320 px
inline int8_t cameraHorizontalZone(const CameraResult& r, int16_t imgWidth = 320) {
    int16_t third = imgWidth / 3;
    if (r.xCenter < third)         return -1;  // links
    if (r.xCenter > imgWidth - third) return  1;  // rechts
    return 0;                                   // mittig
}
