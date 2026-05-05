#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_HuskylensV2.h"
#include "CameraProcessing.h"

struct CameraConfig {
    uint8_t sdaPin    = 9;
    uint8_t sclPin    = 10;
    uint8_t algorithm = ALGORITHM_OBJECT_RECOGNITION;
};

class CameraSensor {
public:
    bool begin(const CameraConfig& cfg);

    // Ergebnis vom HuskyLens abrufen – true wenn neue Daten verfügbar.
    bool update();

    bool isInitialized() const { return _initialized; }

    int            resultCount() const { return _count; }
    CameraResult   getResult(int index) const;

private:
    HuskylensV2   _huskylens;
    CameraConfig  _cfg;
    bool          _initialized = false;

    static constexpr int MAX_RESULTS = 16;
    CameraResult  _results[MAX_RESULTS];
    int           _count = 0;
};
