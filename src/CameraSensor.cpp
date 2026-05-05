#include "CameraSensor.h"

bool CameraSensor::begin(const CameraConfig& cfg) {
    _cfg = cfg;

    // Zweiter I2C-Bus (Wire ist durch das Display belegt auf SDA=4/SCL=5)
    Wire1.begin(cfg.sdaPin, cfg.sclPin);

    if (!_huskylens.begin(Wire1)) {
        Serial.println("[CAMERA] Init fehlgeschlagen – Verkabelung und I2C-Protokoll prüfen");
        Serial.println("[CAMERA]   green >> SDA (9), blue >> SCL (10)");
        Serial.println("[CAMERA]   HuskyLens: System Settings >> Protocol Type >> I2C");
        return false;
    }

    _huskylens.switchAlgorithm(cfg.algorithm);
    _initialized = true;
    Serial.println("[CAMERA] HuskyLens V2 bereit");
    return true;
}

bool CameraSensor::update() {
    if (!_initialized) return false;

    _count = 0;

    if (!_huskylens.getResult(_cfg.algorithm)) return false;

    while (_huskylens.available(_cfg.algorithm) && _count < MAX_RESULTS) {
        Result* raw = static_cast<Result*>(
            _huskylens.popCachedResult(_cfg.algorithm));
        if (!raw) break;

        CameraResult& r = _results[_count++];
        r.id      = raw->ID;
        r.xCenter = raw->xCenter;
        r.yCenter = raw->yCenter;
        r.width   = raw->width;
        r.height  = raw->height;
        r.pitch   = raw->pitch;
        r.yaw     = raw->yaw;
        strncpy(r.name, raw->name, sizeof(r.name) - 1);
        r.name[sizeof(r.name) - 1] = '\0';
    }

    return _count > 0;
}

CameraResult CameraSensor::getResult(int index) const {
    if (index < 0 || index >= _count) return {};
    return _results[index];
}
