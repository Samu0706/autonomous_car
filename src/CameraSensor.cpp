#include "CameraSensor.h"

// Eigener HardwareSerial-Port für HuskyLens (Serial1 ist frei)
static HardwareSerial huskylensSerial(1);

bool CameraSensor::begin(const CameraConfig& cfg) {
    _cfg = cfg;

    huskylensSerial.begin(cfg.baudRate, SERIAL_8N1, cfg.rxPin, cfg.txPin);

    // HuskyLens hat eigenes OS – braucht bei separater Stromversorgung bis zu 5s zum Booten.
    // Retry-Schleife für max. 30 Sekunden.
    Serial.println("[CAMERA] Warte auf HuskyLens V2 via UART (max. 30s) ...");
    uint32_t deadline = millis() + 30000;
    uint8_t attempt = 0;
    while (millis() < deadline) {
        if (_huskylens.begin(huskylensSerial)) {
            Serial.printf("[CAMERA] HuskyLens V2 bereit (nach %d Versuchen)\n", attempt + 1);
            _huskylens.switchAlgorithm(cfg.algorithm);
            _initialized = true;
            return true;
        }
        attempt++;
        Serial.printf("[CAMERA]   Versuch %d fehlgeschlagen, warte...\n", attempt);
        delay(500);
    }

    Serial.println("[CAMERA] FEHLGESCHLAGEN – HuskyLens antwortet nicht nach 30s");
    Serial.println("[CAMERA]   Prüfe: GND verbunden? TX(grün)→Pin9, RX(blau)→Pin10? UART-Modus gesetzt?");
    return false;
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
        strncpy(r.name, raw->name.c_str(), sizeof(r.name) - 1);
        r.name[sizeof(r.name) - 1] = '\0';
    }

    return _count > 0;
}

CameraResult CameraSensor::getResult(int index) const {
    if (index < 0 || index >= _count) return {};
    return _results[index];
}
