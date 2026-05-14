#include "CameraSensor.h"

bool CameraSensor::begin(const CameraConfig& cfg) {
    _cfg = cfg;
    _serial.begin(cfg.baudRate, SERIAL_8N1, cfg.rxPin, cfg.txPin);
    Serial.printf("[CAMERA] UART2 init  RX=%d TX=%d  %lu baud\n",
                  cfg.rxPin, cfg.txPin, cfg.baudRate);
    return true;
}

bool CameraSensor::update() {
    bool gotLine = false;
    while (_serial.available()) {
        _lastReceivedMs = millis();   // jedes empfangene Byte zählt als "connected"
        char c = (char)_serial.read();
        if (c == '\n' || c == '\r') {
            if (_linePos > 0) {
                _lineBuf[_linePos] = '\0';
                parseLine();
                _linePos = 0;
                gotLine  = true;
            }
        } else if (_linePos < BUF_SIZE - 1) {
            _lineBuf[_linePos++] = c;
        }
    }
    return gotLine;
}

void CameraSensor::parseLine() {
    if (strncmp(_lineBuf, "<STARTUP>", 9) == 0) {
        _state = CameraState::NO_TAG;
        return;
    }
    if (strncmp(_lineBuf, "<STATUS>", 8) == 0) {
        _state = CameraState::NO_TAG;
        return;
    }
    if (strncmp(_lineBuf, "<DATA>", 6) == 0) {
        int   id   = 0;
        float ang  = 0.f;
        float dist = 0.f;
        if (sscanf(_lineBuf, "<DATA>,ID=%d,ANG=%f,DIST=%f", &id, &ang, &dist) == 3) {
            _result.id          = id;
            _result.angle_deg   = ang;
            _result.distance_cm = dist;
            _state = CameraState::TAG_FOUND;
        }
    }
}
