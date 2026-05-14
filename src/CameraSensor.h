#pragma once
#include <Arduino.h>
#include "CameraProcessing.h"

/**
 * CameraSensor – OpenMV Cam H7 Plus via UART
 *
 * Protokoll (von der Kamera, 115200 Baud, Zeilenformat):
 *   <STARTUP>,OK
 *   <DATA>,ID=<n>,ANG=<f>,DIST=<f>
 *   <STATUS>,NO_TAG
 *
 * Verdrahtung ESP32-S3:
 *   GPIO 17 (RX) ← TX der Kamera (UART3 P4)
 *   GPIO 18 (TX) → RX der Kamera (UART3 P5)  [optional]
 *   GND          ← GND der Kamera
 */

struct CameraConfig {
    uint8_t  rxPin    = 17;
    uint8_t  txPin    = 18;
    uint32_t baudRate = 115200;
};

class CameraSensor {
public:
    bool begin(const CameraConfig& cfg);

    // Nicht-blockierend – in loop() aufrufen.
    // Gibt true zurück wenn eine vollständige Zeile verarbeitet wurde.
    bool update();

    CameraState           getState()          const { return _state; }
    const AprilTagResult& getResult()         const { return _result; }
    const char*           getRawLine()        const { return _lineBuf; }

    // Zeitstempel des letzten empfangenen Bytes – 0 wenn noch nie empfangen.
    // Wird von Navigation für den 3s-Timeout genutzt.
    uint32_t              getLastReceivedMs() const { return _lastReceivedMs; }

private:
    void parseLine();

    HardwareSerial _serial{2};
    CameraConfig   _cfg;
    CameraState    _state          = CameraState::WAITING;
    AprilTagResult _result;
    uint32_t       _lastReceivedMs = 0;

    static constexpr int BUF_SIZE = 128;
    char _lineBuf[BUF_SIZE];
    int  _linePos = 0;
};
