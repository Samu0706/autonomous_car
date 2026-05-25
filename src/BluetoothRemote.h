#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "FollowTheGap.h"
#include "VehicleStateMachine.h"
#include "SpeedController.h"

class DisplayUI;  // forward declaration – vermeidet circular include mit DisplayUI.h
class Navigation; // forward declaration

/**
 * BLE UART Remote (Nordic UART Service).
 *
 * Empfangs-Protokoll (Web → ESP32):
 *   Einzelzeichen:  L / R / F / C / S / X  (wie bisher)
 *   Key=Value:      SPD=150   – Fahrgeschwindigkeit 0-255
 *                   DMIN=1500 – FGM Mindest-Freidistanz (mm)
 *                   ALPHA=0.4 – FGM Gewicht Lückengröße
 *                   BETA=0.6  – FGM Gewicht Tiefe
 *                   MGAP=3    – FGM Min. Lückenpunkte
 *                   BSTR=0.4  – Bias-Stärke für L/R/F-Befehle
 *                   CTHR=1200 – Kamera-Tag Abstandsschwellwert (mm)
 *
 * Sende-Protokoll (ESP32 → Web, JSON-Notification, max. 200 ms):
 *   {"a":-12.3,"s":"DRIVING","rp":800,"fp":360}
 *   a  = FGM-Lenkwinkel (°)
 *   s  = VSM-Zustandsname
 *   rp = Roh-Lidar-Punkte
 *   fp = Gefilterte Punkte
 */
class BluetoothRemote : public NimBLECharacteristicCallbacks,
                        public NimBLEServerCallbacks {
public:
    void begin(FollowTheGap*        fgm,
               VehicleStateMachine* vsm,
               SpeedController*     spd,
               DisplayUI*           ui,
               Navigation*          nav,
               const char*          deviceName = "LidarCar");

    // Im Arduino-Loop aufrufen
    void update();

    // Von main.cpp nach jedem Scan aufrufen (intern rate-limitiert auf 200 ms)
    // camTagId:      Zuletzt gesehene Tag-ID (0 = kein Tag)
    // camLastSeenMs: millis() zum Zeitpunkt des letzten Tags (0 = nie gesehen)
    // camConnected:  Kamera-UART antwortet innerhalb Timeout
    void sendTelemetry(float steerAngle, const char* state, int rawPts, int filtPts,
                       int camTagId, uint32_t camLastSeenMs, bool camConnected);

    char getLastCommand() const { return _lastCmd; }
    bool isConnected()    const { return _connected; }

private:
    // NimBLE callbacks (BLE-Task-Kontext – nur minimale Arbeit hier)
    void onWrite(NimBLECharacteristic* pChar) override;
    void onConnect(NimBLEServer* pServer) override;
    void onDisconnect(NimBLEServer* pServer) override;

    void handleCommand(const char* cmd);

    // Sendet aktuellen Konfigurationsstand als JSON-Notification an den Client.
    // Wird einmalig ~500 ms nach Verbindungsaufbau aufgerufen (in update()).
    void sendConfig();

    FollowTheGap*         _fgm        = nullptr;
    VehicleStateMachine*  _vsm        = nullptr;
    SpeedController*      _spd        = nullptr;
    DisplayUI*            _ui         = nullptr;
    Navigation*           _nav        = nullptr;

    NimBLECharacteristic* _txChar     = nullptr;   // Telemetrie-Notifications
    unsigned long         _lastTxMs   = 0;

    // Kommando-Puffer: BLE-Task schreibt, Loop-Task liest via update()
    char                  _cmdBuf[64] = {};
    volatile bool         _cmdReady   = false;

    char                  _lastCmd      = '-';
    bool                  _connected    = false;
    float                 _biasStrength = 0.4f;    // Stärke für L/R/F-Befehle

    // Config-Sync: einmalig nach Verbindungsaufbau senden
    volatile bool         _sendConfig   = false;
    unsigned long         _connectedMs  = 0;
};
