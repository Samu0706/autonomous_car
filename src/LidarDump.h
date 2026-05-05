#pragma once
#include "LidarSensor.h"

/**
 * LidarDump
 *
 * Gibt Roh-LiDAR-Punkte als CSV über Serial aus, kompatibel mit
 * dem Python-Script (Lidar_saveScan.py).
 *
 * Format pro Scan:
 *   angle,distance\n   (für jeden Punkt)
 *   END\n              (Scan-Trennzeichen)
 *
 * Toggle: Serial-Kommando 'd' aktiviert / deaktiviert den Dump.
 * Im laufenden Betrieb einfach dauerhaft in main.cpp eingebaut lassen –
 * im inaktiven Zustand entsteht kein Overhead.
 */
class LidarDump {
public:
    // In loop() aufrufen – erkennt 'd' über Serial
    void checkSerial();

    // Nach lidar.isScanReady() aufrufen mit den Roh-Punkten
    void dump(const LidarPoint* points, int count);

    bool isActive() const { return _active; }

private:
    bool _active = false;
};
