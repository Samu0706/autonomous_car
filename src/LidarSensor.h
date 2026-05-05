#pragma once
#include <Arduino.h>
#include <LDS_YDLIDAR_X2_X2L.h>

/**
 * BASIC LIDAR CONTROLLER
 *
 * - Speichert alle Punkte eines vollständigen Scans
 * - Jeder Punkt: Winkel + Distanz
 * - Nach scan_completed = neue Daten verfügbar
 */

struct LidarPoint {
    float angle;
    float distance;
};

struct LidarConfig {
    uint8_t rxPin = 16;
    int maxPoints = 500;   // max Punkte pro Scan
};

class LidarSensor {
public:
    void begin(const LidarConfig& cfg);
    void update();

    // Scan Zugriff
    bool isScanReady() const;
    int  getPointCount() const;
    const LidarPoint *getPoints() const;

    void clearScan();

private:
    static void scanCallback(float angle, float distance, float quality, bool scan_completed);
    static int  serialReadCallback();

    void processPoint(float angle, float distance, bool scan_completed);

    static LidarSensor* _instance;

    LidarConfig _cfg;

    HardwareSerial _serial = HardwareSerial(1);
    LDS_YDLIDAR_X2_X2L _lidar;

    LidarPoint* _points = nullptr;
    int _pointCount = 0;

    bool _scanReady = false;
};