#pragma once
#include <Arduino.h>
#include "LidarSensor.h"

struct LidarProcessingConfig {
    float minDistance   = 150.0f;    // mm – Eigenreflexion rausfiltern
    float maxDistance   = 15000.0f;  // mm – zu weit / ungültig
    int   medianWindow  = 3;         // Nachbarn pro Seite (3 = 7-Punkt-Median)

    // YDLIDAR X2: distance=0 bedeutet kein Echo (Freiraum, Glas, >8m).
    // Als farDistance behandeln, damit FGM keine künstlichen Hindernisse sieht.
    bool  treatZeroAsFar = true;
    float farDistance    = 9000.0f;  // Ersatzdistanz für d=0 (mm)

    // Mindest-Winkelabdeckung (Grad 0-360) damit ein Scan als vollständig gilt
    // und als Referenz gespeichert wird. 270 = 75% Abdeckung.
    // Hinweis: isReady() wird erst true wenn dieser Schwellwert erstmals erreicht
    // wurde – nav.update()/FGM erst danach aufrufen.
    int   minScanCoverage = 270;
};

class LidarProcessing {
public:
    void begin(const LidarProcessingConfig& cfg, int maxPoints);

    // Hauptaufruf – einmal pro Scan.
    // Gibt immer 360 zurück (einen Punkt pro Grad).
    int process(const LidarPoint* raw, int rawCount);

    // Gefilterte Punkte für Display / FGM / ObstacleDetection
    const LidarPoint* getPoints()      const { return _filtered; }
    int               getPointCount()  const { return _filteredCount; }

    // true sobald der erste vollständige Referenzscan aufgebaut wurde.
    // Vor isReady() kann FGM keine sinnvolle Entscheidung treffen.
    bool isReady()         const { return _hasRef; }
    int  getScanCoverage() const { return _coverage; }

private:
    void  applyRangeFilter(const LidarPoint* raw, int rawCount);
    void  discretizeScan();    // float-Winkel → int-Grad, min-Distanz pro Grad
    void  updateReference();   // Referenzscan aufbauen / aktualisieren
    void  buildOutputScan();   // lückenlosen 360-Punkte-Scan in _filtered schreiben
    void  applyMedianFilter();
    float median(float* arr, int count) const;

    LidarProcessingConfig _cfg;

    // Ausgangs-Puffer
    LidarPoint* _filtered      = nullptr;  // Rohdaten-Puffer (applyRangeFilter) +
                                           // 360-Punkte-Ausgabe (buildOutputScan)
    LidarPoint* _temp          = nullptr;  // Median-Arbeitspuffer (360 Elemente)
    int         _filteredCount = 0;
    int         _maxPoints     = 0;

    // Diskretisierte 360°-Scan-Puffer.
    // uint16_t (2 Byte/Grad) statt LidarPoint (8 Byte) spart 75% RAM.
    // Wertebereich: 0–65535 mm – ausreichend für farDistance=9000 mm.
    uint16_t _scanRef [360];   // letzter vollständiger Referenzscan
    uint16_t _scanWork[360];   // aktueller Scan (pro process()-Aufruf)
    bool     _refValid [360];  // gültige Slots in _scanRef
    bool     _workValid[360];  // gültige Slots in _scanWork
    bool     _hasRef    = false;
    int      _coverage  = 0;   // abgedeckte Grad im letzten Scan
};