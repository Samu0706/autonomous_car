#pragma once
#include <Arduino.h>
#include "LidarSensor.h"

/**
 * Follow The Gap Method (FGM)
 *
 * Ermittelt im vorderen Sichtbereich (180° ±90°) zusammenhängende
 * Lücken (Punkte mit Distanz > dmin), bewertet diese nach Größe
 * und mittlerer Tiefe, und berechnet aus der besten Lücke einen
 * relativen Lenkwinkel.
 *
 * Eingabe:  Gefilterte LidarPoints
 * Ausgabe:  Lenkwinkel in Grad relativ (-maxAngle .. +maxAngle)
 *           → direkt kompatibel mit SteeringController::setAngle()
 *
 * Konvention:
 *   LiDAR 180° = vorne (geradeaus)  →  Lenkwinkel  0°
 *   LiDAR < 180° (z.B. 150°)        →  links      negativ
 *   LiDAR > 180° (z.B. 210°)        →  rechts     positiv
 */
struct FGMConfig {
    // Schwellwert: Punkte mit distance > dmin zählen als "frei"
    float dmin = 1500.0f;           // mm

    // Frontbereich (passend zu deinem Setup: 180° = vorne, ±90°)
    float frontSectorLeft  = 110.0f;
    float frontSectorRight = 250.0f;

    // Gewichtung für Bewertung der Lücken
    // Score = alpha * Punktanzahl + beta * mittlere Tiefe
    float alpha = 0.4f;            // Gewichtung Größe (Punktanzahl)
    float beta  = 0.6f;            // Gewichtung Tiefe (mm → skaliert)

    // Minimale Lückengröße (Punkte) – kleinere werden verworfen
    int minGapSize = 3;

    // Maximaler Lenkausschlag (passend zu SteeringController)
    float maxSteerAngle = 30.0f;   // ° – ±30

    // Übereinstimmend mit LidarProcessingConfig::farDistance.
    // Punkte auf diesem Wert (kein Echo → d=0-Ersatz) zählen als blockiert,
    // nicht als Freiraum – verhindert Phantom-Lücken bei fehlendem LiDAR-Signal.
    float farDistance = 9000.0f;

    // --- NEU: Disparity Extender ---
    // Bei jedem Tiefensprung zwischen Nachbarpunkten wird die nähere Seite
    // künstlich um einen Sicherheitsradius aufgebläht, damit das Auto nicht
    // knapp an Hindernis-Kanten vorbeischrammt. Defaults passen zum
    // Visualisierungs-Tool.
    bool  disparityEnabled    = true;
    float disparityInflate    = 300.0f;    // Sicherheitsbreite in mm (20cm Auto + 15cm/Seite Puffer)
    float disparityThreshold  = 000.0f;    // Mindest-Distanzsprung in mm
    float disparityMaxDist    = 4000.0f;   // Disparities jenseits davon ignorieren

    // Nahbereich-Extender: Blasen um sehr nahe Punkte (unabhängig von Tiefensprüngen)
    bool  nearFieldEnabled   = true;
    float nearFieldThreshold = 500.0f;    // mm – Punkte näher als das bekommen Blase
    float nearFieldInflate   = 200.0f;    // mm – Blasendurchmesser (20 cm)
};

/**
 * Richtungsgewichtung – von außen setzbar, bleibt bis zum expliziten Reset aktiv.
 *
 * Werte 0.0 = kein Einfluss, typisch 0.1–0.5.
 * Der Bonus wird auf den normalisierten Score (0..1) addiert, sodass
 * eine gewünschte Richtung gegenüber gleichwertigen Alternativen bevorzugt wird,
 * aber eine deutlich bessere Lücke in einer anderen Richtung trotzdem gewinnt.
 *
 * Winkelkonvention (LiDAR, physische Fahrtrichtung):
 *   > 180° + straightHalfWidth  → rechts
 *   < 180° - straightHalfWidth  → links
 *   sonst                       → geradeaus
 */
struct DirectionBias {
    float left             = 0.0f;   // Bonus für Lücken nach links
    float straight         = 0.0f;   // Bonus für Lücken geradeaus
    float right            = 0.0f;   // Bonus für Lücken nach rechts
    float straightHalfWidth = 20.0f; // ° – Fenster um 180° das als geradeaus gilt
};

class FollowTheGap {
public:
    void begin(const FGMConfig& cfg);

    // Laufzeit-Parametrierung (z.B. über BLE)
    void      setConfig(const FGMConfig& cfg) { _cfg = cfg; }
    FGMConfig getConfig() const               { return _cfg; }

    /**
     * Hauptaufruf – einmal pro Scan.
     * Berechnet den neuen Lenkwinkel aus den gefilterten Punkten.
     * @return true wenn eine gültige Lücke gefunden wurde
     */
    bool process(const LidarPoint* points, int count);

    // Richtungspräferenz setzen / zurücksetzen (thread-safe auf Single-Core ESP32)
    void setDirectionBias(const DirectionBias& bias) { _bias = bias; }
    void clearDirectionBias()                        { _bias = DirectionBias{}; }

    // Ergebnisse
    float getSteeringAngle() const { return _steeringAngle; }
    bool  hasValidGap()      const { return _validGap; }
    int   getGapCount()      const { return _gapCount; }
    int   getBestGapIdx()    const { return _bestGapIdx; }

    struct GapInfo {
        float startAngle;
        float endAngle;
        float centerAngle;
        float meanDepth;    // mm
        float score;
        bool  isBest;
    };
    bool getGapInfo(int idx, GapInfo& out) const;

private:
    // Interne Struktur für eine erkannte Lücke
    struct Gap {
        int   startIdx;      // Index in _frontPoints
        int   endIdx;
        int   size;          // Anzahl Punkte in der Lücke
        float meanDepth;     // mittlere Distanz in mm
        float centerAngle;   // distanzgewichteter Mittelwinkel (LiDAR-System)
        float score;         // Bewertung (inkl. Richtungsbonus)
    };

    static const int MAX_GAPS        = 20;
    static const int MAX_FRONT_POINTS = 400;

    // Frontpunkte extrahieren und nach Winkel sortieren
    int extractFrontPoints(const LidarPoint* points, int count);

    void applyDisparityExtender();
    void applyNearFieldExtender();

    // Lücken finden (zusammenhängende Punkte mit d > dmin)
    int findGaps();

    // Beste Lücke auswählen
    int selectBestGap(int gapCount);

    // Zielwinkel aus Lücke berechnen (gewichteter Winkel-Mittelwert)
    float computeTargetAngle(const Gap& gap);

    // LiDAR-Winkel → relativen Lenkwinkel
    float lidarAngleToSteering(float lidarAngle) const;

    FGMConfig     _cfg;
    DirectionBias _bias;

    // Nur Punkte aus dem Frontsektor (sortiert nach Winkel)
    LidarPoint _frontPoints[MAX_FRONT_POINTS];
    int        _frontCount = 0;

    Gap _gaps[MAX_GAPS];
    int _gapCount = 0;

    float _steeringAngle = 0.0f;
    bool  _validGap      = false;
    int   _bestGapIdx    = -1;
};