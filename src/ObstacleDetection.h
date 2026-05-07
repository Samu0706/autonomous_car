#pragma once
#include <Arduino.h>
#include "LidarSensor.h"
#include "VehicleStateMachine.h"

struct ObstacleConfig {

    // --- Schwellwerte ---
    float limitMiddle         = 500.0f;   // mm – vorne Mitte
    float limitSide           = 190.0f;   // mm – vorne Seiten
    float limitReverseMiddle  = 250.0f;   // mm – hinten Mitte
    float limitReverseSide    = 190.0f;   // mm – hinten Seiten

    // --- Vorne: Winkelbereiche ---
    float frontMiddleLeft  = 160.0f;
    float frontMiddleRight = 200.0f;

    float frontSideLeftLeft  = 110.0f;
    float frontSideLeftRight = 170.0f;

    float frontSideRightLeft  = 190.0f;
    float frontSideRightRight = 250.0f;

    // --- Hinten: Winkelbereiche (symmetrisch zu vorne, um 180° versetzt) ---
    // Mitte hinten:       340°–20°   (wrap-around)
    float rearMiddleLeft  = 340.0f;
    float rearMiddleRight =  20.0f;

    // Seite links hinten: 290°–340°
    float rearSideLeftLeft  = 290.0f;
    float rearSideLeftRight = 340.0f;

    // Seite rechts hinten: 20°–70°
    float rearSideRightLeft  =  20.0f;
    float rearSideRightRight =  70.0f;

    // --- Cluster: Mindestanzahl Punkte für echtes Hindernis ---
    int counterLimit = 1; //Keine fehlerhaften Punkte erkennen, aber trotzdem Hindernisse >10cm ***kann ggf. reduziert werden -> testen***
};

class ObstacleDetection {
public:
    void begin(const ObstacleConfig& cfg, VehicleStateMachine* vsm);

    // Einmal pro gefiltertem Scan aufrufen
    // Ruft intern vsm.update() auf
    void process(const LidarPoint* points, int count);

    // Flags abfragen (optional, für Debug/Display)
    bool isObstacleAhead() const { return _obstacleAhead; }
    bool isObstacleRear()  const { return _obstacleRear;  }

private:
    bool inRange(float angle, float left, float right) const;

    ObstacleConfig       _cfg;
    VehicleStateMachine* _vsm = nullptr;

    bool _obstacleAhead = false;
    bool _obstacleRear  = false;
};