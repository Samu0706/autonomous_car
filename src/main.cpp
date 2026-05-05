#include <Arduino.h>
#include "DisplayController.h"
#include "RotaryEncoderController.h"
#include "LidarSensor.h"
#include "LidarProcessing.h"
#include "ObstacleDetection.h"
#include "FollowTheGap.h"
#include "Navigation.h"
#include "DisplayUI.h"
#include "VehicleStateMachine.h"
#include "SpeedController.h"
#include "SteeringController.h"
#include "LidarDump.h"
#include "BluetoothRemote.h"
#include "CameraSensor.h"

// =====================
// SYSTEM OBJECTS
// =====================
CameraSensor            camera;
DisplayController       display;
RotaryEncoderController encoder;
LidarSensor             lidar;
LidarProcessing         lidarProc;
ObstacleDetection       obstacles;
FollowTheGap            fgm;
Navigation              nav;
VehicleStateMachine     vsm;
SpeedController         speed;
SteeringController      steering;
DisplayUI               ui;
LidarDump               lidarDump;
BluetoothRemote         btRemote;

// =====================
// SETUP
// =====================
void setup() {
    Serial.begin(115200);

    // ---------------------
    // DISPLAY
    // ---------------------
    DisplayConfig dc;
    display.begin(dc);

    // ---------------------
    // ENCODER
    // ---------------------
    RotaryConfig rc;
    encoder.begin(rc);

    // ---------------------
    // LIDAR SENSOR
    // ---------------------
    LidarConfig lc;
    lc.rxPin     = 16;
    lc.maxPoints = 800;
    lidar.begin(lc);

    // ---------------------
    // LIDAR PROCESSING
    // ---------------------
    LidarProcessingConfig lpc;
    lidarProc.begin(lpc, lc.maxPoints);

    // ---------------------
    // SPEED CONTROLLER
    // – muss vor VSM init werden
    // ---------------------
    SpeedConfig sc;
    speed.begin(sc);

    // ---------------------
    // STEERING CONTROLLER
    // – muss vor VSM & Navigation init werden
    // ---------------------
    SteeringConfig stc;
    steering.begin(stc);

    // ---------------------
    // VEHICLE STATE MACHINE
    // – bekommt Speed + Steering
    // ---------------------
    StateMachineConfig smc;
    vsm.configure(smc);
    vsm.begin(&speed, &steering);

    // ---------------------
    // OBSTACLE DETECTION
    // – nach VSM, ruft vsm.update() intern
    // ---------------------
    ObstacleConfig oc;
    obstacles.begin(oc, &vsm);

    // ---------------------
    // FOLLOW THE GAP
    // ---------------------
    FGMConfig fgc;
    // fgc.dmin          = 500.0f;
    // fgc.alpha         = 1.0f;
    // fgc.beta          = 0.01f;
    // fgc.minGapSize    = 3;
    // fgc.maxSteerAngle = 30.0f;
    fgm.begin(fgc);

    // ---------------------
    // NAVIGATION
    // – verknüpft FGM + Steering + VSM
    //   lenkt nur in DRIVING aktiv
    // ---------------------
    nav.begin(&fgm, &steering, &vsm);

    // ---------------------
    // BLUETOOTH REMOTE
    // ---------------------
    btRemote.begin(&fgm, &vsm, &speed, &ui);

    // ---------------------
    // UI
    // ---------------------
    ui.begin(&display, &vsm, &speed, &steering, &nav, &lidarProc, &btRemote);

    // ---------------------
    // CAMERA (HuskyLens V2)
    // SDA=9, SCL=10 via Wire1
    // ---------------------
    CameraConfig cc;
    camera.begin(cc);

    Serial.println("[MAIN] SYSTEM READY");
}

// =====================
// LOOP
// =====================
void loop() {

    unsigned long start = millis();

    // ---------------------
    // CAMERA (HuskyLens V2)
    // ---------------------
    if (camera.update()) {
        int n = camera.resultCount();
        Serial.printf("[CAMERA] %d Objekt(e) erkannt:\n", n);
        for (int i = 0; i < n; i++) {
            CameraResult r = camera.getResult(i);
            Serial.printf("  [%d] ID=%u  pos=(%d,%d)  size=%dx%d  pitch=%.1f  yaw=%.1f  name=%s\n",
                i, r.id, r.xCenter, r.yCenter, r.width, r.height,
                r.pitch, r.yaw, r.name);
        }
    }

    // ---------------------
    // DUMP (toggle mit 'd' über Serial)
    // ---------------------
    lidarDump.checkSerial();

    // ---------------------
    // BLUETOOTH REMOTE
    // ---------------------
    btRemote.update();

    // ---------------------
    // ENCODER
    // ---------------------
    encoder.update();
    EncoderEvent e = encoder.getEvent();

    switch (e) {
        case EncoderEvent::BUTTON_CLICK:      ui.handleClick();    break;
        case EncoderEvent::BUTTON_LONG_PRESS: ui.handleHold();     break;
        case EncoderEvent::ROTATE_LEFT:       ui.handleRotate(-1); break;
        case EncoderEvent::ROTATE_RIGHT:      ui.handleRotate(1);  break;
        default: break;
    }

    // ---------------------
    // LIDAR
    // ---------------------
    lidar.update();

    if (lidar.isScanReady()) {

        // Roh-Dump (nur aktiv wenn per 'd' eingeschaltet)
        lidarDump.dump(lidar.getPoints(), lidar.getPointCount());

        // 1. Rohdaten filtern, diskretisieren, Referenzscan aufbauen/ergänzen.
        //    Gibt immer 360 gleichmäßig verteilte Punkte zurück (sobald isReady()).
        int filteredCount = lidarProc.process(
            lidar.getPoints(),
            lidar.getPointCount()
        );

        // 2. Hindernisse erkennen → ruft vsm.update() intern auf.
        //    Läuft auch vor isReady(), damit der Notfall-Stopp sofort aktiv ist.
        obstacles.process(lidarProc.getPoints(), filteredCount);

        // 3. Navigation / FGM – erst nach vollständigem Referenzscan.
        //    Vorher wären Lücken im Scan mit farDistance (Freiraum) aufgefüllt,
        //    was zu Fehlentscheidungen des Lenkreglers führen kann.
        if (lidarProc.isReady()) {
            nav.update(lidarProc.getPoints(), filteredCount);
        }

        // 4. Display – immer, damit die Map-Ansicht bereits im Idle-Modus
        //    den aufbauenden Scan zeigt (Scan-Coverage sichtbar).
        ui.draw(lidarProc.getPoints(), filteredCount);

        // 5. BLE-Telemetrie (intern auf 200 ms rate-limitiert)
        btRemote.sendTelemetry(
            nav.getCurrentAngle(),
            vsm.getStateName(),
            lidar.getPointCount(),
            filteredCount
        );

        lidar.clearScan();
    }

    // ---------------------
    // LOOP TIME
    // ---------------------
    unsigned long loopTime = millis() - start;
    ui.update(loopTime);
}