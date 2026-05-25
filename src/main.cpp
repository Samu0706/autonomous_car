#include <Arduino.h>
#include <esp_system.h>
#include "CameraSensor.h"
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

// =====================
// DEBUG-MODUS: Geradeausfahrt zum Trim-Kalibrieren
// 1 = FGM/Obstacles/Navigation deaktiviert, Auto fährt immer geradeaus.
//     Trim live über Website-Slider (TRIM=) einstellbar.
// 0 = Normalbetrieb (FGM, AprilTag-Navigation, Hinderniserkennung aktiv)
// =====================
#define DEBUG_STRAIGHT 1

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
    delay(2000);

    Serial.println("\n========================================");
#if DEBUG_STRAIGHT
    Serial.println("    SYSTEM START  [DEBUG: GERADEAUS]");
#else
    Serial.println("         SYSTEM START");
#endif
    Serial.println("========================================");

    // Reset-Grund ausgeben – hilft bei Crash-Diagnose:
    // 1=POWERON  3=SW-Reset  4=PANIC(Crash)  6=BROWNOUT(Spannung)  7=WDT
    esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("[RESET] Grund: %d", (int)rr);
    if      (rr == ESP_RST_PANIC)    Serial.print("  ← PANIC/Crash!");
    else if (rr == ESP_RST_BROWNOUT) Serial.print("  ← BROWNOUT! Spannung zu niedrig.");
    else if (rr == ESP_RST_TASK_WDT) Serial.print("  ← Task-Watchdog (Loop blockiert).");
    Serial.println();

    // ---------------------
    // CAMERA (OpenMV H7 Plus) – UART2, RX=17, TX=18
    // ---------------------
    CameraConfig cc;
    camera.begin(cc);
    Serial.println("[CAMERA] Warte auf erstes Signal (max. 3s)...");
    uint32_t camDeadline = millis() + 3000;
    while (millis() < camDeadline) {
        if (camera.update() && camera.getState() != CameraState::WAITING) {
            Serial.printf("[CAMERA] Signal empfangen: %s\n", camera.getRawLine());
            break;
        }
        delay(50);
    }
    if (camera.getLastReceivedMs() == 0) {
        Serial.println("[CAMERA] Kein Signal – System startet trotzdem");
    }

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
    // ---------------------
    SpeedConfig sc;
    speed.begin(sc);

    // ---------------------
    // STEERING CONTROLLER
    // ---------------------
    SteeringConfig stc;
    steering.begin(stc);

    // ---------------------
    // VEHICLE STATE MACHINE
    // ---------------------
    StateMachineConfig smc;
    vsm.configure(smc);
    vsm.begin(&speed, &steering);

    // ---------------------
    // OBSTACLE DETECTION
    // ---------------------
    ObstacleConfig oc;
    obstacles.begin(oc, &vsm);

    // ---------------------
    // FOLLOW THE GAP
    // ---------------------
    FGMConfig fgc;
    fgm.begin(fgc);

    // ---------------------
    // NAVIGATION – verknüpft FGM, Lenkung, VSM, Motor und Kamera
    // ---------------------
    nav.begin(&fgm, &steering, &vsm, &speed, &camera);

    // ---------------------
    // BLUETOOTH REMOTE
    // ---------------------
    btRemote.begin(&fgm, &vsm, &speed, &steering, &ui, &nav);

    // ---------------------
    // UI
    // ---------------------
    ui.begin(&display, &vsm, &speed, &steering, &nav, &lidarProc, &btRemote);

    Serial.println("[MAIN] SYSTEM READY");
}

// =====================
// LOOP
// =====================
void loop() {

    unsigned long start = millis();

    lidarDump.checkSerial();
    btRemote.update();

    encoder.update();
    EncoderEvent e = encoder.getEvent();
    switch (e) {
        case EncoderEvent::BUTTON_CLICK:      ui.handleClick();    break;
        case EncoderEvent::BUTTON_LONG_PRESS: ui.handleHold();     break;
        case EncoderEvent::ROTATE_LEFT:       ui.handleRotate(-1); break;
        case EncoderEvent::ROTATE_RIGHT:      ui.handleRotate(1);  break;
        default: break;
    }

    lidar.update();

    if (lidar.isScanReady()) {

        lidarDump.dump(lidar.getPoints(), lidar.getPointCount());

        int filteredCount = lidarProc.process(
            lidar.getPoints(),
            lidar.getPointCount()
        );

#if DEBUG_STRAIGHT
        // ── DEBUG: Geradeausfahrt – FGM/Nav/Obstacles deaktiviert ──
        // Lenkung immer auf 0° + aktuellen Trim-Offset.
        // Trim live über BLE TRIM= oder Website-Slider einstellbar.
        steering.setAngle(0.0f);
#else
        // ── NORMALBETRIEB ──
        obstacles.process(lidarProc.getPoints(), filteredCount);

        if (lidarProc.isReady()) {
            nav.update(lidarProc.getPoints(), filteredCount);
        }
#endif

        ui.draw(lidarProc.getPoints(), filteredCount);

        btRemote.sendTelemetry(
            steering.getCurrentAngle(),
            vsm.getStateName(),
            lidar.getPointCount(),
            filteredCount,
            nav.getLastTagId(),
            nav.getLastTagSeenMs(),
            nav.isCameraConnected()
        );

        lidar.clearScan();
    }

#if !DEBUG_STRAIGHT
    // Kamera-Logik nur im Normalbetrieb (AprilTag-Navigation)
    nav.updateCamera();
#endif

    unsigned long loopTime = millis() - start;
    ui.update(loopTime);
}
