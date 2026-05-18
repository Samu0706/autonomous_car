#pragma once
#include "FollowTheGap.h"
#include "SteeringController.h"
#include "VehicleStateMachine.h"
#include "SpeedController.h"
#include "CameraSensor.h"
#include "LidarSensor.h"

/**
 * Navigation – Orchestrator für FGM und Kamera (OpenMV AprilTags).
 *
 * Tag-Kodierung:
 *   ID 1 = Start      → vsm.start()  wenn IDLE
 *   ID 2 = Stop       → Motor sofort stoppen, nach 3s vsm.stop() → IDLE
 *   ID 3 = Links      → FGM-Bias links   (nur wenn dist < distThreshold)
 *   ID 4 = Rechts     → FGM-Bias rechts  (nur wenn dist < distThreshold)
 *   ID 5 = Gerade     → FGM-Bias gerade  (sofort bei Erkennung)
 *
 * Bias-Reset: 2 s nachdem das Richtungs-Tag nicht mehr sichtbar ist.
 * Stop-Bedingung: Tag muss 3 s ununterbrochen bei < distThreshold sichtbar sein.
 * Standard-Schwellwert: 1.2 m (per BLE CTHR= einstellbar).
 * Kamera-Timeout: 3 s kein Signal → "nicht verbunden"; bei Reconnect normaler Betrieb.
 *
 * Aufruf-Schema in loop():
 *   1. nav.update(points, count)  – nach Lidar-Scan (Lenkung)
 *   2. nav.updateCamera()         – am Ende von loop() (überschreibt ggf. VSM-Motorbefehle)
 */
class Navigation {
public:
    void begin(FollowTheGap*        fgm,
               SteeringController*  str,
               VehicleStateMachine* vsm,
               SpeedController*     spd,
               CameraSensor*        cam);

    // Lidar-basiertes Lenken – nur in DRIVING aktiv; nach Lidar-Scan aufrufen.
    void update(const LidarPoint* points, int count);

    // Kamera-Logik – am ENDE jeder loop()-Iteration aufrufen.
    // Verarbeitet alle gepufferten UART-Zeilen, setzt Bias / Stop-Logik,
    // und überschreibt ggf. VSM-Motorbefehle (Motor-Halt).
    void updateCamera();

    // Einstellbarer Tag-Abstandsschwellwert (Standard 1.2 m)
    void     setTagDistThreshold(float mm) { _distThresholdMm = mm; }

    // ── Getter für Telemetrie ──────────────────────────────────────
    float    getCurrentAngle()   const;
    int      getLastTagId()      const { return _lastTagId; }
    uint32_t getLastTagSeenMs()  const { return _lastTagSeenMs; }
    bool     isCameraConnected() const;

private:
    static int tagPriority(int id);
    void handleTag(const AprilTagResult& tag);
    void checkBiasExpiry();
    void checkStopTimer();
    void resumeMotor();

    FollowTheGap*        _fgm = nullptr;
    SteeringController*  _str = nullptr;
    VehicleStateMachine* _vsm = nullptr;
    SpeedController*     _spd = nullptr;
    CameraSensor*        _cam = nullptr;

    // Letztes gesehenes Tag (für Telemetrie)
    int      _lastTagId      = 0;
    uint32_t _lastTagSeenMs  = 0;

    // Richtungs-Bias
    int      _biasTagId          = 0;
    uint32_t _biasTagLastSeenMs  = 0;
    uint32_t _biasHoldUntilMs    = 0;   // Mindesthaltedauer ab erster Aktivierung
    bool     _biasActive         = false;

    // Stop-Tag Sequenz
    bool     _stopActive         = false;
    uint32_t _stopFirstSeenMs    = 0;
    uint32_t _stopLastSeenMs     = 0;
    bool     _motorHeld          = false;  // Motor wurde durch uns gestoppt

    // Einstellbarer Tag-Abstandsschwellwert
    float                    _distThresholdMm    = 1200.0f; // Standard 1.2 m

    // Konstanten
    static constexpr uint32_t BIAS_RESET_MS      = 2000;  // nach letztem Sehen
    static constexpr uint32_t BIAS_MIN_HOLD_MS   = 2000;  // Mindesthaltedauer ab Aktivierung
    static constexpr uint32_t STOP_DELAY_MS      = 3000;
    static constexpr uint32_t STOP_GRACE_MS      = 500;    // kurze Unterbrechung tolerieren
    static constexpr uint32_t CAM_TIMEOUT_MS     = 3000;
    static constexpr float    BIAS_STRENGTH      = 1.0f;
};
