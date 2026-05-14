#include "Navigation.h"

// =============================
// BEGIN
// =============================
void Navigation::begin(FollowTheGap*        fgm,
                       SteeringController*  str,
                       VehicleStateMachine* vsm,
                       SpeedController*     spd,
                       CameraSensor*        cam) {
    _fgm = fgm;
    _str = str;
    _vsm = vsm;
    _spd = spd;
    _cam = cam;
}

// =============================
// KAMERA VERBUNDEN?
// =============================
bool Navigation::isCameraConnected() const {
    if (!_cam) return false;
    if (_cam->getLastReceivedMs() == 0) return false;  // noch nie empfangen
    return (millis() - _cam->getLastReceivedMs()) < CAM_TIMEOUT_MS;
}

// =============================
// TAG-PRIORITÄT
// Stop > Start > Gerade > Links > Rechts
// =============================
int Navigation::tagPriority(int id) {
    switch (id) {
        case 2: return 5;
        case 1: return 4;
        case 5: return 3;
        case 3: return 2;
        case 4: return 1;
        default: return 0;
    }
}

// =============================
// UPDATE CAMERA  –  Ende jeder loop()-Iteration
// =============================
void Navigation::updateCamera() {
    if (!_cam) return;

    // ── UART zuerst leeren (aktualisiert _lastReceivedMs) ─────────
    // Muss VOR dem Connectivity-Check passieren, damit Reconnect erkannt wird.
    bool           tagFound = false;
    int            bestPrio = -1;
    AprilTagResult bestTag;

    while (_cam->update()) {
        if (_cam->getState() == CameraState::TAG_FOUND) {
            const AprilTagResult& t = _cam->getResult();
            int prio = tagPriority(t.id);
            if (prio > bestPrio) {
                bestPrio = prio;
                bestTag  = t;
                tagFound = true;
            }
        }
    }

    // ── Kamera-Timeout: aktive Zustände zurücksetzen ──────────────
    if (!isCameraConnected()) {
        if (_biasActive) {
            _fgm->clearDirectionBias();
            _biasActive = false;
            _biasTagId  = 0;
        }
        if (_motorHeld) {
            resumeMotor();
            _motorHeld  = false;
        }
        _stopActive = false;
        return;
    }

    uint32_t now = millis();

    if (tagFound) {
        _lastTagId     = bestTag.id;
        _lastTagSeenMs = now;
        handleTag(bestTag);
    }

    checkBiasExpiry();
    checkStopTimer();

    // ── Motor-Halt durchsetzen (überschreibt ggf. VSM-Befehle) ───
    if (_motorHeld) _spd->stop();
}

// =============================
// TAG-BEHANDLUNG
// =============================
void Navigation::handleTag(const AprilTagResult& tag) {
    uint32_t now    = millis();
    float    distMm = tag.distance_cm * 10.0f;
    bool     close  = distMm < DIST_THRESHOLD_MM;

    switch (tag.id) {

        // ── ID 1: Start ──────────────────────────────────────────
        case 1:
            if (_vsm->getState() == VehicleState::IDLE) {
                Serial.println("[NAV] Start-Tag erkannt → DRIVING");
                _vsm->start();
            }
            break;

        // ── ID 2: Stop ───────────────────────────────────────────
        case 2: {
            if (!close) break;
            if (_vsm->getState() == VehicleState::IDLE) break;  // bereits gestoppt

            if (!_stopActive) {
                _stopActive      = true;
                _stopFirstSeenMs = now;
                _stopLastSeenMs  = now;
                _motorHeld       = true;
                _spd->stop();
                Serial.println("[NAV] Stop-Tag < 0.9m → Motor gestoppt, warte 3s");
            } else {
                _stopLastSeenMs = now;  // Tag weiter sichtbar → Timer läuft
            }
            break;
        }

        // ── ID 3: Links ──────────────────────────────────────────
        case 3: {
            if (!close) break;
            DirectionBias bias;
            bias.left = BIAS_STRENGTH;
            _fgm->setDirectionBias(bias);
            _biasActive        = true;
            _biasTagId         = 3;
            _biasTagLastSeenMs = now;
            break;
        }

        // ── ID 4: Rechts ─────────────────────────────────────────
        case 4: {
            if (!close) break;
            DirectionBias bias;
            bias.right = BIAS_STRENGTH;
            _fgm->setDirectionBias(bias);
            _biasActive        = true;
            _biasTagId         = 4;
            _biasTagLastSeenMs = now;
            break;
        }

        // ── ID 5: Gerade ─────────────────────────────────────────
        case 5: {
            DirectionBias bias;
            bias.straight = BIAS_STRENGTH;
            _fgm->setDirectionBias(bias);
            _biasActive        = true;
            _biasTagId         = 5;
            _biasTagLastSeenMs = now;
            break;
        }
    }
}

// =============================
// BIAS ABLAUF PRÜFEN
// =============================
void Navigation::checkBiasExpiry() {
    if (!_biasActive) return;
    if ((millis() - _biasTagLastSeenMs) >= BIAS_RESET_MS) {
        _fgm->clearDirectionBias();
        _biasActive = false;
        _biasTagId  = 0;
        Serial.println("[NAV] Richtungs-Bias zurückgesetzt");
    }
}

// =============================
// STOP-TIMER PRÜFEN
// =============================
void Navigation::checkStopTimer() {
    if (!_stopActive) return;

    uint32_t now = millis();

    // Grace-Period: kurze Unterbrechung im Sichtfeld tolerieren
    if (now - _stopLastSeenMs > STOP_GRACE_MS) {
        Serial.println("[NAV] Stop-Tag verschwunden → Fahrt fortgesetzt");
        _stopActive = false;
        if (_motorHeld) {
            resumeMotor();
            _motorHeld = false;
        }
        return;
    }

    // 3-Sekunden-Timer abgelaufen → in IDLE wechseln
    if (now - _stopFirstSeenMs >= STOP_DELAY_MS) {
        Serial.println("[NAV] Stop-Tag 3s erkannt → IDLE");
        _vsm->stop();
        _stopActive = false;
        _motorHeld  = false;
    }
}

// =============================
// MOTOR WIEDERAUFNEHMEN
// =============================
void Navigation::resumeMotor() {
    if (!_spd || !_vsm) return;
    switch (_vsm->getState()) {
        case VehicleState::DRIVING:   _spd->forward(); break;
        case VehicleState::REVERSING: _spd->reverse(); break;
        default: break;
    }
}

// =============================
// UPDATE (LIDAR-BASIERT)
// Nur in DRIVING aktiv; nach jedem Lidar-Scan aufrufen.
// =============================
void Navigation::update(const LidarPoint* points, int count) {
    if (_vsm->getState() != VehicleState::DRIVING) return;

    if (_fgm->process(points, count) && _fgm->hasValidGap()) {
        _str->setAngle(_fgm->getSteeringAngle());
    } else {
        _str->setAngle(0.0f);
    }
}

// =============================
// CURRENT ANGLE (für Debug/Display)
// =============================
float Navigation::getCurrentAngle() const {
    return _fgm ? _fgm->getSteeringAngle() : 0.0f;
}
