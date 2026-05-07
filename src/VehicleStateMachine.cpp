#include "VehicleStateMachine.h"

// =============================
// INIT
// =============================
void VehicleStateMachine::begin(SpeedController* spd, SteeringController* str) {
    _speed    = spd;
    _steering = str;
    _state    = VehicleState::IDLE;
    _stateEnteredAt_ms = millis();
}

// =============================
// START
// =============================
void VehicleStateMachine::start() {
    if (_state == VehicleState::IDLE) {
        transitionTo(VehicleState::DRIVING);
    }
}

// =============================
// STOP
// =============================
void VehicleStateMachine::stop() {
    transitionTo(VehicleState::IDLE);
}

// =============================
// UPDATE
// =============================
void VehicleStateMachine::update(bool obstacleAhead, bool obstacleRear) {

    unsigned long now = millis();

    switch (_state) {

        // -------------------------
        case VehicleState::IDLE:
            break;

        // -------------------------
        case VehicleState::DRIVING:
            // Lenkwinkel laufend aufzeichnen
            _steerHistory[_steerHistIdx] = _steering->getCurrentAngle();
            _steerHistIdx = (_steerHistIdx + 1) % STEER_HISTORY;
            if (_steerHistCount < STEER_HISTORY) _steerHistCount++;

            if (obstacleAhead) {
                _speed->stop();
                transitionTo(VehicleState::WAIT_BEFORE_REVERSE);
            }
            break;

        // -------------------------
        case VehicleState::WAIT_BEFORE_REVERSE:
            if (now - _stateEnteredAt_ms >= _cfg.waitBeforeReverse_ms) {
                if (!obstacleAhead) {
                    transitionTo(VehicleState::DRIVING);
                } else {
                    transitionTo(VehicleState::REVERSING);
                }
            }
            break;

        // -------------------------
        case VehicleState::REVERSING:
            if (obstacleRear) {
                _speed->stop();
                transitionTo(VehicleState::WAIT_BEFORE_EMERGENCY);
            } else if (now - _stateEnteredAt_ms >= _cfg.reverseTime_ms) {
                transitionTo(VehicleState::DRIVING);
            }
            break;

        // -------------------------
        case VehicleState::WAIT_BEFORE_EMERGENCY:
            if (now - _stateEnteredAt_ms >= _cfg.waitBeforeEmergency_ms) {
                if (!obstacleRear && !obstacleAhead) {
                    transitionTo(VehicleState::DRIVING);
                } else {
                    transitionTo(VehicleState::EMERGENCY);
                }
            }
            break;

        // -------------------------
        case VehicleState::EMERGENCY:
            if (!obstacleRear && !obstacleAhead &&
                now - _stateEnteredAt_ms > _cfg.holdOffTime_ms) {
                transitionTo(VehicleState::IDLE);
            }
            break;
    }
}

// =============================
// TRANSITION
// =============================
void VehicleStateMachine::transitionTo(VehicleState newState) {

    if (newState == _state) return;

    _state             = newState;
    _stateEnteredAt_ms = millis();

    Serial.print("[VSM] -> ");
    Serial.println(getStateName());

    switch (_state) {

        case VehicleState::IDLE:
            _steering->center();
            _speed->stop();
            break;

        case VehicleState::DRIVING:
            _steering->center();
            _speed->forward();
            // Puffer zurücksetzen – neue Fahrt, neue Historie
            _steerHistIdx   = 0;
            _steerHistCount = 0;
            break;

        case VehicleState::WAIT_BEFORE_REVERSE: {
            float median = computeSteerMedian();
            if (fabsf(median) < 5.0f) {
                _reverseSteerAngle  = _fallbackReverseDir * 20.0f;
                _fallbackReverseDir = -_fallbackReverseDir;
            } else {
                _reverseSteerAngle = -median;
            }
            _speed->stop();
            _steering->setAngle(_reverseSteerAngle);
            Serial.printf("[VSM] reverseAngle=%.1f (median von %d Winkeln)\n",
                          _reverseSteerAngle, _steerHistCount);
            break;
        }

        case VehicleState::REVERSING:
            _steering->setAngle(_reverseSteerAngle);
            _speed->reverse();
            break;

        case VehicleState::WAIT_BEFORE_EMERGENCY:
            _speed->stop();
            // Lenkung mittig – kein Platz mehr zum Manövrieren
            _steering->center();
            break;

        case VehicleState::EMERGENCY:
            _speed->stop();
            _steering->center();
            break;
    }
}

// =============================
// MEDIAN DER LENKWINKEL-HISTORIE
// =============================
float VehicleStateMachine::computeSteerMedian() const {
    if (_steerHistCount == 0) return 0.0f;

    float sorted[STEER_HISTORY];
    memcpy(sorted, _steerHistory, _steerHistCount * sizeof(float));

    // Insertion Sort – für n≤20 ausreichend schnell
    for (int i = 1; i < _steerHistCount; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[_steerHistCount / 2];
}

// =============================
// STATE NAME
// =============================
const char* VehicleStateMachine::getStateName() const {
    switch (_state) {
        case VehicleState::IDLE:                  return "IDLE";
        case VehicleState::DRIVING:               return "DRIVING";
        case VehicleState::WAIT_BEFORE_REVERSE:   return "WAITING...";
        case VehicleState::REVERSING:             return "REVERSING";
        case VehicleState::WAIT_BEFORE_EMERGENCY: return "BLOCKED";
        case VehicleState::EMERGENCY:             return "EMERGENCY";
        default:                                  return "UNKNOWN";
    }
}