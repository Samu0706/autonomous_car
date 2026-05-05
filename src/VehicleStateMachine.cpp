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
            // Lenkung wieder geradeaus – FGM / ObstacleDetection übernimmt ab hier
            _steering->center();
            _speed->forward();
            break;

        case VehicleState::WAIT_BEFORE_REVERSE:
            // Motor steht, Lenkwinkel für kommendes Reversing vorwählen
            // damit der Servo Zeit hat zu reagieren bevor Motor anläuft
            _speed->stop();
            _steering->setAngle(_reverseSteerAngle);
            break;

        case VehicleState::REVERSING:
            // Lenkwinkel bereits gesetzt in WAIT_BEFORE_REVERSE –
            // hier nochmals setzen falls direkter Übergang ohne Wartezeit
            _reverseCount++;
            if (_reverseCount % 3 == 0) {
                _reverseSteerAngle = -_reverseSteerAngle;
            }
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