#pragma once
#include <Arduino.h>
#include "SpeedController.h"
#include "SteeringController.h"

/**
 * CONTROL LAYER – Fahrzeug-Zustandsmaschine
 *
 * Zustände:
 *   IDLE       – Fahrzeug steht, wartet auf Start vom Display
 *   DRIVING    – Normalbetrieb vorwärts
 *   REVERSING  – Rückwärtsfahrt nach Hindernis vorne
 *   EMERGENCY  – Not-Stopp (Hindernis hinten beim Rückwärtsfahren)
 */
enum class VehicleState {
    IDLE,
    DRIVING,
    WAIT_BEFORE_REVERSE,
    REVERSING,
    WAIT_BEFORE_EMERGENCY,
    EMERGENCY,
};

struct StateMachineConfig {
    uint32_t reverseTime_ms          = 2000;    // Rückwärtsfahrtdauer
    uint32_t holdOffTime_ms          = 1500;   // Emergency Reset-Wartezeit
    uint32_t waitBeforeReverse_ms    = 2000;   // Warten vor Rückwärts
    uint32_t waitBeforeEmergency_ms  = 2000;   // Warten vor Emergency
};

class VehicleStateMachine {
public:
    // SpeedController muss vor start() übergeben werden
    void begin(SpeedController* spd, SteeringController* str);
    void configure(const StateMachineConfig& cfg) { _cfg = cfg; }

    // Vom Display aufgerufen
    void start();   // IDLE → DRIVING
    void stop();    // Jeder Zustand → IDLE

    // Einmal pro loop() aufrufen
    // obstacleAhead: Hindernis vorne | obstacleRear: Hindernis hinten
    void update(bool obstacleAhead, bool obstacleRear);

    VehicleState getState()           const { return _state; }
    const char*  getStateName()       const;

    bool shouldDriveForward()         const { return _state == VehicleState::DRIVING;   }
    bool shouldReverse()              const { return _state == VehicleState::REVERSING; }
    bool shouldStop()                 const { return _state == VehicleState::IDLE ||
                                                     _state == VehicleState::EMERGENCY; }

    // Lenkwinkel für Rückwärtsfahrt (wechselt mit jedem REVERSING)
    float getReverseSteerAngle()      const { return _reverseSteerAngle; }

private:
    void transitionTo(VehicleState newState);

    SpeedController*   _speed = nullptr;
    StateMachineConfig _cfg;
    VehicleState       _state = VehicleState::IDLE;
    SteeringController* _steering = nullptr;

    unsigned long _stateEnteredAt_ms  = 0;
    float         _reverseSteerAngle  = 20.0f;  // Lenkwinkel für aktuelle Rückwärtsfahrt
    float         _fallbackReverseDir = 1.0f;   // Alterniert wenn letzter Winkel ≈ 0
};