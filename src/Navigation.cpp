#include "Navigation.h"

// =============================
// BEGIN
// =============================
void Navigation::begin(FollowTheGap* fgm,
                       SteeringController* str,
                       VehicleStateMachine* vsm) {
    _fgm = fgm;
    _str = str;
    _vsm = vsm;
}

// =============================
// UPDATE
// =============================
void Navigation::update(const LidarPoint* points, int count) {

    // Nur in DRIVING aktiv – in REVERSING / EMERGENCY / WAIT_*
    // setzt die VSM die Lenkung selbst in transitionTo()
    if (_vsm->getState() != VehicleState::DRIVING) {
        return;
    }

    if (_fgm->process(points, count) && _fgm->hasValidGap()) {
        _str->setAngle(_fgm->getSteeringAngle());
    } else {
        // Keine Lücke erkannt → geradeaus (Sicherheitsverhalten)
        _str->setAngle(0.0f);
    }
}

// =============================
// CURRENT ANGLE (für Debug/Display)
// =============================
float Navigation::getCurrentAngle() const {
    return _fgm ? _fgm->getSteeringAngle() : 0.0f;
}