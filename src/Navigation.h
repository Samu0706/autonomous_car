#pragma once
#include "FollowTheGap.h"
#include "SteeringController.h"
#include "VehicleStateMachine.h"
#include "LidarSensor.h"

/**
 * Verknüpft FollowTheGap mit SteeringController.
 * Aktiv nur wenn VSM im Zustand DRIVING ist.
 * In REVERSING / EMERGENCY / etc. übernimmt die VSM die Lenkung selbst.
 */
class Navigation {
public:
    void begin(FollowTheGap* fgm,
               SteeringController* str,
               VehicleStateMachine* vsm);

    // Nach ObstacleDetection aufrufen
    void update(const LidarPoint* points, int count);

    float getCurrentAngle() const;

private:
    FollowTheGap*        _fgm = nullptr;
    SteeringController*  _str = nullptr;
    VehicleStateMachine* _vsm = nullptr;
};