#pragma once

#include <Arduino.h>
#include "DisplayController.h"
#include "LidarSensor.h"
#include "LidarProcessing.h"
#include "VehicleStateMachine.h"
#include "SpeedController.h"
#include "SteeringController.h"
#include "Navigation.h"
#include "BluetoothRemote.h"

enum class UIScreen {
    COCKPIT,
    MAP,
    DEBUG,
    SETUP
};

enum class SetupParam {
    SPEED
};

enum class DriveView { COCKPIT_DRIVE, MAP, DEBUG };

class DisplayUI {
public:
    void begin(DisplayController* d,
               VehicleStateMachine* vsm,
               SpeedController* spd,
               SteeringController* str,
               Navigation* nav,
               LidarProcessing* lidarProc,
               BluetoothRemote* bt);

    void handleClick();
    void handleHold();
    void handleRotate(int dir); // -1 / +1

    // Von außen aufrufbar (z.B. BluetoothRemote), um Fahrmodus zu steuern
    void setDriveActive(bool active);

    void update(unsigned long loopTime);

    void draw(const LidarPoint* points, int count);

private:

    // Screens
    void drawCockpit();
    void drawMap(const LidarPoint* points, int count);
    void drawDebug(int lidarPoints);
    void drawSetup();

    void filterLidar(const LidarPoint* points, int count);

    void applyStartStop();
    void drawDrive();

private:
    DisplayController*  _display   = nullptr;
    VehicleStateMachine* _vsm      = nullptr;
    SpeedController*    _speed     = nullptr;
    SteeringController* _steering  = nullptr;
    Navigation*         _nav       = nullptr;
    LidarProcessing*    _lidarProc = nullptr;
    BluetoothRemote*    _bt        = nullptr;

    UIScreen _screen = UIScreen::COCKPIT;

    bool _running = false;

    // Setup
    SetupParam _setupParam = SetupParam::SPEED;
    int _speedValue = 22;

    // Map
    float _scale = 0.04f;
    float _distSum[360];
    int _count[360];

    unsigned long _loopTime = 0;
    bool _driveActive = false;

    int _animFrame = 0;
    unsigned long _lastAnim = 0;
    DriveView     _driveView  = DriveView::COCKPIT_DRIVE;

};