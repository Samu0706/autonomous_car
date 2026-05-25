#include "DisplayUI.h"
#include <math.h>

// =============================
// INIT
// =============================
void DisplayUI::begin(DisplayController* d,
                      VehicleStateMachine* vsm,
                      SpeedController* spd,
                      SteeringController* str,
                      Navigation* nav,
                      LidarProcessing* lidarProc,
                      BluetoothRemote* bt) {
    _display   = d;
    _vsm       = vsm;
    _speed     = spd;
    _steering  = str;
    _nav       = nav;
    _lidarProc = lidarProc;
    _bt        = bt;
}

// =============================
// CLICK
// =============================
void DisplayUI::handleClick() {
    if (_driveActive) {
        _driveActive = false;
        _driveView   = DriveView::COCKPIT_DRIVE;
        _vsm->stop();
        _speed->stop();
        _screen = UIScreen::COCKPIT;
        return;
    }
    _screen = (UIScreen)(((int)_screen + 1) % 4);
}

// =============================
// HOLD
// =============================
void DisplayUI::handleHold() {
    if (_driveActive) {
        _driveActive = false;
        _driveView   = DriveView::COCKPIT_DRIVE;
        _vsm->stop();
        _speed->stop();
        _screen = UIScreen::COCKPIT;
        return;
    }
    _vsm->start();
    _speed->forward();
    _driveActive = true;
    _driveView   = DriveView::COCKPIT_DRIVE;
}

// =============================
// ROTARY ENCODER
// =============================
void DisplayUI::handleRotate(int dir) {
    if (_driveActive) {
        int v = (int)_driveView + dir;
        if (v < 0) v = 2;
        if (v > 2) v = 0;
        _driveView = (DriveView)v;
        return;
    }
    if (_screen == UIScreen::MAP) {
        _scale += dir * 0.005f;
        _scale = constrain(_scale, 0.01f, 0.12f);
    }
    if (_screen == UIScreen::SETUP) {
        _speedValue += dir;
        _speedValue = constrain(_speedValue, 0, 255);
        _speed->setForwardSpeed(_speedValue);
    }
}

// =============================
void DisplayUI::setDriveActive(bool active) {
    _driveActive = active;
    if (!active) {
        _driveView = DriveView::COCKPIT_DRIVE;
        _screen    = UIScreen::COCKPIT;
    }
}

// =============================
void DisplayUI::update(unsigned long loopTime) {
    _loopTime = loopTime;
}

// =============================
// MAIN DRAW ROUTER
// =============================
void DisplayUI::draw(const LidarPoint* points, int count) {
    // I²C sendBuffer() blockiert ~25 ms – auf max. 10 Hz begrenzen.
    static unsigned long lastDrawMs = 0;
    unsigned long now = millis();
    if (now - lastDrawMs < 100) return;
    lastDrawMs = now;

    if (_driveActive) {
        switch (_driveView) {
            case DriveView::COCKPIT_DRIVE: drawDrive();            break;
            case DriveView::MAP:           drawMap(points, count); break;
            case DriveView::DEBUG:         drawDebug(count);       break;
        }
        return;
    }
    switch (_screen) {
        case UIScreen::COCKPIT: drawCockpit();           break;
        case UIScreen::MAP:     drawMap(points, count);  break;
        case UIScreen::DEBUG:   drawDebug(count);        break;
        case UIScreen::SETUP:   drawSetup();             break;
    }
}

// =============================
// COCKPIT  –  128x64  (IDLE)
// =============================
void DisplayUI::drawCockpit() {
    _display->clear();

    // Aufwärmphase: Fortschritt des LIDAR-Referenzscans anzeigen
    if (_lidarProc && !_lidarProc->isReady()) {
        int cov = _lidarProc->getScanCoverage();

        _display->printAt(0,  0,  " LIDAR  INIT... ");
        _display->printAt(0,  12, "================");

        String covStr = "Scan: " + String(cov) + " / 360";
        _display->printAt(0, 24, covStr);

        // Fortschrittsbalken: [################] = 14 Füllzeichen
        int filled = (int)((cov / 360.0f) * 14);
        if (filled > 14) filled = 14;
        String bar = "[";
        for (int i = 0; i < 14; i++) bar += (i < filled) ? '#' : '-';
        bar += "]";
        _display->printAt(0, 37, bar);

        _display->printAt(0, 52, "Bitte warten...");
        _display->update();
        return;
    }

    // Normalbetrieb: System bereit
    _display->printAt(0,  0,  "==================");
    _display->printAt(4,  10, "  SYSTEM  READY");
    _display->printAt(0,  20, "==================");

    _display->printAt(0,  35, "Hold  = START");
    _display->printAt(0,  45, "Click = Screens");
    _display->printAt(0,  55, "IDLE");

    _display->update();
}

// =============================
// DRIVE  –  128x64
// =============================
void DisplayUI::drawDrive() {
    _display->clear();

    if (_vsm->getState() == VehicleState::EMERGENCY) {
        _display->printAt(0,  8,  "!! EMERGENCY !!");
        _display->printAt(0,  28, "STOPP aktiv");
        _display->printAt(0,  48, "Btn = Zurueck");
        _display->update();
        return;
    }

    _display->printAt(0,  0,  "================");
    _display->printAt(0,  10, " >> DRIVING >>  ");
    _display->printAt(0,  20, "================");

    _display->printAt(56, 32, ">");

    _display->printAt(0,  44, "----------------");

    String spdStr = "Spd:" + String(_speedValue);
    _display->printAt(0,  55, spdStr);
    _display->printAt(92, 55, "STOP");

    _display->update();
}

// =============================
// MAP  –  128x64
// =============================
void DisplayUI::drawMap(const LidarPoint* points, int count) {
    _display->clear();
    const int cx = 64;
    const int cy = 32;
    for (int i = 0; i < count; i++) {
        float rad = (points[i].angle + 90.0f) * DEG_TO_RAD;
        int x = cx + (int)(cos(rad) * points[i].distance * _scale);
        int y = cy + (int)(sin(rad) * points[i].distance * _scale);
        if (x >= 0 && x < 128 && y >= 0 && y < 64) {
            _display->printAt(x, y, ".");
        }
    }
    _display->printAt(cx, cy, "O");
    _display->update();
}

// =============================
// DEBUG  –  128x64
//
// Y= 0   "DEBUG"
// Y=14   "State:  XXXXXXX"
// Y=26   "Points: XXXXX"
// Y=38   "Angle:  +XX.X"
// Y=50   "BT: X  ok/--"
// =============================
void DisplayUI::drawDebug(int lidarPoints) {
    _display->clear();

    _display->printAt(0,  0,  "DEBUG");

    _display->printAt(0,  14, "State:");
    _display->printAt(50, 14, _vsm->getStateName());

    _display->printAt(0,  26, "Points:");
    _display->printAt(50, 26, String(lidarPoints));

    _display->printAt(0,  38, "Angle:");
    if (_nav) {
        _display->printAt(50, 38, String(_nav->getCurrentAngle(), 1));
    } else {
        _display->printAt(50, 38, "---");
    }

    _display->printAt(0,  50, "BT:");
    if (_bt) {
        String btStr = String(_bt->getLastCommand());
        btStr += _bt->isConnected() ? "  ok" : "  --";
        _display->printAt(28, 50, btStr);
    } else {
        _display->printAt(28, 50, "-  --");
    }

    _display->update();
}

// =============================
// SETUP  –  128x64
// =============================
void DisplayUI::drawSetup() {
    _display->clear();

    _display->printAt(0,  0,  "SETUP");
    _display->printAt(0,  20, "Speed:");
    _display->printAt(55, 20, String(_speedValue));
    _display->printAt(0,  40, "Enc  = Wert");
    _display->printAt(0,  52, "Hold = Param");

    _display->update();
}