#include "LidarSensor.h"

LidarSensor* LidarSensor::_instance = nullptr;

void LidarSensor::begin(const LidarConfig& cfg) {
    _cfg = cfg;
    _instance = this;

    _points = new LidarPoint[_cfg.maxPoints];

    uint32_t baud = _lidar.getSerialBaudRate();
    _serial.begin(baud, SERIAL_8N1, _cfg.rxPin);

    _lidar.setScanPointCallback(scanCallback);
    _lidar.setSerialReadCallback(serialReadCallback);

    _lidar.init();
    _lidar.start();
}

void LidarSensor::update() {
    _lidar.loop();
}

int LidarSensor::serialReadCallback() {
    if (_instance)
        return _instance->_serial.read();
    return -1;
}

void LidarSensor::scanCallback(float angle, float distance, float quality, bool scan_completed) {
    if (_instance)
        _instance->processPoint(angle, distance, scan_completed);
}

void LidarSensor::processPoint(float angle, float distance, bool scan_completed) {

    // Punkt speichern – aber NICHT nach scan_completed (nächster Scan läuft sonst rein)
    if (!_scanReady && _pointCount < _cfg.maxPoints) {
        _points[_pointCount].angle = angle;
        _points[_pointCount].distance = distance;
        _pointCount++;
    }

    // Scan fertig
    if (scan_completed) {
        _scanReady = true;
    }
}

// ---------------- GETTER ----------------

bool LidarSensor::isScanReady() const {
    return _scanReady;
}

int LidarSensor::getPointCount() const {
    return _pointCount;
}

const LidarPoint* LidarSensor::getPoints() const {
    return _points;
}

void LidarSensor::clearScan() {
    _pointCount = 0;
    _scanReady = false;
}