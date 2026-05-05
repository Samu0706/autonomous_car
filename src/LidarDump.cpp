#include "LidarDump.h"
#include <Arduino.h>

void LidarDump::checkSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'd') {
            _active = !_active;
            Serial.println(_active ? "[DUMP] ON – Sende Roh-Scans" : "[DUMP] OFF");
        }
    }
}

void LidarDump::dump(const LidarPoint* points, int count) {
    if (!_active) return;
    for (int i = 0; i < count; i++) {
        Serial.print(points[i].angle, 2);
        Serial.print(',');
        Serial.println(points[i].distance, 1);
    }
    Serial.println("END");
}
