#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

/**
 * CONTROL LAYER – Display Controller
 *
 * Steuert ein SBC-OLED01 (128x64 OLED) via I2C
 *
 * Features:
 * - Textausgabe
 * - Cursorsteuerung
 * - Buffer-basiertes Rendering
 */

struct DisplayConfig {
    uint8_t sdaPin = 4;
    uint8_t sclPin = 5;

    uint8_t address1 = 0x3C;
    uint8_t address2 = 0x3D;

    const u8g2_cb_t* rotation = U8G2_R0;  
};

class DisplayController {
public:
    void begin(const DisplayConfig& cfg);

    void clear();
    void update();

    void setCursor(int x, int y);

    void print(const String& text);
    void println(const String& text);

    void printAt(int x, int y, const String& text);

private:
    bool tryInit(uint8_t address);

    DisplayConfig _cfg;

    U8G2* _u8g2 = nullptr;
    uint8_t _activeAddress = 0;
};