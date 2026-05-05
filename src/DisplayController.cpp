#include "DisplayController.h"

void DisplayController::begin(const DisplayConfig& cfg) {
    _cfg = cfg;

    // I2C Pins setzen (ESP32-S3!)
    Wire.begin(_cfg.sdaPin, _cfg.sclPin);

    // Häufig korrekt für SBC-OLED01
    _u8g2 = new U8G2_SH1106_128X64_NONAME_F_HW_I2C(_cfg.rotation, U8X8_PIN_NONE);

    // Adresse testen
    if (!tryInit(_cfg.address1)) {
        if (!tryInit(_cfg.address2)) {
            Serial.println("Display nicht gefunden!");
            return;
        }
    }

    _u8g2->clearBuffer();
    _u8g2->setFont(u8g2_font_6x10_tf);
    _u8g2->sendBuffer();

    Serial.printf("Display init @ 0x%X\n", _activeAddress);
}

bool DisplayController::tryInit(uint8_t address) {
    _u8g2->setI2CAddress(address << 1);  // U8g2 erwartet 8-bit

    _u8g2->begin();

    // kein echtes Feedback möglich → wir akzeptieren Init
    _activeAddress = address;
    return true;
}

// ---- Public API ----

void DisplayController::clear() {
    if (!_u8g2) return;
    _u8g2->clearBuffer();
}

void DisplayController::update() {
    if (!_u8g2) return;
    _u8g2->sendBuffer();
}

void DisplayController::setCursor(int x, int y) {
    if (!_u8g2) return;
    _u8g2->setCursor(x, y);
}

void DisplayController::print(const String& text) {
    if (!_u8g2) return;
    _u8g2->print(text);
}

void DisplayController::println(const String& text) {
    if (!_u8g2) return;
    _u8g2->print(text);
    _u8g2->print("\n");
}

void DisplayController::printAt(int x, int y, const String& text) {
    if (!_u8g2) return;
    _u8g2->setCursor(x, y);
    _u8g2->print(text);
}