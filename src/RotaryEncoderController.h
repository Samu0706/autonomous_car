#pragma once
#include <Arduino.h>
#include <AiEsp32RotaryEncoder.h>

/**
 * CONTROL LAYER – Rotary Encoder Controller
 *
 * Liest Drehencoder + Button (SW)
 *
 * Features:
 * - Positionsänderung (links/rechts)
 * - Button (kurz / lang)
 * - vorbereitet für UI-Steuerung
 */

enum class EncoderEvent {
    NONE,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    BUTTON_CLICK,
    BUTTON_LONG_PRESS
};

struct RotaryConfig {
    uint8_t pinCLK = 12;
    uint8_t pinDT  = 13;
    uint8_t pinSW  = 14;

    int minValue = -100;
    int maxValue = 100;
    int step     = 1;

    uint16_t longPressTime_ms = 800;
};

class RotaryEncoderController {
public:
    void begin(const RotaryConfig& cfg);

    void update();  // muss regelmäßig im loop() aufgerufen werden

    EncoderEvent getEvent();
    int getValue() const { return _value; }

private:
    static void IRAM_ATTR readEncoderISR();

    RotaryConfig _cfg;

    AiEsp32RotaryEncoder* _encoder = nullptr;

    int _value = 0;
    int _lastValue = 0;

    bool _buttonLastState = HIGH;
    unsigned long _buttonPressTime = 0;

    EncoderEvent _event = EncoderEvent::NONE;
};