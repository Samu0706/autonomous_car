#include "RotaryEncoderController.h"

// globale ISR Weiterleitung (Library benötigt das)
static AiEsp32RotaryEncoder* globalEncoder = nullptr;

void IRAM_ATTR RotaryEncoderController::readEncoderISR() {
    if (globalEncoder) {
        globalEncoder->readEncoder_ISR();
    }
}

void RotaryEncoderController::begin(const RotaryConfig& cfg) {
    _cfg = cfg;

    _encoder = new AiEsp32RotaryEncoder(
        _cfg.pinCLK,
        _cfg.pinDT,
        _cfg.pinSW,
        -1  // kein separater VCC Pin
    );

    globalEncoder = _encoder;

    _encoder->begin();
    _encoder->setup(readEncoderISR);

    _encoder->setBoundaries(_cfg.minValue, _cfg.maxValue, false);
    _encoder->setAcceleration(0);
    

    pinMode(_cfg.pinSW, INPUT_PULLUP);

    _value = 0;
    _lastValue = 0;
}

// ----------------------------
// UPDATE
// ----------------------------
void RotaryEncoderController::update() {

    _event = EncoderEvent::NONE;

    // Rotation erkennen
    if (_encoder->encoderChanged()) {

        int raw = _encoder->readEncoder();
        _value = -raw;

        if (_value > _lastValue) {
            _event = EncoderEvent::ROTATE_RIGHT;
        }
        else if (_value < _lastValue) {
            _event = EncoderEvent::ROTATE_LEFT;
        }

        _lastValue = _value;
    }

    // Button handling
    bool buttonState = digitalRead(_cfg.pinSW);

    // gedrückt (LOW)
    if (buttonState == LOW && _buttonLastState == HIGH) {
        _buttonPressTime = millis();
    }

    // losgelassen
    if (buttonState == HIGH && _buttonLastState == LOW) {

        unsigned long pressDuration = millis() - _buttonPressTime;

        if (pressDuration > _cfg.longPressTime_ms) {
            _event = EncoderEvent::BUTTON_LONG_PRESS;
        } else {
            _event = EncoderEvent::BUTTON_CLICK;
        }
    }

    _buttonLastState = buttonState;
}

// ----------------------------
// EVENT AUSLESEN
// ----------------------------
EncoderEvent RotaryEncoderController::getEvent() {
    EncoderEvent e = _event;
    _event = EncoderEvent::NONE; // einmalig konsumieren
    return e;
}