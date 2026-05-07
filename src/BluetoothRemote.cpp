#include "BluetoothRemote.h"
#include "DisplayUI.h"
#include <string.h>
#include <stdlib.h>

#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// =============================
// BEGIN
// =============================
void BluetoothRemote::begin(FollowTheGap*        fgm,
                             VehicleStateMachine* vsm,
                             SpeedController*     spd,
                             DisplayUI*           ui,
                             const char*          deviceName) {
    _fgm = fgm;
    _vsm = vsm;
    _spd = spd;
    _ui  = ui;

    NimBLEDevice::init(deviceName);

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(this);

    NimBLEService* svc = server->createService(NUS_SERVICE_UUID);

    _txChar = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);

    svc->createCharacteristic(NUS_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    )->setCallbacks(this);

    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE_UUID);
    adv->start();

    Serial.print("[BLE] advertising as: ");
    Serial.println(deviceName);
}

// =============================
// UPDATE  –  Arduino-Loop-Kontext
// =============================
void BluetoothRemote::update() {
    if (!_cmdReady) return;
    _cmdReady = false;
    handleCommand(_cmdBuf);
}

// =============================
// SEND TELEMETRY
// Rate-limitiert auf 200 ms; sendet JSON-Notification
// =============================
void BluetoothRemote::sendTelemetry(float steerAngle, const char* state,
                                     int rawPts, int filtPts) {
    if (!_connected || !_txChar) return;

    unsigned long now = millis();
    if (now - _lastTxMs < 200) return;
    _lastTxMs = now;

    char buf[512];
    int n = snprintf(buf, sizeof(buf),
        "{\"a\":%.1f,\"s\":\"%s\",\"rp\":%d,\"fp\":%d,\"g\":[",
        steerAngle, state, rawPts, filtPts);

    // Gap-Daten anhängen (max 8 Lücken, jede als [sa,ea,ca,dcm,sc100])
    if (_fgm) {
        int gapCount = _fgm->getGapCount();
        int limit    = gapCount < 8 ? gapCount : 8;
        bool first   = true;
        for (int i = 0; i < limit && n < (int)sizeof(buf) - 40; i++) {
            FollowTheGap::GapInfo gi;
            if (!_fgm->getGapInfo(i, gi)) continue;
            if (!first) buf[n++] = ',';
            first = false;
            n += snprintf(buf + n, sizeof(buf) - n,
                "[%d,%d,%d,%d,%d,%d]",
                (int)gi.startAngle,
                (int)gi.endAngle,
                (int)gi.centerAngle,
                (int)(gi.meanDepth / 10.0f),   // mm → cm
                (int)(gi.score * 100.0f),        // 0.0–1.0 → 0–100
                gi.isBest ? 1 : 0);
        }
    }

    if (n < (int)sizeof(buf) - 2) {
        buf[n++] = ']';
        buf[n++] = '}';
        buf[n]   = '\0';
    }

    _txChar->setValue((uint8_t*)buf, (size_t)n);
    _txChar->notify();
}

// =============================
// ON WRITE  –  BLE-Task-Kontext
// Nur Puffern, keine Logik hier (Thread-Safety)
// =============================
void BluetoothRemote::onWrite(NimBLECharacteristic* pChar) {
    std::string val = pChar->getValue();
    size_t n = 0;
    for (size_t i = 0; i < val.size() && n < 62; i++) {
        char c = val[i];
        if (c != '\r' && c != '\n')
            _cmdBuf[n++] = (char)toupper((unsigned char)c);
    }
    _cmdBuf[n] = '\0';
    _cmdReady = true;
}

// =============================
// CONNECT / DISCONNECT
// =============================
void BluetoothRemote::onConnect(NimBLEServer*) {
    _connected = true;
    Serial.println("[BLE] client connected");
}

void BluetoothRemote::onDisconnect(NimBLEServer*) {
    _connected = false;
    Serial.println("[BLE] disconnected – restarting advertising");
    NimBLEDevice::startAdvertising();
}

// =============================
// HANDLE COMMAND  –  Loop-Kontext
// Unterstützt Einzelzeichen (L/R/F/C/S/X) und Key=Value-Paare
// =============================
void BluetoothRemote::handleCommand(const char* cmd) {

    const char* eq = strchr(cmd, '=');

    // ── Key=Value-Parameter ──────────────────────────────────────
    if (eq) {
        char key[16] = {};
        size_t kLen = (size_t)(eq - cmd);
        if (kLen == 0 || kLen >= sizeof(key)) return;
        memcpy(key, cmd, kLen);

        float  fval = atof(eq + 1);
        int    ival = atoi(eq + 1);

        Serial.print("[BLE] param: "); Serial.print(key);
        Serial.print("="); Serial.println(eq + 1);

        if (strcmp(key, "SPD") == 0) {
            int v = constrain(ival, 0, 255);
            _spd->setForwardSpeed((uint8_t)v);

        } else if (strcmp(key, "DMIN") == 0) {
            FGMConfig cfg = _fgm->getConfig();
            cfg.dmin = fval;
            _fgm->setConfig(cfg);

        } else if (strcmp(key, "ALPHA") == 0) {
            FGMConfig cfg = _fgm->getConfig();
            cfg.alpha = constrain(fval, 0.0f, 1.0f);
            _fgm->setConfig(cfg);

        } else if (strcmp(key, "BETA") == 0) {
            FGMConfig cfg = _fgm->getConfig();
            cfg.beta = constrain(fval, 0.0f, 1.0f);
            _fgm->setConfig(cfg);

        } else if (strcmp(key, "MGAP") == 0) {
            FGMConfig cfg = _fgm->getConfig();
            cfg.minGapSize = constrain(ival, 1, 20);
            _fgm->setConfig(cfg);

        } else if (strcmp(key, "BSTR") == 0) {
            _biasStrength = constrain(fval, 0.0f, 1.0f);

        } else if (strcmp(key, "L") == 0) {
            DirectionBias bias;
            bias.left = constrain(fval, 0.0f, 1.0f);
            _fgm->setDirectionBias(bias);
            _biasStrength = bias.left;

        } else if (strcmp(key, "R") == 0) {
            DirectionBias bias;
            bias.right = constrain(fval, 0.0f, 1.0f);
            _fgm->setDirectionBias(bias);
            _biasStrength = bias.right;

        } else if (strcmp(key, "F") == 0) {
            DirectionBias bias;
            bias.straight = constrain(fval, 0.0f, 1.0f);
            _fgm->setDirectionBias(bias);
            _biasStrength = bias.straight;
        }
        return;
    }

    // ── Einzelzeichen-Befehle ────────────────────────────────────
    if (cmd[0] == '\0' || cmd[1] != '\0') return;  // leer oder mehrteilig ohne '='
    char c = cmd[0];
    _lastCmd = c;

    Serial.print("[BLE] cmd: "); Serial.println(c);

    DirectionBias bias;
    switch (c) {
        case 'L':
            bias.left = _biasStrength;
            _fgm->setDirectionBias(bias);
            break;
        case 'R':
            bias.right = _biasStrength;
            _fgm->setDirectionBias(bias);
            break;
        case 'F':
            bias.straight = _biasStrength;
            _fgm->setDirectionBias(bias);
            break;
        case 'C':
            _fgm->clearDirectionBias();
            break;
        case 'S':
            _vsm->start();
            _spd->forward();
            if (_ui) _ui->setDriveActive(true);
            break;
        case 'X':
            _vsm->stop();
            _spd->stop();
            if (_ui) _ui->setDriveActive(false);
            break;
        default:
            break;
    }
}
