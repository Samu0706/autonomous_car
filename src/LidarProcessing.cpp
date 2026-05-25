#include "LidarProcessing.h"
#include <string.h>   // memset, memcpy
#include <algorithm>  // std::sort für Median (nicht mehr genutzt, aber Kompatibilität)

// =============================
// BEGIN
// =============================
void LidarProcessing::begin(const LidarProcessingConfig& cfg, int maxPoints) {
    _cfg       = cfg;
    _maxPoints = maxPoints;

    // _filtered dient zuerst als Rohdaten-Puffer für applyRangeFilter (maxPoints),
    // danach wird er in buildOutputScan() mit genau 360 Punkten überschrieben.
    // maxPoints muss >= 360 sein (in main.cpp: 800 → kein Problem).
    _filtered = new LidarPoint[maxPoints];
    _temp     = new LidarPoint[360];

    memset(_scanRef,   0, sizeof(_scanRef));
    memset(_scanWork,  0, sizeof(_scanWork));
    memset(_refValid,  0, sizeof(_refValid));
    memset(_workValid, 0, sizeof(_workValid));
    _hasRef   = false;
    _coverage = 0;
}

// =============================
// HAUPTAUFRUF
// =============================
int LidarProcessing::process(const LidarPoint* raw, int rawCount) {
    applyRangeFilter(raw, rawCount);  // Schritt 1: Rohdaten filtern → _filtered
    discretizeScan();                  // Schritt 2: float-Winkel → int-Grad
    updateReference();                 // Schritt 3: Referenzscan aufbauen/aktualisieren
    buildOutputScan();                 // Schritt 4: lückenlosen 360-Punkte-Scan ausgeben
    applyMedianFilter();               // Schritt 5: Glättung
    return _filteredCount;
}

// =============================
// SCHRITT 1 – RANGE FILTER
// Kopiert gültige Punkte in _filtered.
// d=0 (Out-of-Range beim YDLIDAR X2) wird optional als Freiraum behandelt.
// =============================
void LidarProcessing::applyRangeFilter(const LidarPoint* raw, int rawCount) {

    _filteredCount = 0;

    for (int i = 0; i < rawCount; i++) {

        float d = raw[i].distance;

        if (_cfg.treatZeroAsFar && d <= 0.5f) {
            d = _cfg.farDistance;
        }

        // OOR-Ersatzpunkte (d ≈ farDistance) dürfen den Range-Check überspringen,
        // auch wenn farDistance > maxDistance konfiguriert ist.
        bool isOORReplacement = (_cfg.treatZeroAsFar && d >= _cfg.farDistance - 0.5f);
        if (!isOORReplacement) {
            if (d < _cfg.minDistance || d > _cfg.maxDistance) continue;
        }

        float a = raw[i].angle;
        while (a <   0.0f) a += 360.0f;
        while (a >= 360.0f) a -= 360.0f;

        if (_filteredCount < _maxPoints) {
            _filtered[_filteredCount].angle    = a;
            _filtered[_filteredCount].distance = d;
            _filteredCount++;
        }
    }
}

// =============================
// SCHRITT 2 – DISKRETISIERUNG
// Mappt float-Winkel auf ganzzahlige Grad 0–359.
// Treffen mehrere Punkte auf denselben Grad, wird die kleinste Distanz
// gespeichert (nächstes Hindernis = konservativste Wahl für Sicherheit).
// =============================
void LidarProcessing::discretizeScan() {

    memset(_workValid, 0, sizeof(_workValid));
    // _scanWork muss nicht genullt werden: wird nur über _workValid[]==true gelesen,
    // und beim ersten Schreiben auf einen Slot direkt gesetzt.

    for (int i = 0; i < _filteredCount; i++) {

        int deg = (int)(_filtered[i].angle + 0.5f) % 360;

        float fd = _filtered[i].distance;
        if (fd < 0.0f)     fd = 0.0f;
        if (fd > 65535.0f) fd = 65535.0f;
        uint16_t d = (uint16_t)fd;

        if (!_workValid[deg]) {
            _scanWork[deg]  = d;
            _workValid[deg] = true;
        } else if (d < _scanWork[deg]) {
            _scanWork[deg] = d;
        }
    }

    _coverage = 0;
    for (int i = 0; i < 360; i++) {
        if (_workValid[i]) _coverage++;
    }
}

// =============================
// SCHRITT 3 – REFERENZSCAN AKTUALISIEREN
//
// Aufwärmphase (!_hasRef):
//   Akkumuliert Punkte aus mehreren Scans in _scanRef, bis minScanCoverage
//   erreicht ist. Nutzt Minimum (= konservativster je gemessener Wert pro Grad).
//
// Laufbetrieb (_hasRef == true):
//   Vollständiger Scan (>= minScanCoverage): _scanRef komplett ersetzen.
//   Lückenhafter Scan: nur vorhandene Grad-Slots in _scanRef aktualisieren,
//   fehlende Slots bleiben vom letzten vollständigen Scan erhalten.
// =============================
void LidarProcessing::updateReference() {

    if (!_hasRef) {
        // Aufwärmphase: gradweise akkumulieren
        for (int i = 0; i < 360; i++) {
            if (!_workValid[i]) continue;
            if (!_refValid[i]) {
                _scanRef[i]  = _scanWork[i];
                _refValid[i] = true;
            } else if (_scanWork[i] < _scanRef[i]) {
                _scanRef[i] = _scanWork[i];
            }
        }

        int refCoverage = 0;
        for (int i = 0; i < 360; i++) if (_refValid[i]) refCoverage++;

        if (refCoverage >= _cfg.minScanCoverage) {
            _hasRef = true;
            Serial.println("[LidarProc] Referenzscan bereit");
        }

    } else {
        if (_coverage >= _cfg.minScanCoverage) {
            // Vollständiger Scan → Referenz komplett ersetzen
            memcpy(_scanRef,  _scanWork,  sizeof(_scanRef));
            memcpy(_refValid, _workValid, sizeof(_refValid));
        } else {
            // Lückenhafter Scan → nur neue Punkte übernehmen
            for (int i = 0; i < 360; i++) {
                if (_workValid[i]) {
                    _scanRef[i]  = _scanWork[i];
                    _refValid[i] = true;
                }
            }
        }
    }
}

// =============================
// SCHRITT 4 – AUSGANGSSCAN AUFBAUEN
// Schreibt exakt 360 sortierte LidarPoints in _filtered.
// Priorität pro Grad: aktueller Scan > Referenz > farDistance (Freiraum).
// angle = Grad-Mitte (deg + 0.5°) für korrekte Winkel-Arithmetik in FGM.
// =============================
void LidarProcessing::buildOutputScan() {

    _filteredCount = 0;

    for (int deg = 0; deg < 360; deg++) {
        float dist;

        if (_workValid[deg]) {
            dist = (float)_scanWork[deg];
        } else if (_refValid[deg]) {
            // Fehlender Punkt aus Referenzscan ergänzt
            dist = (float)_scanRef[deg];
        } else {
            // Keinerlei Information vorhanden: Freiraum annehmen.
            // Tritt typischerweise nur in der Aufwärmphase auf.
            dist = _cfg.farDistance;
        }

        _filtered[_filteredCount].angle    = (float)deg + 0.5f;
        _filtered[_filteredCount].distance = dist;
        _filteredCount++;
    }
    // _filteredCount ist jetzt immer 360
}

// =============================
// SCHRITT 5 – GLEITENDER MEDIAN
// Arbeitet auf genau 360 gleichmäßig verteilten Punkten.
// Ring-Wrap ist trivial (Index 0 = 0°, Index 359 = 359°).
// =============================
void LidarProcessing::applyMedianFilter() {

    if (_filteredCount == 0) return;

    int w          = _cfg.medianWindow;
    int windowSize = 2 * w + 1;

    // Statischer Puffer – max. medianWindow=10 → 2*10+1=21.
    // Kein heap-alloc pro Scan → verhindert Fragmentierung / nullptr-Crash.
    static float window[21];

    for (int i = 0; i < _filteredCount; i++) {
        _temp[i] = _filtered[i];
    }

    for (int i = 0; i < _filteredCount; i++) {

        int count = 0;

        for (int j = -w; j <= w; j++) {
            int idx = i + j;
            if (idx < 0)               idx += _filteredCount;
            if (idx >= _filteredCount) idx -= _filteredCount;
            window[count++] = _filtered[idx].distance;
        }

        _temp[i].distance = median(window, count);
    }

    for (int i = 0; i < _filteredCount; i++) {
        _filtered[i] = _temp[i];
    }
}

// =============================
// HILFSFUNKTION – Median
// Insertion Sort – für kleine Fenster (max ~7 Elemente) schneller als std::sort
// =============================
float LidarProcessing::median(float* arr, int count) const {

    for (int i = 1; i < count; i++) {
        float key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }

    return arr[count / 2];
}
