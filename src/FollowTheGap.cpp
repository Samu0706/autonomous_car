#include "FollowTheGap.h"
#include <math.h>

// Auf 1 setzen um FGM-Lenkwinkel/Gap-Details seriell auszugeben.
// Im Fahrbetrieb auf 0 lassen – Serial.print blockiert den Loop bei vollem TX-Puffer.
#define FGM_DEBUG 0

// =============================
// BEGIN
// =============================
void FollowTheGap::begin(const FGMConfig& cfg) {
    _cfg = cfg;
}

// =============================
// PROCESS – Hauptaufruf
// =============================
bool FollowTheGap::process(const LidarPoint* points, int count) {

    _validGap    = false;
    _gapCount    = 0;
    _bestGapIdx  = -1;

    _frontCount = extractFrontPoints(points, count);

#if FGM_DEBUG
    Serial.print("[FGM] front="); Serial.print(_frontCount);
#endif

    if (_frontCount < _cfg.minGapSize) {
#if FGM_DEBUG
        Serial.println(" → zu wenig Punkte");
#endif
        return false;
    }

    if (_cfg.disparityEnabled)  applyDisparityExtender();
    if (_cfg.nearFieldEnabled)  applyNearFieldExtender();

    _gapCount = findGaps();

#if FGM_DEBUG
    Serial.print(" gaps="); Serial.print(_gapCount);
#endif

    if (_gapCount == 0) {
#if FGM_DEBUG
        Serial.println(" → keine Lücke, dmin zu hoch?");
#endif
        _steeringAngle = 0.0f;
        return false;
    }

    int bestIdx = selectBestGap(_gapCount);
    if (bestIdx < 0) {
#if FGM_DEBUG
        Serial.println(" → keine beste Lücke");
#endif
        return false;
    }

    float targetLidarAngle = computeTargetAngle(_gaps[bestIdx]);
    _steeringAngle = lidarAngleToSteering(targetLidarAngle);
    _validGap      = true;

#if FGM_DEBUG
    Serial.print(" bestSize="); Serial.print(_gaps[bestIdx].size);
    Serial.print(" depth=");    Serial.print(_gaps[bestIdx].meanDepth);
    Serial.print(" → angle=");  Serial.println(_steeringAngle);
#endif

    return true;
}

// =============================
// 1. FRONTPUNKTE EXTRAHIEREN
// Nur Punkte im Frontsektor (90°–270°), sortiert nach Winkel
// =============================
int FollowTheGap::extractFrontPoints(const LidarPoint* points, int count) {

    int n = 0;

    for (int i = 0; i < count && n < MAX_FRONT_POINTS; i++) {

        float a = points[i].angle;

        // Frontsektor: 90° ≤ a ≤ 270°
        if (a >= _cfg.frontSectorLeft && a <= _cfg.frontSectorRight) {
            _frontPoints[n].angle    = a;
            _frontPoints[n].distance = points[i].distance;
            n++;
        }
    }

    // Nach Winkel sortieren (Insertion Sort – klein und stabil)
    for (int i = 1; i < n; i++) {
        LidarPoint key = _frontPoints[i];
        int j = i - 1;
        while (j >= 0 && _frontPoints[j].angle > key.angle) {
            _frontPoints[j + 1] = _frontPoints[j];
            j--;
        }
        _frontPoints[j + 1] = key;
    }

    return n;
}

// =============================
// 1b. DISPARITY EXTENDER
// Bei jedem Tiefensprung zwischen benachbarten Punkten (Disparity)
// wird die nähere Seite künstlich aufgebläht, sodass das Auto einen
// Sicherheitsabstand zu Hindernis-Kanten hält.
//
// Read/Write getrennt um Kaskaden-Effekte zu vermeiden:
// Disparities werden auf einem Snapshot (srcDist) erkannt, geschrieben
// wird ins _frontPoints-Array.
// =============================
void FollowTheGap::applyDisparityExtender() {

    if (_frontCount < 2) return;

    // Snapshot der Original-Distanzen (Winkel bleiben unverändert)
    // Bei MAX_FRONT_POINTS=400 sind das 1600 Bytes auf dem Stack — ok für ESP32.
    float srcDist[MAX_FRONT_POINTS];
    for (int i = 0; i < _frontCount; i++) {
        srcDist[i] = _frontPoints[i].distance;
    }

    for (int i = 0; i < _frontCount - 1; i++) {

        float da = srcDist[i];
        float db = srcDist[i + 1];
        if (da <= 0.0f || db <= 0.0f) continue;

        float diff = db - da;
        float absDiff = (diff < 0.0f) ? -diff : diff;
        if (absDiff < _cfg.disparityThreshold) continue;

        // Nähere Seite bestimmen
        int   nearerIdx = (diff > 0.0f) ? i : i + 1;
        float nearerD   = srcDist[nearerIdx];
        if (nearerD < 100.0f)                    continue;  // zu nah – Schutz
        if (nearerD > _cfg.disparityMaxDist)     continue;  // zu weit – nicht navigationsrelevant

        // Angularer Halbbereich, den wir aufblähen
        // halfAngle = asin( (inflate/2) / nearerD )
        float ratio = (_cfg.disparityInflate * 0.5f) / nearerD;
        if (ratio > 0.99f) ratio = 0.99f;
        float halfAngleDeg = asinf(ratio) * 180.0f / (float)M_PI;

        // Richtung: weg von der ferneren Seite
        int dir = (diff > 0.0f) ? +1 : -1;
        float baseAngle = _frontPoints[nearerIdx].angle;

        // Nachbarn in diese Richtung auf nearerD "schrumpfen" (nie vergrößern)
        int j = nearerIdx;
        while (j >= 0 && j < _frontCount) {
            float delta = _frontPoints[j].angle - baseAngle;
            if (delta < 0.0f) delta = -delta;
            if (delta > halfAngleDeg) break;

            if (_frontPoints[j].distance > nearerD) {
                _frontPoints[j].distance = nearerD;
            }
            j += dir;
        }
    }
}

// =============================
// 1c. NAHBEREICH-EXTENDER
// Für jeden Punkt näher als nearFieldThreshold wird ein Winkelbereich
// (Blasendurchmesser nearFieldInflate) auf seine Distanz herabgesetzt.
// Erkennung auf Snapshot (srcDist), Schreiben in _frontPoints – kein Kaskaden-Effekt.
// =============================
void FollowTheGap::applyNearFieldExtender() {

    if (_frontCount < 1) return;

    float srcDist[MAX_FRONT_POINTS];
    for (int i = 0; i < _frontCount; i++) {
        srcDist[i] = _frontPoints[i].distance;
    }

    for (int i = 0; i < _frontCount; i++) {
        float d = srcDist[i];
        if (d <= 0.0f || d > _cfg.nearFieldThreshold) continue;

        float ratio = (_cfg.nearFieldInflate * 0.5f) / d;
        if (ratio > 0.99f) ratio = 0.99f;
        float halfAngleDeg = asinf(ratio) * 180.0f / (float)M_PI;
        float baseAngle = _frontPoints[i].angle;

        for (int j = 0; j < _frontCount; j++) {
            float delta = _frontPoints[j].angle - baseAngle;
            if (delta < 0.0f) delta = -delta;
            if (delta > halfAngleDeg) continue;
            if (_frontPoints[j].distance > d) {
                _frontPoints[j].distance = d;
            }
        }
    }
}

// =============================
// 2. LÜCKEN FINDEN
// Zusammenhängende Punkte mit distance > dmin
// =============================
int FollowTheGap::findGaps() {

    int  gapCount   = 0;
    int  gapStart   = -1;
    bool inGap      = false;

    for (int i = 0; i < _frontCount; i++) {

        // Frei = Distanz über dmin UND kein farDistance-Fallback (kein Echosignal).
        // Ohne die farDistance-Prüfung würden d=0-Ersatzwerte (9000 mm) als tiefer
        // Freiraum gewertet und phantom Lücken erzeugen.
        float d      = _frontPoints[i].distance;
        bool isFree  = (d > _cfg.dmin) && (d < _cfg.farDistance - 100.0f);

        if (isFree && !inGap) {
            // Lücke beginnt
            gapStart = i;
            inGap    = true;
        }
        else if (!isFree && inGap) {
            // Lücke endet
            int size = i - gapStart;
            if (size >= _cfg.minGapSize && gapCount < MAX_GAPS) {
                _gaps[gapCount].startIdx = gapStart;
                _gaps[gapCount].endIdx   = i - 1;
                _gaps[gapCount].size     = size;
                gapCount++;
            }
            inGap = false;
        }
    }

    // Letzte Lücke abschließen falls noch offen
    if (inGap) {
        int size = _frontCount - gapStart;
        if (size >= _cfg.minGapSize && gapCount < MAX_GAPS) {
            _gaps[gapCount].startIdx = gapStart;
            _gaps[gapCount].endIdx   = _frontCount - 1;
            _gaps[gapCount].size     = size;
            gapCount++;
        }
    }

    // Mittlere Tiefe und gewichteten Mittelwinkel für jede Lücke berechnen
    for (int g = 0; g < gapCount; g++) {
        float dSum = 0.0f;
        float wSum = 0.0f;   // Σ(angle * distance) für gewichteten Mittelwinkel
        int   n    = 0;
        for (int i = _gaps[g].startIdx; i <= _gaps[g].endIdx; i++) {
            float d = _frontPoints[i].distance;
            float a = _frontPoints[i].angle;
            dSum += d;
            wSum += a * d;
            n++;
        }
        _gaps[g].meanDepth   = (n > 0) ? (dSum / n) : 0.0f;
        _gaps[g].centerAngle = (dSum > 0.0f)
            ? (wSum / dSum)
            : (_frontPoints[_gaps[g].startIdx].angle + _frontPoints[_gaps[g].endIdx].angle) * 0.5f;
    }

    return gapCount;
}

// =============================
// 3. BESTE LÜCKE AUSWÄHLEN
// Score = alpha * normSize + beta * normDepth + Richtungsbonus
// =============================
int FollowTheGap::selectBestGap(int gapCount) {

    int   bestIdx   = -1;
    float bestScore = -1.0f;

    const float sMin = 180.0f - _bias.straightHalfWidth;
    const float sMax = 180.0f + _bias.straightHalfWidth;

    for (int g = 0; g < gapCount; g++) {
        float normSize  = (float)_gaps[g].size / _frontCount;
        float normDepth = _gaps[g].meanDepth / 6000.0f;
        if (normDepth > 1.0f) normDepth = 1.0f;  // YDLIDAR gibt bis 8000mm zurück → ohne Clamp score > 1
        float baseScore = _cfg.alpha * normSize + _cfg.beta * normDepth;

        // Richtungsbonus: welcher Zone gehört der Lücken-Mittelpunkt an?
        float biasBonus = 0.0f;
        float ca = _gaps[g].centerAngle;
        if (ca >= sMin && ca <= sMax) biasBonus = _bias.straight;
        else if (ca > sMax)           biasBonus = _bias.right;
        else                          biasBonus = _bias.left;

        _gaps[g].score = baseScore + biasBonus;

        if (_gaps[g].score > bestScore) {
            bestScore = _gaps[g].score;
            bestIdx   = g;
        }
    }

    _bestGapIdx = bestIdx;
    return bestIdx;
}

// =============================
// GAP INFO GETTER
// =============================
bool FollowTheGap::getGapInfo(int idx, GapInfo& out) const {
    if (idx < 0 || idx >= _gapCount) return false;
    out.startAngle  = _frontPoints[_gaps[idx].startIdx].angle;
    out.endAngle    = _frontPoints[_gaps[idx].endIdx].angle;
    out.centerAngle = _gaps[idx].centerAngle;
    out.meanDepth   = _gaps[idx].meanDepth;
    out.score       = _gaps[idx].score;
    out.isBest      = (idx == _bestGapIdx);
    return true;
}

// =============================
// 4. ZIELWINKEL AUS LÜCKE
// Gewichteter Mittelwert: Σ(angle·dist) / Σ(dist)
// → tiefere Punkte haben mehr Gewicht (sicherer Weg)
// =============================
float FollowTheGap::computeTargetAngle(const Gap& gap) {

    float weightedAngleSum = 0.0f;
    float distSum          = 0.0f;

    for (int i = gap.startIdx; i <= gap.endIdx; i++) {
        float a = _frontPoints[i].angle;
        float d = _frontPoints[i].distance;
        weightedAngleSum += a * d;
        distSum          += d;
    }

    if (distSum <= 0.0f) {
        // Fallback: einfacher Mittelwert
        return (_frontPoints[gap.startIdx].angle +
                _frontPoints[gap.endIdx].angle) * 0.5f;
    }

    return weightedAngleSum / distSum;
}

// =============================
// 5. WINKEL-UMRECHNUNG
// LiDAR 180° = vorne  → Lenkwinkel   0°
// LiDAR < 180° = links → Lenkwinkel negativ
// LiDAR > 180° = rechts → Lenkwinkel positiv
// Mapping der LiDAR-Range (90°..270°) auf ±maxSteerAngle
// =============================
float FollowTheGap::lidarAngleToSteering(float lidarAngle) const {

    // Offset zu 180° (geradeaus): rechts (+), links (-)
    float delta = lidarAngle - 180.0f;

    // Linear auf ±maxSteerAngle mappen, dann steerGain anwenden.
    // Clamp auf ±maxSteerAngle bleibt – mechanischer Ausschlag wird nie überschritten.
    float steer = (delta / 90.0f) * _cfg.maxSteerAngle * _cfg.steerGain;

    if (steer >  _cfg.maxSteerAngle) steer =  _cfg.maxSteerAngle;
    if (steer < -_cfg.maxSteerAngle) steer = -_cfg.maxSteerAngle;

    return steer;
}