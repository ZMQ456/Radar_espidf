#include "sleep_analyzer.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

#include "radar_platform.h"

const float SleepAnalyzer::SLEEPINESS_THRESHOLD = 0.6f;
const float SleepAnalyzer::BASELINE_MOVEMENT_THRESHOLD = 0.2f;
const float SleepAnalyzer::BASELINE_HR_STABILITY_THRESHOLD = 5.0f;
const float SleepAnalyzer::BASELINE_RR_STABILITY_THRESHOLD = 2.0f;
const float SleepAnalyzer::EMA_ALPHA = 0.2f;
const float SleepAnalyzer::CONFIDENCE_MARGIN = 0.1f;
const float SleepAnalyzer::BASELINE_BETA = 0.01f;
const float SleepAnalyzer::HYSTERESIS_ENTER_DEEP = 0.7f;
const float SleepAnalyzer::HYSTERESIS_EXIT_DEEP = 0.5f;
const float SleepAnalyzer::HYSTERESIS_ENTER_REM = 0.6f;
const float SleepAnalyzer::HYSTERESIS_EXIT_REM = 0.4f;

static const char *const SLEEP_STATE_NAMES[] = {
    "no_person",
    "in_bed",
    "awake",
    "light_sleep",
    "deep_sleep",
    "rem_sleep",
    "out_of_bed",
    "getting_up",
    "session_end"
};

SleepAnalyzer::SleepAnalyzer()
{
    reset();
}

void SleepAnalyzer::reset()
{
    currentState = SLEEP_NO_PERSON;
    pendingState = SLEEP_NO_PERSON;
    memset(&stats, 0, sizeof(stats));
    memset(&score, 0, sizeof(score));
    memset(&cycle, 0, sizeof(cycle));

    stateEnterTime = radar_now_ms();
    pendingStateTime = 0;
    noPersonTimer = 0;
    sleepinessDuration = 0;
    awakeDuration = 0;
    deepSleepDuration = 0;
    lightSleepDuration = 0;
    remSleepDuration = 0;
    movementHighDuration = 0;
    gettingUpDuration = 0;
    deepStableDuration = 0;

    baselineHR = 70.0f;
    baselineRR = 16.0f;
    baselineCalibrated = false;
    baselineSampleCount = 0;
    baselineHRSum = 0.0f;
    baselineRRSum = 0.0f;
    lastBaselineHR = 0.0f;
    lastBaselineRR = 0.0f;
    hrStabilitySum = 0.0f;
    rrStabilitySum = 0.0f;
    stabilitySampleCount = 0;

    currentSleepiness = 0.0f;
    currentDeepScore = 0.0f;
    currentLightScore = 0.0f;
    currentAwakeScore = 0.0f;
    currentRemScore = 0.0f;
    lastRRValue = 0.0f;
    wasAsleep = false;
}

PresenceData SleepAnalyzer::evaluatePresence(const HeartRateData &hrData,
                                             const RespirationData &rrData,
                                             const BodyMovementData &movementData) const
{
    PresenceData p = {};
    p.distance = -1.0f;
    p.motionEnergy = movementData.isValid ? movementData.activityLevel : 0.0f;

    if (hrData.isValid || rrData.isValid) {
        p.isPresent = true;
        p.confidence = 0.9f;
    }

    if (movementData.isValid && movementData.movement > 5) {
        p.isPresent = true;
        p.confidence = std::max(p.confidence, 0.7f);
    }

    return p;
}

float SleepAnalyzer::sigmoid(float x) const
{
    return 1.0f / (1.0f + std::exp(-x));
}

float SleepAnalyzer::emaSmooth(float input, float last, float alpha) const
{
    return alpha * input + (1.0f - alpha) * last;
}

float SleepAnalyzer::updateBaseline(float current, float input, float beta) const
{
    return (1.0f - beta) * current + beta * input;
}

bool SleepAnalyzer::isBestScore(float scoreValue, float s2, float s3, float s4, float margin) const
{
    return (scoreValue > s2 + margin && scoreValue > s3 + margin && scoreValue > s4 + margin);
}

float SleepAnalyzer::normalizeHR(float hr) const
{
    if (!baselineCalibrated) return 0.0f;
    return constrain_value((hr - baselineHR) / 20.0f, -1.0f, 1.0f);
}

float SleepAnalyzer::normalizeRR(float rr) const
{
    if (!baselineCalibrated) return 0.0f;
    return constrain_value((rr - baselineRR) / 4.0f, -1.0f, 1.0f);
}

float SleepAnalyzer::normalizeHRV(float hrv) const
{
    return constrain_value(hrv / 50.0f, 0.0f, 1.0f);
}

float SleepAnalyzer::normalizeMovement(float movement) const
{
    return constrain_value(movement / 100.0f, 0.0f, 1.0f);
}

void SleepAnalyzer::calibrateBaseline(const HeartRateData &hrData,
                                      const RespirationData &rrData,
                                      const BodyMovementData &movementData)
{
    if (!hrData.isValid || !rrData.isValid) return;
    if (currentState != SLEEP_AWAKE && currentState != SLEEP_IN_BED) return;
    if (movementData.isValid && normalizeMovement(movementData.movement) > BASELINE_MOVEMENT_THRESHOLD) return;

    if (baselineCalibrated) {
        const float hrDiff = std::fabs(hrData.bpmSmoothed - lastBaselineHR);
        const float rrDiff = std::fabs(rrData.rateSmoothed - lastBaselineRR);
        hrStabilitySum += hrDiff;
        rrStabilitySum += rrDiff;
        stabilitySampleCount++;

        if (stabilitySampleCount >= 5) {
            const float avgHrDiff = hrStabilitySum / stabilitySampleCount;
            const float avgRrDiff = rrStabilitySum / stabilitySampleCount;
            hrStabilitySum = 0.0f;
            rrStabilitySum = 0.0f;
            stabilitySampleCount = 0;
            if (avgHrDiff > BASELINE_HR_STABILITY_THRESHOLD || avgRrDiff > BASELINE_RR_STABILITY_THRESHOLD) {
                return;
            }
        }

        baselineHR = updateBaseline(baselineHR, hrData.bpmSmoothed, BASELINE_BETA);
        baselineRR = updateBaseline(baselineRR, rrData.rateSmoothed, BASELINE_BETA);
        lastBaselineHR = hrData.bpmSmoothed;
        lastBaselineRR = rrData.rateSmoothed;
        return;
    }

    baselineHRSum += hrData.bpmSmoothed;
    baselineRRSum += rrData.rateSmoothed;
    baselineSampleCount++;

    if (baselineSampleCount >= 30) {
        lastBaselineHR = baselineHR;
        lastBaselineRR = baselineRR;
        baselineHR = baselineHRSum / baselineSampleCount;
        baselineRR = baselineRRSum / baselineSampleCount;
        baselineCalibrated = true;
        baselineHRSum = 0.0f;
        baselineRRSum = 0.0f;
        baselineSampleCount = 0;
    }
}

float SleepAnalyzer::calculateSleepinessScore(const HeartRateData &hrData,
                                              const RespirationData &rrData,
                                              const HRVEstimate &hrvData,
                                              const BodyMovementData &movementData) const
{
    float hrSleepFactor = 0.5f;
    float hrvNorm = 0.0f;
    float rrStable = 0.0f;
    float moveNorm = 0.0f;

    if (hrData.isValid) {
        const float hrNorm = normalizeHR(hrData.bpmSmoothed);
        hrSleepFactor = (1.0f - hrNorm) * 0.5f;
    }
    if (hrvData.isValid) hrvNorm = normalizeHRV(hrvData.rmssd);
    if (rrData.isValid) rrStable = 1.0f - constrain_value(rrData.variability / 5.0f, 0.0f, 1.0f);
    if (movementData.isValid) moveNorm = normalizeMovement(movementData.movement);

    const float x = 3.0f * (hrSleepFactor - 0.5f)
                  + 2.5f * (hrvNorm - 0.5f)
                  + 2.0f * (rrStable - 0.5f)
                  + 2.5f * (0.5f - moveNorm);
    return sigmoid(x);
}

float SleepAnalyzer::calculateDeepSleepScore(const HeartRateData &hrData,
                                             const RespirationData &,
                                             const HRVEstimate &hrvData,
                                             const BodyMovementData &movementData) const
{
    if (movementData.isValid && movementData.movement > DEEP_SLEEP_HARD_MOVEMENT_LIMIT) return 0.0f;

    float hrSleepFactor = 0.5f;
    float hrvNorm = 0.0f;
    float moveNorm = 0.0f;

    if (hrData.isValid) {
        const float hrNorm = normalizeHR(hrData.bpmSmoothed);
        hrSleepFactor = (1.0f - hrNorm) * 0.5f;
    }
    if (hrvData.isValid) hrvNorm = normalizeHRV(hrvData.rmssd);
    if (movementData.isValid) moveNorm = normalizeMovement(movementData.movement);

    const float x = 4.0f * (hrSleepFactor - 0.5f)
                  + 3.0f * (hrvNorm - 0.5f)
                  + 2.0f * (0.5f - moveNorm);
    return sigmoid(x);
}

float SleepAnalyzer::calculateLightSleepScore(const HeartRateData &hrData,
                                              const RespirationData &rrData,
                                              const HRVEstimate &hrvData,
                                              const BodyMovementData &movementData) const
{
    float hrMid = 0.0f;
    float hrvMid = 0.0f;
    float moveMid = 0.0f;
    float rrStable = 0.0f;

    if (hrData.isValid) {
        const float hrNorm = normalizeHR(hrData.bpmSmoothed);
        const float hrSleepFactor = (1.0f - hrNorm) * 0.5f;
        hrMid = 1.0f - std::fabs(hrSleepFactor - 0.5f) * 2.0f;
    }
    if (hrvData.isValid) {
        const float hrvNorm = normalizeHRV(hrvData.rmssd);
        hrvMid = 1.0f - std::fabs(hrvNorm - 0.5f) * 2.0f;
    }
    if (movementData.isValid) {
        const float moveNorm = normalizeMovement(movementData.movement);
        if (moveNorm >= 0.1f && moveNorm <= 0.4f) {
            moveMid = 1.0f - std::fabs(moveNorm - 0.25f) * 4.0f;
        }
    }
    if (rrData.isValid) rrStable = rrData.regularity;

    return constrain_value(0.3f * hrMid + 0.3f * hrvMid + 0.2f * moveMid + 0.2f * rrStable, 0.0f, 1.0f);
}

float SleepAnalyzer::calculateAwakeScore(const HeartRateData &hrData,
                                         const RespirationData &rrData,
                                         const HRVEstimate &,
                                         const BodyMovementData &movementData) const
{
    float moveNorm = 0.0f;
    float hrAwakeFactor = 0.5f;
    float rrVar = 0.0f;

    if (movementData.isValid) moveNorm = normalizeMovement(movementData.movement);
    if (hrData.isValid) {
        const float hrNorm = normalizeHR(hrData.bpmSmoothed);
        hrAwakeFactor = (hrNorm + 1.0f) * 0.5f;
    }
    if (rrData.isValid) rrVar = constrain_value(rrData.variability / 5.0f, 0.0f, 1.0f);

    const float x = 3.0f * (moveNorm - 0.3f)
                  + 2.0f * (hrAwakeFactor - 0.5f)
                  + 1.5f * (rrVar - 0.3f);
    return sigmoid(x);
}

float SleepAnalyzer::calculateRemScore(const HeartRateData &hrData,
                                       const RespirationData &rrData,
                                       const HRVEstimate &hrvData,
                                       const BodyMovementData &movementData)
{
    float hrHigh = 0.0f;
    float hrvLow = 0.0f;
    float rrUnstable = 0.0f;
    float moveLow = 0.0f;
    float rrChange = 0.0f;

    if (hrData.isValid) hrHigh = constrain_value(normalizeHR(hrData.bpmSmoothed), 0.0f, 1.0f);
    if (hrvData.isValid) hrvLow = 1.0f - normalizeHRV(hrvData.rmssd);
    if (rrData.isValid) {
        rrUnstable = constrain_value(rrData.variability / 5.0f, 0.0f, 1.0f);
        if (lastRRValue > 0.0f) {
            rrChange = constrain_value(std::fabs(rrData.rateSmoothed - lastRRValue) / 3.0f, 0.0f, 1.0f);
        }
        lastRRValue = rrData.rateSmoothed;
    }
    if (movementData.isValid) moveLow = 1.0f - normalizeMovement(movementData.movement);
    if (movementData.isValid && movementData.movement > DEEP_SLEEP_HARD_MOVEMENT_LIMIT) return 0.0f;

    const float x = 2.5f * (hrHigh - 0.3f)
                  + 2.0f * (hrvLow - 0.3f)
                  + 1.5f * (rrUnstable - 0.3f)
                  + 2.0f * (rrChange - 0.3f)
                  + 2.0f * (moveLow - 0.5f);
    return sigmoid(x);
}

bool SleepAnalyzer::tryTransitionTo(SleepState target, uint64_t confirmMs)
{
    if (pendingState != target) {
        pendingState = target;
        pendingStateTime = radar_now_ms();
        return false;
    }
    if (radar_now_ms() - pendingStateTime >= confirmMs) {
        currentState = target;
        stateEnterTime = radar_now_ms();
        pendingState = target;
        return true;
    }
    return false;
}

void SleepAnalyzer::updateSleepCycle()
{
    if (currentState == SLEEP_DEEP_SLEEP && !cycle.inDeepPhase) {
        cycle.inDeepPhase = true;
        if (cycle.cycleStartTime == 0) cycle.cycleStartTime = radar_now_ms();
    }
    if (currentState == SLEEP_REM_SLEEP && !cycle.inRemPhase) cycle.inRemPhase = true;
    if (currentState == SLEEP_LIGHT_SLEEP && cycle.inDeepPhase) {
        cycle.inDeepPhase = false;
        cycle.lastDeepEndTime = radar_now_ms();
    }
    if (currentState == SLEEP_LIGHT_SLEEP && cycle.inRemPhase) {
        cycle.inRemPhase = false;
        cycle.lastRemEndTime = radar_now_ms();
    }
    if ((currentState == SLEEP_AWAKE || currentState == SLEEP_OUT_OF_BED) &&
        (cycle.inDeepPhase || cycle.inRemPhase) &&
        cycle.cycleStartTime > 0) {
        cycle.cycleCount++;
        cycle.inDeepPhase = false;
        cycle.inRemPhase = false;
        cycle.cycleStartTime = radar_now_ms();
        stats.sleepCycles = cycle.cycleCount;
    }
}

void SleepAnalyzer::updateState(const PresenceData &presence,
                                const HeartRateData &hrData,
                                const RespirationData &rrData,
                                const HRVEstimate &hrvData,
                                const BodyMovementData &movementData)
{
    const uint64_t now = radar_now_ms();
    if (currentState != SLEEP_NO_PERSON && currentState != SLEEP_SESSION_END &&
        (now - stateEnterTime) < MIN_STATE_DWELL_MS) {
        return;
    }

    if (!presence.isPresent) {
        noPersonTimer += 1000;
    } else {
        noPersonTimer = 0;
    }

    const float movement = movementData.isValid ? movementData.movement : 100.0f;
    const float rawSleepiness = calculateSleepinessScore(hrData, rrData, hrvData, movementData);
    currentSleepiness = emaSmooth(rawSleepiness, currentSleepiness, EMA_ALPHA);

    switch (currentState) {
    case SLEEP_NO_PERSON:
        if (presence.isPresent) {
            currentState = movement > 40.0f ? SLEEP_AWAKE : SLEEP_IN_BED;
            stateEnterTime = now;
            pendingState = currentState;
            stats.sessionStartTime = now;
        }
        break;

    case SLEEP_IN_BED:
    case SLEEP_AWAKE:
        if (!presence.isPresent && noPersonTimer > OUT_OF_BED_SECONDS * 1000ULL) {
            currentState = SLEEP_OUT_OF_BED;
            stateEnterTime = now;
            pendingState = currentState;
            break;
        }
        if (wasAsleep && movement >= GETTING_UP_MOVEMENT_THRESHOLD) {
            gettingUpDuration += 1000;
            if (gettingUpDuration >= GETTING_UP_MIN_SECONDS * 1000ULL) {
                currentState = SLEEP_GETTING_UP;
                stateEnterTime = now;
                pendingState = currentState;
                gettingUpDuration = 0;
                break;
            }
        } else {
            gettingUpDuration = 0;
        }

        if (currentSleepiness > SLEEPINESS_THRESHOLD && movement < SLEEPINESS_MOVEMENT_THRESHOLD) {
            sleepinessDuration += 1000;
            if (sleepinessDuration >= SLEEPINESS_MIN_SECONDS * 1000ULL) {
                currentState = SLEEP_LIGHT_SLEEP;
                stateEnterTime = now;
                pendingState = currentState;
                stats.sleepStartTime = now;
                stats.sleepLatency = (stats.sessionStartTime > 0) ? (now - stats.sessionStartTime) : 0;
                wasAsleep = true;
                cycle.cycleStartTime = now;
            }
        } else {
            sleepinessDuration = 0;
            currentState = movement >= MOVEMENT_HIGH_THRESHOLD ? SLEEP_AWAKE : SLEEP_IN_BED;
            pendingState = currentState;
        }
        break;

    case SLEEP_LIGHT_SLEEP: {
        if (!presence.isPresent && noPersonTimer > OUT_OF_BED_SECONDS * 1000ULL) {
            currentState = SLEEP_OUT_OF_BED;
            stateEnterTime = now;
            pendingState = currentState;
            stats.wakeCount++;
            break;
        }

        const float rawDeep = calculateDeepSleepScore(hrData, rrData, hrvData, movementData);
        const float rawLight = calculateLightSleepScore(hrData, rrData, hrvData, movementData);
        const float rawAwake = calculateAwakeScore(hrData, rrData, hrvData, movementData);
        const float rawRem = calculateRemScore(hrData, rrData, hrvData, movementData);

        currentDeepScore = emaSmooth(rawDeep, currentDeepScore, EMA_ALPHA);
        currentLightScore = emaSmooth(rawLight, currentLightScore, EMA_ALPHA);
        currentAwakeScore = emaSmooth(rawAwake, currentAwakeScore, EMA_ALPHA);
        currentRemScore = emaSmooth(rawRem, currentRemScore, EMA_ALPHA);

        if (isBestScore(currentAwakeScore, currentDeepScore, currentLightScore, currentRemScore, CONFIDENCE_MARGIN)) {
            awakeDuration += 1000;
            deepStableDuration = 0;
            if (tryTransitionTo(SLEEP_AWAKE, AWAKE_SLOW_CONFIRM_SECONDS * 1000ULL)) {
                stats.wakeCount++;
                awakeDuration = 0;
            }
        } else if (isBestScore(currentDeepScore, currentAwakeScore, currentLightScore, currentRemScore, CONFIDENCE_MARGIN) &&
                   currentDeepScore >= HYSTERESIS_ENTER_DEEP) {
            if (movementData.isValid && movementData.movement < DEEP_SLEEP_HARD_MOVEMENT_LIMIT) deepStableDuration += 1000;
            else deepStableDuration = 0;
            awakeDuration = 0;
            if (deepStableDuration >= DEEP_STABLE_MIN_SECONDS * 1000ULL) {
                tryTransitionTo(SLEEP_DEEP_SLEEP, DEEP_SLEEP_CONFIRM_SECONDS * 1000ULL);
            }
        } else if (isBestScore(currentRemScore, currentAwakeScore, currentDeepScore, currentLightScore, CONFIDENCE_MARGIN) &&
                   currentRemScore >= HYSTERESIS_ENTER_REM) {
            remSleepDuration += 1000;
            if (tryTransitionTo(SLEEP_REM_SLEEP, REM_CONFIRM_SECONDS * 1000ULL)) remSleepDuration = 0;
        } else {
            deepStableDuration = 0;
            awakeDuration = 0;
            pendingState = SLEEP_LIGHT_SLEEP;
        }
        break;
    }

    case SLEEP_DEEP_SLEEP:
        if (!presence.isPresent && noPersonTimer > OUT_OF_BED_SECONDS * 1000ULL) {
            currentState = SLEEP_OUT_OF_BED;
            stateEnterTime = now;
            pendingState = currentState;
            stats.wakeCount++;
            break;
        }
        if (movement > FAST_AWAKE_MOVEMENT_THRESHOLD) {
            currentState = SLEEP_AWAKE;
            stateEnterTime = now;
            pendingState = currentState;
            stats.wakeCount++;
            break;
        }
        if (movement > DEEP_SLEEP_HARD_MOVEMENT_LIMIT) {
            lightSleepDuration += 1000;
            tryTransitionTo(SLEEP_LIGHT_SLEEP, LIGHT_SLEEP_CONFIRM_SECONDS * 1000ULL);
        } else {
            lightSleepDuration = 0;
            pendingState = SLEEP_DEEP_SLEEP;
        }
        break;

    case SLEEP_REM_SLEEP:
        if (!presence.isPresent && noPersonTimer > OUT_OF_BED_SECONDS * 1000ULL) {
            currentState = SLEEP_OUT_OF_BED;
            stateEnterTime = now;
            pendingState = currentState;
            stats.wakeCount++;
            break;
        }
        if (movement > FAST_AWAKE_MOVEMENT_THRESHOLD) {
            currentState = SLEEP_AWAKE;
            stateEnterTime = now;
            pendingState = currentState;
            stats.wakeCount++;
            break;
        }
        if (calculateLightSleepScore(hrData, rrData, hrvData, movementData) > currentRemScore) {
            tryTransitionTo(SLEEP_LIGHT_SLEEP, LIGHT_SLEEP_CONFIRM_SECONDS * 1000ULL);
        }
        break;

    case SLEEP_OUT_OF_BED:
        if (presence.isPresent) {
            currentState = SLEEP_IN_BED;
            stateEnterTime = now;
            pendingState = currentState;
        } else if (noPersonTimer > NO_PERSON_END_SECONDS * 1000ULL) {
            currentState = SLEEP_SESSION_END;
            stateEnterTime = now;
            pendingState = currentState;
            if (wasAsleep) calculateSleepScore();
        }
        break;

    case SLEEP_GETTING_UP:
        if (!presence.isPresent && noPersonTimer > NO_PERSON_END_SECONDS * 1000ULL) {
            currentState = SLEEP_SESSION_END;
            stateEnterTime = now;
            pendingState = currentState;
            if (wasAsleep) calculateSleepScore();
        } else if (movement < GETTING_UP_MOVEMENT_THRESHOLD) {
            gettingUpDuration += 1000;
            if (gettingUpDuration >= 60000ULL) {
                currentState = SLEEP_AWAKE;
                stateEnterTime = now;
                pendingState = currentState;
                gettingUpDuration = 0;
            }
        } else {
            gettingUpDuration = 0;
        }
        break;

    case SLEEP_SESSION_END:
        if (presence.isPresent) {
            currentState = SLEEP_IN_BED;
            stateEnterTime = now;
            pendingState = currentState;
            noPersonTimer = 0;
        }
        break;
    }
}

void SleepAnalyzer::updateStatistics(uint64_t dt)
{
    switch (currentState) {
    case SLEEP_LIGHT_SLEEP:
        stats.lightSleepTime += dt;
        stats.totalSleepTime += dt;
        break;
    case SLEEP_DEEP_SLEEP:
        stats.deepSleepTime += dt;
        stats.totalSleepTime += dt;
        break;
    case SLEEP_REM_SLEEP:
        stats.remSleepTime += dt;
        stats.totalSleepTime += dt;
        break;
    case SLEEP_AWAKE:
        stats.awakeTime += dt;
        break;
    case SLEEP_OUT_OF_BED:
        stats.outOfBedTime += dt;
        break;
    default:
        break;
    }
}

void SleepAnalyzer::calculateSleepScore()
{
    const float totalHours = static_cast<float>(stats.totalSleepTime) / 3600000.0f;
    if (totalHours >= 7.0f && totalHours <= 9.0f) score.durationScore = 18.0f;
    else if (totalHours >= 6.0f && totalHours < 7.0f) score.durationScore = 13.0f;
    else if (totalHours > 9.0f && totalHours <= 10.0f) score.durationScore = 13.0f;
    else score.durationScore = 5.0f;

    const float deepRatio = stats.totalSleepTime > 0 ? static_cast<float>(stats.deepSleepTime) / stats.totalSleepTime : 0.0f;
    if (deepRatio > 0.2f) score.deepScore = 14.0f;
    else if (deepRatio > 0.15f) score.deepScore = 11.0f;
    else if (deepRatio > 0.1f) score.deepScore = 7.0f;
    else score.deepScore = 3.0f;

    if (stats.wakeCount <= 1) score.continuityScore = 11.0f;
    else if (stats.wakeCount <= 3) score.continuityScore = 7.0f;
    else if (stats.wakeCount <= 5) score.continuityScore = 4.0f;
    else score.continuityScore = 2.0f;

    score.physiologyScore = 7.0f;
    const float latencyMin = static_cast<float>(stats.sleepLatency) / 60000.0f;
    if (latencyMin < 20.0f) score.latencyScore = 8.0f;
    else if (latencyMin < 30.0f) score.latencyScore = 6.0f;
    else if (latencyMin < 45.0f) score.latencyScore = 3.0f;
    else score.latencyScore = 1.0f;

    float sleepEfficiency = 0.0f;
    if (stats.totalSleepTime + stats.awakeTime > 0) {
        sleepEfficiency = static_cast<float>(stats.totalSleepTime) / (stats.totalSleepTime + stats.awakeTime);
    }
    if (sleepEfficiency > 0.9f) score.efficiencyScore = 14.0f;
    else if (sleepEfficiency > 0.8f) score.efficiencyScore = 10.0f;
    else if (sleepEfficiency > 0.7f) score.efficiencyScore = 6.0f;
    else score.efficiencyScore = 3.0f;

    const float remRatio = stats.totalSleepTime > 0 ? static_cast<float>(stats.remSleepTime) / stats.totalSleepTime : 0.0f;
    float cycleScoreVal = 0.0f;
    if (stats.sleepCycles >= 4) cycleScoreVal = 20.0f;
    else if (stats.sleepCycles >= 3) cycleScoreVal = 15.0f;
    else if (stats.sleepCycles >= 2) cycleScoreVal = 10.0f;
    else if (stats.sleepCycles >= 1) cycleScoreVal = 6.0f;
    else cycleScoreVal = 2.0f;
    if (remRatio >= 0.2f && remRatio <= 0.25f) cycleScoreVal += 8.0f;
    else if (remRatio >= 0.15f) cycleScoreVal += 5.0f;
    else if (remRatio > 0.0f) cycleScoreVal += 2.0f;
    score.cycleScore = constrain_value(cycleScoreVal, 0.0f, 28.0f);

    const float rawTotal = score.durationScore + score.deepScore + score.continuityScore +
                           score.physiologyScore + score.latencyScore + score.efficiencyScore +
                           score.cycleScore;
    score.totalScore = constrain_value(rawTotal, 0.0f, 100.0f);
}

void SleepAnalyzer::update(const HeartRateData &hrData,
                           const RespirationData &rrData,
                           const HRVEstimate &hrvData,
                           const BodyMovementData &movementData)
{
    calibrateBaseline(hrData, rrData, movementData);
    const PresenceData presence = evaluatePresence(hrData, rrData, movementData);
    updateState(presence, hrData, rrData, hrvData, movementData);
    updateSleepCycle();
    updateStatistics(1000);
}

std::string SleepAnalyzer::formatState() const
{
    const uint64_t stateDuration = (radar_now_ms() - stateEnterTime) / 1000ULL;
    std::ostringstream oss;
    oss << SLEEP_STATE_NAMES[currentState]
        << " duration=" << stateDuration
        << "s sleepiness=" << currentSleepiness
        << " cycles=" << cycle.cycleCount;
    return oss.str();
}

std::string SleepAnalyzer::formatStatistics() const
{
    std::ostringstream oss;
    oss << "total=" << stats.totalSleepTime
        << " deep=" << stats.deepSleepTime
        << " light=" << stats.lightSleepTime
        << " rem=" << stats.remSleepTime
        << " awake=" << stats.awakeTime
        << " out=" << stats.outOfBedTime
        << " wakes=" << stats.wakeCount
        << " score=" << score.totalScore;
    return oss.str();
}
