#include "emotion_analyzer_simple.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

#include "radar_platform.h"

const int SimpleEmotionAnalyzer::WINDOW_SIZE;

SimpleEmotionAnalyzer::SimpleEmotionAnalyzer(int histSize)
    : emotionHistory(new EmotionType[histSize]),
      historySize(histSize),
      historyIndex(0),
      historyCount(0),
      windowIndex(0),
      windowCount(0)
{
    memset(&baseline, 0, sizeof(baseline));
    memset(&lastResult, 0, sizeof(lastResult));
    memset(emotionProbs, 0, sizeof(emotionProbs));
    memset(prevProbs, 0, sizeof(prevProbs));
    memset(hrWindow, 0, sizeof(hrWindow));
    memset(rrWindow, 0, sizeof(rrWindow));
    memset(hrvWindow, 0, sizeof(hrvWindow));
    memset(emotionHistory, 0, historySize * sizeof(EmotionType));

    baseline.hrResting = 72.0f;
    baseline.rrResting = 16.0f;
    baseline.isColdStarting = true;
}

SimpleEmotionAnalyzer::~SimpleEmotionAnalyzer()
{
    delete[] emotionHistory;
}

EmotionResult SimpleEmotionAnalyzer::analyze(const HeartRateData &hrData,
                                             const RespirationData &rrData,
                                             const HRVEstimate &hrvData,
                                             const BodyMovementData &movementData)
{
    EmotionResult result = {};
    result.timestamp = radar_now_ms();

    if (!hrData.isValid && !rrData.isValid) {
        return result;
    }

    const float smoothedHR = getSmoothedHR(hrData);
    const float smoothedRR = getSmoothedRR(rrData);
    const float smoothedHRV = getSmoothedHRV(hrvData);

    windowIndex = (windowIndex + 1) % WINDOW_SIZE;
    if (windowCount < WINDOW_SIZE - 1) {
        windowCount++;
    }

    HeartRateData smoothHrData = hrData;
    RespirationData smoothRrData = rrData;
    HRVEstimate smoothHrvData = hrvData;
    if (smoothHrData.isValid) smoothHrData.bpmSmoothed = smoothedHR;
    if (smoothRrData.isValid) smoothRrData.rateSmoothed = smoothedRR;
    if (smoothHrvData.isValid) smoothHrvData.rmssd = smoothedHRV;

    emotionProbs[EMOTION_CALM] = calculateCalmScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_HAPPY] = calculateHappyScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_EXCITED] = calculateExcitedScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_ANXIOUS] = calculateAnxiousScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_ANGRY] = calculateAngryScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_SAD] = calculateSadScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_STRESSED] = calculateStressedScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_RELAXED] = calculateRelaxedScore(smoothHrData, smoothRrData, smoothHrvData, movementData);
    emotionProbs[EMOTION_UNKNOWN] = 0.02f;

    normalizeProbabilities();

    int maxIdx = 0;
    for (int i = 1; i < 9; ++i) {
        if (emotionProbs[i] > emotionProbs[maxIdx]) {
            maxIdx = i;
        }
    }
    emotionProbs[maxIdx] *= 1.3f;
    normalizeProbabilities();
    smoothProbabilities();

    int secondIdx = 0;
    float maxProb = emotionProbs[0];
    float secondProb = 0.0f;
    maxIdx = 0;
    for (int i = 1; i < 9; ++i) {
        if (emotionProbs[i] > maxProb) {
            secondProb = maxProb;
            secondIdx = maxIdx;
            maxProb = emotionProbs[i];
            maxIdx = i;
        } else if (emotionProbs[i] > secondProb) {
            secondProb = emotionProbs[i];
            secondIdx = i;
        }
    }

    result.primaryEmotion = static_cast<EmotionType>(maxIdx);
    result.secondaryEmotion = static_cast<EmotionType>(secondIdx);
    result.confidence = maxProb;

    const float hrFactor = std::fabs(hrData.bpmSmoothed - baseline.hrResting) / 40.0f;
    const float hrvFactor = hrvData.isValid ? (1.0f - sigmoid(hrvData.rmssd, 0.02f, 40.0f)) : 0.5f;
    const float rrFactor = rrData.isValid ? std::fabs(rrData.rateSmoothed - baseline.rrResting) / 10.0f : 0.3f;
    result.intensity = 0.4f
        + 0.3f * constrain_value(hrFactor, 0.0f, 1.0f)
        + 0.2f * constrain_value(hrvFactor, 0.0f, 1.0f)
        + 0.1f * constrain_value(rrFactor, 0.0f, 1.0f);
    result.intensity = constrain_value(result.intensity, 0.3f, 1.0f);

    calculateDimensions();
    calculateStressLevels(rrData, hrvData);

    result.valence = lastResult.valence;
    result.arousal = lastResult.arousal;
    result.stressLevel = lastResult.stressLevel;
    result.anxietyLevel = lastResult.anxietyLevel;
    result.relaxationLevel = lastResult.relaxationLevel;
    result.sympatheticActivity = hrvData.isValid ? (1.0f - hrvData.autonomicBalance) : 0.5f;
    result.parasympatheticActivity = hrvData.isValid ? hrvData.autonomicBalance : 0.5f;
    result.isValid = true;

    emotionHistory[historyIndex] = result.primaryEmotion;
    historyIndex = (historyIndex + 1) % historySize;
    if (historyCount < historySize) {
        historyCount++;
    }
    lastResult = result;
    return result;
}

void SimpleEmotionAnalyzer::setBaseline(const UserBaseline &bl)
{
    baseline = bl;
}

void SimpleEmotionAnalyzer::calibrateBaseline(const HeartRateData &hrData,
                                              const RespirationData &rrData,
                                              const BodyMovementData &movementData)
{
    if (!movementData.isValid || movementData.movementSmoothed > 30.0f) {
        return;
    }

    if (baseline.isColdStarting && baseline.coldStartCount < UserBaseline::COLD_START_SAMPLES) {
        if (hrData.isValid) baseline.coldStartHrSum += hrData.bpmSmoothed;
        if (rrData.isValid) baseline.coldStartRrSum += rrData.rateSmoothed;
        baseline.coldStartCount++;

        if (baseline.coldStartCount >= UserBaseline::COLD_START_SAMPLES) {
            if (hrData.isValid && baseline.coldStartHrSum > 0.0f) baseline.hrResting = baseline.coldStartHrSum / baseline.coldStartCount;
            if (rrData.isValid && baseline.coldStartRrSum > 0.0f) baseline.rrResting = baseline.coldStartRrSum / baseline.coldStartCount;
            baseline.isColdStarting = false;
            baseline.isCalibrated = true;
            baseline.calibrationTime = radar_now_ms();
        }
        return;
    }

    if (hrData.isValid) baseline.hrResting = baseline.hrResting * 0.7f + hrData.bpmSmoothed * 0.3f;
    if (rrData.isValid) baseline.rrResting = baseline.rrResting * 0.7f + rrData.rateSmoothed * 0.3f;
    baseline.isCalibrated = true;
    baseline.calibrationTime = radar_now_ms();
}

EmotionType SimpleEmotionAnalyzer::getRecentDominantEmotion(int seconds)
{
    int counts[9] = {0};
    const int entries = std::min(historyCount, seconds / 2);
    for (int i = 0; i < entries; ++i) {
        const int idx = (historyIndex - 1 - i + historySize) % historySize;
        counts[emotionHistory[idx]]++;
    }

    int maxCount = 0;
    EmotionType dominant = EMOTION_CALM;
    for (int i = 0; i < 9; ++i) {
        if (counts[i] > maxCount) {
            maxCount = counts[i];
            dominant = static_cast<EmotionType>(i);
        }
    }
    return dominant;
}

void SimpleEmotionAnalyzer::reset()
{
    memset(emotionProbs, 0, sizeof(emotionProbs));
    memset(prevProbs, 0, sizeof(prevProbs));
    memset(hrWindow, 0, sizeof(hrWindow));
    memset(rrWindow, 0, sizeof(rrWindow));
    memset(hrvWindow, 0, sizeof(hrvWindow));
    memset(emotionHistory, 0, historySize * sizeof(EmotionType));
    memset(&lastResult, 0, sizeof(lastResult));
    historyIndex = 0;
    historyCount = 0;
    windowIndex = 0;
    windowCount = 0;
}

float SimpleEmotionAnalyzer::calculateCalmScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.45f * gaussian(std::fabs(normalizeHR(hr.bpmSmoothed, baseline.hrResting)), 0.0f, 0.3f);
    if (rr.isValid) score += 0.27f * gaussian(std::fabs(normalizeRR(rr.rateSmoothed, baseline.rrResting)), 0.0f, 0.5f) + 0.16f * rr.regularity;
    if (hrv.isValid) score += 0.10f * sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.7f);
    if (movement.isValid) score += 0.08f * gaussian(normalizeMovement(movement.movementSmoothed), 0.15f, 0.2f);
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateHappyScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.45f * gaussian(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 0.3f, 0.25f);
    if (rr.isValid) score += 0.34f * gaussian(std::fabs(normalizeRR(rr.rateSmoothed, baseline.rrResting)), 0.0f, 0.5f);
    if (hrv.isValid) score += 0.10f * sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.9f);
    if (movement.isValid) score += 0.08f * gaussian(normalizeMovement(movement.movementSmoothed), 0.45f, 0.25f);
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateExcitedScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.45f * sigmoid(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.6f);
    if (rr.isValid) score += 0.34f * sigmoid(normalizeRR(rr.rateSmoothed, baseline.rrResting), 2.0f, 0.5f);
    if (hrv.isValid) score += 0.10f * gaussian(normalizeHRV(hrv.rmssd), 0.7f, 0.4f);
    if (movement.isValid) score += 0.10f * sigmoid(normalizeMovement(movement.movementSmoothed), 2.0f, 0.6f);
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateAnxiousScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.36f * sigmoid(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.4f);
    if (hrv.isValid) score += 0.25f * (1.0f - sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.6f));
    if (rr.isValid) score += 0.16f * (1.0f - rr.regularity);
    if (movement.isValid) score += 0.10f * gaussian(normalizeMovement(movement.movementSmoothed), 0.55f, 0.3f);
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateAngryScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.36f * sigmoid(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.75f);
    if (hrv.isValid) score += 0.25f * (1.0f - sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.5f));
    if (rr.isValid) score += 0.20f * (1.0f - rr.regularity);
    if (movement.isValid) score += 0.10f * sigmoid(normalizeMovement(movement.movementSmoothed), 2.0f, 0.7f);
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateSadScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.40f * sigmoid(-normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.2f);
    if (rr.isValid) score += 0.28f * sigmoid(-normalizeRR(rr.rateSmoothed, baseline.rrResting), 2.0f, 0.3f);
    if (hrv.isValid) score += 0.10f * (1.0f - sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.9f));
    if (movement.isValid) score += 0.10f * (1.0f - sigmoid(normalizeMovement(movement.movementSmoothed), 2.0f, 0.2f));
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateStressedScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.36f * sigmoid(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.4f);
    if (hrv.isValid) score += 0.34f * (1.0f - sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.6f));
    if (rr.isValid) score += 0.20f * sigmoid(normalizeRR(rr.rateSmoothed, baseline.rrResting), 2.0f, 0.5f);
    if (movement.isValid) score += 0.08f * gaussian(normalizeMovement(movement.movementSmoothed), 0.4f, 0.25f);
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::calculateRelaxedScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.31f * sigmoid(-normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.25f);
    if (hrv.isValid) score += 0.28f * sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 1.0f);
    if (rr.isValid) score += 0.16f * sigmoid(-normalizeRR(rr.rateSmoothed, baseline.rrResting), 2.0f, 0.3f) + 0.10f * rr.regularity;
    if (movement.isValid) score += 0.10f * (1.0f - sigmoid(normalizeMovement(movement.movementSmoothed), 2.0f, 0.15f));
    return constrain_value(score, 0.0f, 1.0f);
}

float SimpleEmotionAnalyzer::sigmoid(float x, float k, float x0) const
{
    return 1.0f / (1.0f + std::exp(-k * (x - x0)));
}

float SimpleEmotionAnalyzer::gaussian(float x, float mean, float std) const
{
    const float diff = x - mean;
    return std::exp(-(diff * diff) / (2.0f * std * std));
}

void SimpleEmotionAnalyzer::normalizeProbabilities()
{
    float sum = 0.0f;
    for (float probability : emotionProbs) sum += probability;
    if (sum > 0.0f) {
        for (float &probability : emotionProbs) probability /= sum;
    }
}

void SimpleEmotionAnalyzer::smoothProbabilities()
{
    for (int i = 0; i < 9; ++i) {
        const float diff = std::fabs(emotionProbs[i] - prevProbs[i]);
        const float adaptiveAlpha = (diff > 0.2f) ? 0.6f : 0.25f;
        emotionProbs[i] = adaptiveAlpha * emotionProbs[i] + (1.0f - adaptiveAlpha) * prevProbs[i];
        prevProbs[i] = emotionProbs[i];
    }
}

void SimpleEmotionAnalyzer::calculateDimensions()
{
    const float positive = emotionProbs[EMOTION_HAPPY] + emotionProbs[EMOTION_EXCITED] +
                           emotionProbs[EMOTION_RELAXED] + emotionProbs[EMOTION_CALM];
    const float negative = emotionProbs[EMOTION_ANXIOUS] + emotionProbs[EMOTION_ANGRY] +
                           emotionProbs[EMOTION_SAD] + emotionProbs[EMOTION_STRESSED];
    lastResult.valence = positive - negative;

    const float highArousal = emotionProbs[EMOTION_EXCITED] + emotionProbs[EMOTION_ANXIOUS] + emotionProbs[EMOTION_ANGRY];
    const float lowArousal = emotionProbs[EMOTION_CALM] + emotionProbs[EMOTION_RELAXED] + emotionProbs[EMOTION_SAD];
    const float total = highArousal + lowArousal;
    lastResult.arousal = total > 0.0f ? highArousal / total : 0.5f;
}

void SimpleEmotionAnalyzer::calculateStressLevels(const RespirationData &rr, const HRVEstimate &hrv)
{
    float hrvNorm = 0.0f;
    if (hrv.isValid) hrvNorm = constrain_value(1.0f - normalizeHRV(hrv.rmssd), 0.0f, 1.0f);
    lastResult.stressLevel = constrain_value((0.40f * hrvNorm) * 100.0f, 0.0f, 100.0f);
    lastResult.anxietyLevel = constrain_value(emotionProbs[EMOTION_ANXIOUS] * 70.0f + emotionProbs[EMOTION_STRESSED] * 50.0f + (1.0f - rr.regularity) * 20.0f, 0.0f, 100.0f);
    lastResult.relaxationLevel = constrain_value(emotionProbs[EMOTION_RELAXED] * 70.0f + emotionProbs[EMOTION_CALM] * 50.0f, 0.0f, 100.0f);
}

float SimpleEmotionAnalyzer::getSmoothedHR(const HeartRateData &hr)
{
    if (!hr.isValid) return baseline.hrResting;
    hrWindow[windowIndex] = hr.bpmSmoothed;
    float sum = 0.0f;
    const int count = std::min(windowCount + 1, WINDOW_SIZE);
    for (int i = 0; i < count; ++i) {
        const int idx = (windowIndex - i + WINDOW_SIZE) % WINDOW_SIZE;
        sum += hrWindow[idx];
    }
    return count > 0 ? sum / count : hr.bpmSmoothed;
}

float SimpleEmotionAnalyzer::getSmoothedRR(const RespirationData &rr)
{
    if (!rr.isValid) return baseline.rrResting;
    rrWindow[windowIndex] = rr.rateSmoothed;
    float sum = 0.0f;
    const int count = std::min(windowCount + 1, WINDOW_SIZE);
    for (int i = 0; i < count; ++i) {
        const int idx = (windowIndex - i + WINDOW_SIZE) % WINDOW_SIZE;
        sum += rrWindow[idx];
    }
    return count > 0 ? sum / count : rr.rateSmoothed;
}

float SimpleEmotionAnalyzer::getSmoothedHRV(const HRVEstimate &hrv)
{
    if (!hrv.isValid) return 40.0f;
    hrvWindow[windowIndex] = hrv.rmssd;
    float sum = 0.0f;
    const int count = std::min(windowCount + 1, WINDOW_SIZE);
    for (int i = 0; i < count; ++i) {
        const int idx = (windowIndex - i + WINDOW_SIZE) % WINDOW_SIZE;
        sum += hrvWindow[idx];
    }
    return count > 0 ? sum / count : hrv.rmssd;
}

float SimpleEmotionAnalyzer::normalizeHR(float hr, float baselineValue) const { return (hr - baselineValue) / 20.0f; }
float SimpleEmotionAnalyzer::normalizeRR(float rr, float baselineValue) const { return (rr - baselineValue) / 4.0f; }
float SimpleEmotionAnalyzer::normalizeHRV(float hrv) const { return hrv / 50.0f; }
float SimpleEmotionAnalyzer::normalizeMovement(float movement) const { return movement / 100.0f; }

std::string EmotionOutput::toBrief(const EmotionResult &result)
{
    if (!result.isValid) return "analyzing";
    return std::string(EMOTION_NAMES[result.primaryEmotion]) + " " + std::to_string(static_cast<int>(result.confidence * 100.0f)) + "%";
}

std::string EmotionOutput::toJson(const EmotionResult &result)
{
    std::ostringstream oss;
    oss << "{"
        << "\"emotion\":\"" << EMOTION_NAMES[result.primaryEmotion] << "\","
        << "\"confidence\":" << result.confidence << ","
        << "\"intensity\":" << result.intensity << ","
        << "\"timestamp\":" << result.timestamp
        << "}";
    return oss.str();
}
