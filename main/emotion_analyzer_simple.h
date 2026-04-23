#ifndef EMOTION_ANALYZER_SIMPLE_H
#define EMOTION_ANALYZER_SIMPLE_H

#include <stdint.h>
#include <string>

#include "data_processor.h"
#include "radar_config.h"

struct EmotionResult {
    EmotionType primaryEmotion;
    EmotionType secondaryEmotion;
    float confidence;
    float intensity;
    float valence;
    float arousal;
    float stressLevel;
    float anxietyLevel;
    float relaxationLevel;
    float sympatheticActivity;
    float parasympatheticActivity;
    bool isValid;
    uint64_t timestamp;
};

struct UserBaseline {
    static const int COLD_START_SAMPLES = 15;

    float hrResting;
    float hrMin;
    float hrMax;
    float rrResting;
    bool isCalibrated;
    uint64_t calibrationTime;
    float coldStartHrSum;
    float coldStartRrSum;
    int coldStartCount;
    bool isColdStarting;
};

struct BodyMovementData {
    uint8_t movement;
    float movementSmoothed;
    float movementMean;
    float movementStd;
    float activityLevel;
    bool isValid;
    uint64_t timestamp;
};

class SimpleEmotionAnalyzer {
public:
    explicit SimpleEmotionAnalyzer(int histSize = 30);
    ~SimpleEmotionAnalyzer();

    EmotionResult analyze(const HeartRateData &hrData,
                          const RespirationData &rrData,
                          const HRVEstimate &hrvData,
                          const BodyMovementData &movementData);

    void setBaseline(const UserBaseline &bl);
    UserBaseline getBaseline() const { return baseline; }
    void calibrateBaseline(const HeartRateData &hrData,
                           const RespirationData &rrData,
                           const BodyMovementData &movementData);
    void setSmoothing(float factor) { smoothingFactor = constrain_value(factor, 0.0f, 1.0f); }
    EmotionType getRecentDominantEmotion(int seconds = 60);
    void reset();

private:
    float calculateCalmScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement);
    float calculateHappyScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement);
    float calculateExcitedScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement);
    float calculateAnxiousScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement);
    float calculateAngryScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement);
    float calculateSadScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement);
    float calculateStressedScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement);
    float calculateRelaxedScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement);

    float sigmoid(float x, float k = 1.0f, float x0 = 0.0f) const;
    float gaussian(float x, float mean, float std) const;
    void normalizeProbabilities();
    void smoothProbabilities();
    void calculateDimensions();
    void calculateStressLevels(const RespirationData &rr, const HRVEstimate &hrv);
    float getSmoothedHR(const HeartRateData &hr);
    float getSmoothedRR(const RespirationData &rr);
    float getSmoothedHRV(const HRVEstimate &hrv);
    float normalizeHR(float hr, float baselineValue) const;
    float normalizeRR(float rr, float baselineValue) const;
    float normalizeHRV(float hrv) const;
    float normalizeMovement(float movement) const;

    UserBaseline baseline;
    EmotionResult lastResult;
    float emotionProbs[9];
    float smoothingFactor;
    float prevProbs[9];
    EmotionType *emotionHistory;
    int historySize;
    int historyIndex;
    int historyCount;

    static const int WINDOW_SIZE = 15;
    float hrWindow[WINDOW_SIZE];
    float rrWindow[WINDOW_SIZE];
    float hrvWindow[WINDOW_SIZE];
    int windowIndex;
    int windowCount;
};

class EmotionOutput {
public:
    static std::string toBrief(const EmotionResult &result);
    static std::string toJson(const EmotionResult &result);
};

#endif
