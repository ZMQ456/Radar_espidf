#ifndef SLEEP_ANALYZER_H
#define SLEEP_ANALYZER_H

#include <stdint.h>
#include <string>

#include "data_processor.h"
#include "emotion_analyzer_simple.h"

enum SleepState {
    SLEEP_NO_PERSON = 0,
    SLEEP_IN_BED,
    SLEEP_AWAKE,
    SLEEP_LIGHT_SLEEP,
    SLEEP_DEEP_SLEEP,
    SLEEP_REM_SLEEP,
    SLEEP_OUT_OF_BED,
    SLEEP_GETTING_UP,
    SLEEP_SESSION_END
};

struct PresenceData {
    bool isPresent;
    float distance;
    float confidence;
    float motionEnergy;
};

struct SleepCycle {
    int cycleCount;
    uint64_t cycleStartTime;
    bool inDeepPhase;
    bool inRemPhase;
    uint64_t lastDeepEndTime;
    uint64_t lastRemEndTime;
};

struct SleepStatistics {
    uint64_t totalSleepTime;
    uint64_t deepSleepTime;
    uint64_t lightSleepTime;
    uint64_t remSleepTime;
    uint64_t awakeTime;
    uint64_t outOfBedTime;
    uint64_t sleepLatency;
    int wakeCount;
    int sleepCycles;
    uint64_t sessionStartTime;
    uint64_t sleepStartTime;
    uint64_t lastWakeTime;
};

struct SleepScore {
    float durationScore;
    float deepScore;
    float continuityScore;
    float physiologyScore;
    float latencyScore;
    float efficiencyScore;
    float cycleScore;
    float totalScore;
};

class SleepAnalyzer {
public:
    SleepAnalyzer();
    ~SleepAnalyzer() = default;

    void update(const HeartRateData &hrData,
                const RespirationData &rrData,
                const HRVEstimate &hrvData,
                const BodyMovementData &movementData);

    SleepState getCurrentState() const { return currentState; }
    SleepStatistics getStatistics() const { return stats; }
    SleepScore getScore() const { return score; }
    SleepCycle getCycle() const { return cycle; }
    float getSleepiness() const { return currentSleepiness; }

    void reset();
    std::string formatState() const;
    std::string formatStatistics() const;

private:
    static const float EMA_ALPHA;
    static const float CONFIDENCE_MARGIN;
    static const float BASELINE_BETA;
    static const float SLEEPINESS_THRESHOLD;
    static const float BASELINE_MOVEMENT_THRESHOLD;
    static const float BASELINE_HR_STABILITY_THRESHOLD;
    static const float BASELINE_RR_STABILITY_THRESHOLD;
    static const float HYSTERESIS_ENTER_DEEP;
    static const float HYSTERESIS_EXIT_DEEP;
    static const float HYSTERESIS_ENTER_REM;
    static const float HYSTERESIS_EXIT_REM;

    static const int MIN_STATE_DWELL_MS = 10000;
    static const int DEEP_SLEEP_CONFIRM_SECONDS = 60;
    static const int LIGHT_SLEEP_CONFIRM_SECONDS = 30;
    static const int AWAKE_CONFIRM_SECONDS = 15;
    static const int AWAKE_SLOW_CONFIRM_SECONDS = 30;
    static const int NO_PERSON_END_SECONDS = 600;
    static const int OUT_OF_BED_SECONDS = 30;
    static const int SLEEPINESS_MIN_SECONDS = 300;
    static const int MOVEMENT_HIGH_THRESHOLD = 50;
    static const int DEEP_SLEEP_HARD_MOVEMENT_LIMIT = 15;
    static const int FAST_AWAKE_MOVEMENT_THRESHOLD = 60;
    static const int SLEEPINESS_MOVEMENT_THRESHOLD = 10;
    static const int DEEP_STABLE_MIN_SECONDS = 300;
    static const int REM_CONFIRM_SECONDS = 60;
    static const int GETTING_UP_MIN_SECONDS = 300;
    static const int GETTING_UP_MOVEMENT_THRESHOLD = 30;

    PresenceData evaluatePresence(const HeartRateData &hrData,
                                  const RespirationData &rrData,
                                  const BodyMovementData &movementData) const;
    float sigmoid(float x) const;
    float emaSmooth(float input, float last, float alpha) const;
    float updateBaseline(float current, float input, float beta) const;
    float calculateSleepinessScore(const HeartRateData &hrData,
                                   const RespirationData &rrData,
                                   const HRVEstimate &hrvData,
                                   const BodyMovementData &movementData) const;
    float calculateDeepSleepScore(const HeartRateData &hrData,
                                  const RespirationData &rrData,
                                  const HRVEstimate &hrvData,
                                  const BodyMovementData &movementData) const;
    float calculateLightSleepScore(const HeartRateData &hrData,
                                   const RespirationData &rrData,
                                   const HRVEstimate &hrvData,
                                   const BodyMovementData &movementData) const;
    float calculateAwakeScore(const HeartRateData &hrData,
                              const RespirationData &rrData,
                              const HRVEstimate &hrvData,
                              const BodyMovementData &movementData) const;
    float calculateRemScore(const HeartRateData &hrData,
                            const RespirationData &rrData,
                            const HRVEstimate &hrvData,
                            const BodyMovementData &movementData);
    void updateState(const PresenceData &presence,
                     const HeartRateData &hrData,
                     const RespirationData &rrData,
                     const HRVEstimate &hrvData,
                     const BodyMovementData &movementData);
    void updateStatistics(uint64_t dt);
    void updateSleepCycle();
    void calculateSleepScore();
    void calibrateBaseline(const HeartRateData &hrData,
                           const RespirationData &rrData,
                           const BodyMovementData &movementData);
    float normalizeHR(float hr) const;
    float normalizeRR(float rr) const;
    float normalizeHRV(float hrv) const;
    float normalizeMovement(float movement) const;
    bool tryTransitionTo(SleepState target, uint64_t confirmMs);
    bool isBestScore(float score, float s2, float s3, float s4, float margin) const;

    SleepState currentState;
    SleepState pendingState;
    SleepStatistics stats;
    SleepScore score;
    SleepCycle cycle;

    uint64_t stateEnterTime;
    uint64_t pendingStateTime;
    uint64_t noPersonTimer;
    uint64_t sleepinessDuration;
    uint64_t awakeDuration;
    uint64_t deepSleepDuration;
    uint64_t lightSleepDuration;
    uint64_t remSleepDuration;
    uint64_t movementHighDuration;
    uint64_t gettingUpDuration;
    uint64_t deepStableDuration;

    float baselineHR;
    float baselineRR;
    bool baselineCalibrated;
    int baselineSampleCount;
    float baselineHRSum;
    float baselineRRSum;
    float lastBaselineHR;
    float lastBaselineRR;
    float hrStabilitySum;
    float rrStabilitySum;
    int stabilitySampleCount;
    float lastRRValue;
    float currentSleepiness;
    float currentDeepScore;
    float currentLightScore;
    float currentAwakeScore;
    float currentRemScore;
    bool wasAsleep;
};

#endif
