#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

#include <stdint.h>

#include "radar_config.h"

struct HeartRateData {
    float bpm;
    float bpmSmoothed;
    float bpmMean;
    float bpmStd;
    float bpmMin;
    float bpmMax;
    float trend;
    float variability;
    float quality;
    bool isValid;
    uint64_t timestamp;
};

struct RespirationData {
    float rate;
    float rateSmoothed;
    float rateMean;
    float rateStd;
    float regularity;
    float variability;
    float quality;
    bool isValid;
    uint64_t timestamp;
};

struct HRVEstimate {
    float rmssd;
    float sdnn;
    float stressIndex;
    float autonomicBalance;
    bool isValid;
};

class HeartRateProcessor {
public:
    explicit HeartRateProcessor(int histSize = 100);
    ~HeartRateProcessor();

    void addData(float bpm, float confidence = 80.0f);
    HeartRateData getData() const;
    HRVEstimate estimateHRV() const;
    void reset();

private:
    float calculateVariability() const;
    float calculateTrend() const;

    float *bpmHistory;
    int historySize;
    int historyIndex;
    int historyCount;
    float lastSmoothed;
    float alpha;
    float bpmSum;
    float bpmSumSq;
    float lastValidBpm;
    uint64_t lastValidTime;
};

class RespirationProcessor {
public:
    explicit RespirationProcessor(int histSize = 50);
    ~RespirationProcessor();

    void addData(float rate, float confidence = 80.0f);
    RespirationData getData() const;
    void reset();

private:
    float calculateRegularity() const;
    float calculateVariability() const;

    float *rateHistory;
    int historySize;
    int historyIndex;
    int historyCount;
    float lastSmoothed;
    float alpha;
    float lastValidRate;
    uint64_t lastValidTime;
};

class PhysioDataProcessor {
public:
    PhysioDataProcessor();
    ~PhysioDataProcessor();

    void update(float hr, float rr, float hrConf = 80.0f, float rrConf = 80.0f);
    HeartRateData getHeartRateData() const;
    RespirationData getRespirationData() const;
    HRVEstimate getHRVEstimate() const;
    void reset();

private:
    HeartRateProcessor *hrProcessor;
    RespirationProcessor *rrProcessor;
};

#endif
