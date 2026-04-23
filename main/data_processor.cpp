#include "data_processor.h"

#include <cmath>
#include <cstring>

#include "radar_platform.h"

HeartRateProcessor::HeartRateProcessor(int histSize)
    : bpmHistory(new float[histSize]),
      historySize(histSize),
      historyIndex(0),
      historyCount(0),
      lastSmoothed(72.0f),
      alpha(EMA_ALPHA_HR),
      bpmSum(0.0f),
      bpmSumSq(0.0f),
      lastValidBpm(72.0f),
      lastValidTime(0) {
    memset(bpmHistory, 0, historySize * sizeof(float));
}

HeartRateProcessor::~HeartRateProcessor()
{
    delete[] bpmHistory;
}

void HeartRateProcessor::addData(float bpm, float confidence)
{
    (void)confidence;

    if (bpm < HR_MIN_NORMAL || bpm > HR_MAX_NORMAL) {
        return;
    }

    if (historyCount > 0) {
        const float lastBpm = bpmHistory[(historyIndex - 1 + historySize) % historySize];
        if (std::fabs(bpm - lastBpm) > HR_SUDDEN_CHANGE) {
            bpm = lastSmoothed + (bpm - lastSmoothed) * 0.3f;
        }
    }

    bpmHistory[historyIndex] = bpm;
    historyIndex = (historyIndex + 1) % historySize;
    if (historyCount < historySize) {
        historyCount++;
    }

    bpmSum += bpm;
    bpmSumSq += bpm * bpm;
    lastSmoothed = alpha * bpm + (1.0f - alpha) * lastSmoothed;
    lastValidBpm = bpm;
    lastValidTime = radar_now_ms();
}

HeartRateData HeartRateProcessor::getData() const
{
    HeartRateData data = {};

    if (historyCount == 0) {
        data.isValid = false;
        return data;
    }

    data.bpm = bpmHistory[(historyIndex - 1 + historySize) % historySize];
    data.bpmSmoothed = lastSmoothed;
    data.bpmMean = bpmSum / historyCount;

    const float variance = (bpmSumSq / historyCount) - (data.bpmMean * data.bpmMean);
    data.bpmStd = variance > 0.0f ? std::sqrt(variance) : 0.0f;

    data.bpmMin = 300.0f;
    data.bpmMax = 0.0f;
    for (int i = 0; i < historyCount; ++i) {
        if (bpmHistory[i] < data.bpmMin) {
            data.bpmMin = bpmHistory[i];
        }
        if (bpmHistory[i] > data.bpmMax) {
            data.bpmMax = bpmHistory[i];
        }
    }

    data.trend = calculateTrend();
    data.variability = calculateVariability();

    const uint64_t age_ms = radar_now_ms() - lastValidTime;
    if (age_ms < 3000ULL) {
        data.quality = 0.8f + 0.2f * (1.0f - data.bpmStd / 30.0f);
        data.quality = constrain_value(data.quality, 0.0f, 1.0f);
    } else {
        data.quality = 0.3f;
    }

    data.isValid = true;
    data.timestamp = radar_now_ms();
    return data;
}

HRVEstimate HeartRateProcessor::estimateHRV() const
{
    HRVEstimate hrv = {};

    if (historyCount < 10) {
        hrv.isValid = false;
        return hrv;
    }

    const HeartRateData hrData = getData();
    const float hrVariability = hrData.bpmStd;

    hrv.rmssd = hrVariability * 8.0f;
    hrv.sdnn = hrVariability * 10.0f;
    hrv.stressIndex = hrv.rmssd > 0.0f ? 1000.0f / hrv.rmssd : 50.0f;

    if (hrv.rmssd > 0.0f) {
        const float normalized = (hrv.rmssd - 20.0f) / 30.0f;
        hrv.autonomicBalance = 0.3f + 0.4f * constrain_value(normalized, 0.0f, 1.0f);
    } else {
        hrv.autonomicBalance = 0.5f;
    }

    hrv.isValid = true;
    return hrv;
}

void HeartRateProcessor::reset()
{
    historyIndex = 0;
    historyCount = 0;
    lastSmoothed = 72.0f;
    bpmSum = 0.0f;
    bpmSumSq = 0.0f;
    lastValidBpm = 72.0f;
    lastValidTime = 0;
    memset(bpmHistory, 0, historySize * sizeof(float));
}

float HeartRateProcessor::calculateVariability() const
{
    if (historyCount < 3) {
        return 0.0f;
    }

    float sumSqDiff = 0.0f;
    int count = 0;

    for (int i = 1; i < historyCount; ++i) {
        const int idx1 = (historyIndex - i - 1 + historySize) % historySize;
        const int idx2 = (historyIndex - i + historySize) % historySize;
        const float diff = bpmHistory[idx1] - bpmHistory[idx2];
        sumSqDiff += diff * diff;
        count++;
    }

    return count > 0 ? std::sqrt(sumSqDiff / count) : 0.0f;
}

float HeartRateProcessor::calculateTrend() const
{
    if (historyCount < 10) {
        return 0.0f;
    }

    const int half = historyCount / 2;
    float firstHalf = 0.0f;
    float secondHalf = 0.0f;

    for (int i = 0; i < half; ++i) {
        const int idx = (historyIndex - historyCount + i + historySize) % historySize;
        firstHalf += bpmHistory[idx];
    }
    firstHalf /= half;

    for (int i = historyCount - half; i < historyCount; ++i) {
        const int idx = (historyIndex - historyCount + i + historySize) % historySize;
        secondHalf += bpmHistory[idx];
    }
    secondHalf /= half;

    return secondHalf - firstHalf;
}

RespirationProcessor::RespirationProcessor(int histSize)
    : rateHistory(new float[histSize]),
      historySize(histSize),
      historyIndex(0),
      historyCount(0),
      lastSmoothed(16.0f),
      alpha(EMA_ALPHA_RR),
      lastValidRate(16.0f),
      lastValidTime(0) {
    memset(rateHistory, 0, historySize * sizeof(float));
}

RespirationProcessor::~RespirationProcessor()
{
    delete[] rateHistory;
}

void RespirationProcessor::addData(float rate, float confidence)
{
    (void)confidence;

    if (rate < RR_MIN_NORMAL || rate > RR_MAX_NORMAL) {
        return;
    }

    if (historyCount > 0) {
        const float lastRate = rateHistory[(historyIndex - 1 + historySize) % historySize];
        if (std::fabs(rate - lastRate) > RR_SUDDEN_CHANGE) {
            rate = lastSmoothed + (rate - lastSmoothed) * 0.3f;
        }
    }

    rateHistory[historyIndex] = rate;
    historyIndex = (historyIndex + 1) % historySize;
    if (historyCount < historySize) {
        historyCount++;
    }

    lastSmoothed = alpha * rate + (1.0f - alpha) * lastSmoothed;
    lastValidRate = rate;
    lastValidTime = radar_now_ms();
}

RespirationData RespirationProcessor::getData() const
{
    RespirationData data = {};

    if (historyCount == 0) {
        data.isValid = false;
        return data;
    }

    data.rate = rateHistory[(historyIndex - 1 + historySize) % historySize];
    data.rateSmoothed = lastSmoothed;

    float sum = 0.0f;
    for (int i = 0; i < historyCount; ++i) {
        sum += rateHistory[i];
    }
    data.rateMean = sum / historyCount;

    float sumSq = 0.0f;
    for (int i = 0; i < historyCount; ++i) {
        const float diff = rateHistory[i] - data.rateMean;
        sumSq += diff * diff;
    }
    data.rateStd = std::sqrt(sumSq / historyCount);
    data.regularity = calculateRegularity();
    data.variability = calculateVariability();

    const uint64_t age_ms = radar_now_ms() - lastValidTime;
    if (age_ms < 5000ULL) {
        data.quality = 0.7f + 0.3f * data.regularity;
        data.quality = constrain_value(data.quality, 0.0f, 1.0f);
    } else {
        data.quality = 0.3f;
    }

    data.isValid = true;
    data.timestamp = radar_now_ms();
    return data;
}

void RespirationProcessor::reset()
{
    historyIndex = 0;
    historyCount = 0;
    lastSmoothed = 16.0f;
    lastValidRate = 16.0f;
    lastValidTime = 0;
    memset(rateHistory, 0, historySize * sizeof(float));
}

float RespirationProcessor::calculateRegularity() const
{
    if (historyCount < 5) {
        return 0.8f;
    }

    float sum = 0.0f;
    for (int i = 0; i < historyCount; ++i) {
        sum += rateHistory[i];
    }
    const float mean = sum / historyCount;

    float sumSq = 0.0f;
    for (int i = 0; i < historyCount; ++i) {
        const float diff = rateHistory[i] - mean;
        sumSq += diff * diff;
    }

    const float stddev = std::sqrt(sumSq / historyCount);
    const float cv = mean > 0.0f ? stddev / mean : 0.0f;
    return constrain_value(1.0f - cv * 3.0f, 0.0f, 1.0f);
}

float RespirationProcessor::calculateVariability() const
{
    if (historyCount < 3) {
        return 0.0f;
    }

    float sumDiff = 0.0f;
    for (int i = 1; i < historyCount; ++i) {
        const int idx1 = (historyIndex - i - 1 + historySize) % historySize;
        const int idx2 = (historyIndex - i + historySize) % historySize;
        sumDiff += std::fabs(rateHistory[idx1] - rateHistory[idx2]);
    }

    return sumDiff / (historyCount - 1);
}

PhysioDataProcessor::PhysioDataProcessor()
    : hrProcessor(new HeartRateProcessor(100)),
      rrProcessor(new RespirationProcessor(50))
{
}

PhysioDataProcessor::~PhysioDataProcessor()
{
    delete hrProcessor;
    delete rrProcessor;
}

void PhysioDataProcessor::update(float hr, float rr, float hrConf, float rrConf)
{
    hrProcessor->addData(hr, hrConf);
    rrProcessor->addData(rr, rrConf);
}

HeartRateData PhysioDataProcessor::getHeartRateData() const
{
    return hrProcessor->getData();
}

RespirationData PhysioDataProcessor::getRespirationData() const
{
    return rrProcessor->getData();
}

HRVEstimate PhysioDataProcessor::getHRVEstimate() const
{
    return hrProcessor->estimateHRV();
}

void PhysioDataProcessor::reset()
{
    hrProcessor->reset();
    rrProcessor->reset();
}
