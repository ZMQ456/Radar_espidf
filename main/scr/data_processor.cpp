/**
 * @file data_processor.cpp
 * @brief 生理信号数据处理模块
 * 
 * 功能概述：
 * - 心率（Heart Rate）处理：EMA 平滑、趋势分析、变异性计算
 * - 呼吸率（Respiration Rate）处理：平滑、规律性分析
 * - HRV（心率变异性）估算：压力指数、自主神经平衡
 * 
 * 核心算法：
 * 1. EMA（指数移动平均）：平滑突变，提取趋势
 * 2. 环形缓冲区：保存历史数据，支持统计分析
 * 3. 突变检测：过滤异常跳变，保持连续性
 * 4. 质量评估：基于时间衰减和标准差的数据可信度
 */
#include "data_processor.h"

#include <cmath>
#include <cstring>

#include "radar_platform.h"

/**
 * @brief 构造函数：初始化心率处理器
 * 
 * @param histSize 历史数据缓冲区大小（默认 100 个点）
 * 
 * 初始化内容：
 * - 分配环形缓冲区（bpmHistory）
 * - 设置 EMA 平滑系数（alpha = EMA_ALPHA_HR，约 0.3）
 * - 初始化统计变量（sum, sumSq）
 * - 设置默认值（72 bpm 作为初始值）
 */
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

/**
 * @brief 析构函数：释放动态分配的缓冲区
 */
HeartRateProcessor::~HeartRateProcessor()
{
    delete[] bpmHistory;
}

/**
 * @brief 添加新的心率数据点
 * 
 * @param bpm 原始心率值（次/分钟）
 * @param confidence 置信度（未使用，保留接口）
 * 
 * 处理流程：
 * 1. 有效性检查：过滤超出正常范围的值（HR_MIN_NORMAL ~ HR_MAX_NORMAL）
 * 2. 突变检测：如果相对上次值变化超过 HR_SUDDEN_CHANGE，进行平滑处理
 * 3. 写入环形缓冲区：循环覆盖最旧的数据
 * 4. 更新统计量：累加和、平方和
 * 5. EMA 平滑：提取趋势分量
 * 6. 记录时间戳：用于后续质量评估
 */
void HeartRateProcessor::addData(float bpm, float confidence)
{
    (void)confidence;

    // 步骤 1: 有效性检查 - 过滤异常值
    if (bpm < HR_MIN_NORMAL || bpm > HR_MAX_NORMAL) {
        return;
    }

    // 步骤 2: 突变检测 - 防止跳变
    if (historyCount > 0) {
        const float lastBpm = bpmHistory[(historyIndex - 1 + historySize) % historySize];
        if (std::fabs(bpm - lastBpm) > HR_SUDDEN_CHANGE) {
            bpm = lastSmoothed + (bpm - lastSmoothed) * 0.3f;
        }
    }

    // 步骤 3: 写入环形缓冲区
    bpmHistory[historyIndex] = bpm;
    historyIndex = (historyIndex + 1) % historySize;
    if (historyCount < historySize) {
        historyCount++;
    }

    // 步骤 4: 更新统计量
    bpmSum += bpm;
    bpmSumSq += bpm * bpm;
    
    // 步骤 5: EMA 平滑
    lastSmoothed = alpha * bpm + (1.0f - alpha) * lastSmoothed;
    
    // 步骤 6: 记录有效值和时间戳
    lastValidBpm = bpm;
    lastValidTime = radar_now_ms();
}

/**
 * @brief 获取当前心率数据统计结果
 * 
 * @return HeartRateData 包含完整统计信息的心率数据结构
 * 
 * 计算内容：
 * - 瞬时值：最新的心率点
 * - 平滑值：EMA 滤波后的趋势
 * - 均值/标准差：历史数据的统计特征
 * - 最大/最小值：历史范围内的极值
 * - 趋势：前后半段均值差（上升/下降趋势）
 * - 变异性：相邻点差分的 RMS
 * - 质量分数：基于时间衰减和标准差的可信度
 */
HeartRateData HeartRateProcessor::getData() const
{
    HeartRateData data = {};

    // 边界检查：无数据时返回无效标记
    if (historyCount == 0) {
        data.isValid = false;
        return data;
    }

    // 瞬时值：环形缓冲区最后一个有效点
    data.bpm = bpmHistory[(historyIndex - 1 + historySize) % historySize];
    data.bpmSmoothed = lastSmoothed;
    data.bpmMean = bpmSum / historyCount;

    // 标准差：sqrt(E[X²] - E[X]²)
    const float variance = (bpmSumSq / historyCount) - (data.bpmMean * data.bpmMean);
    data.bpmStd = variance > 0.0f ? std::sqrt(variance) : 0.0f;

    // 遍历缓冲区，找出最大/最小值
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

    // 趋势分析和变异性计算
    data.trend = calculateTrend();
    data.variability = calculateVariability();

    // 质量评估：基于时间衰减和标准差
    const uint64_t age_ms = radar_now_ms() - lastValidTime;
    if (age_ms < 3000ULL) {
        // 3 秒内：基础质量 0.8 + 标准差调整 0.2
        data.quality = 0.8f + 0.2f * (1.0f - data.bpmStd / 30.0f);
        data.quality = constrain_value(data.quality, 0.0f, 1.0f);
    } else {
        // 超过 3 秒未更新：质量衰减到 0.3
        data.quality = 0.3f;
    }

    data.isValid = true;
    data.timestamp = radar_now_ms();
    return data;
}

/**
 * @brief 估算 HRV（心率变异性）指标
 * 
 * @return HRVEstimate HRV 估算结果
 * 
 * HRV 指标说明：
 * - RMSSD：相邻 RR 间期差值的均方根（正常范围 20-50ms）
 * - SDNN：NN 间期标准差（反映整体变异性）
 * - Stress Index：压力指数（RMSSD 的倒数，越高越紧张）
 * - Autonomic Balance：自主神经平衡（0-1，0.5 为平衡状态）
 * 
 * 注意：这是基于心率数据的简化估算，非医疗级 HRV 测量
 */
HRVEstimate HeartRateProcessor::estimateHRV() const
{
    HRVEstimate hrv = {};

    // 至少需要 10 个数据点才能进行 HRV 分析
    if (historyCount < 10) {
        hrv.isValid = false;
        return hrv;
    }

    // 获取当前心率统计数据
    const HeartRateData hrData = getData();
    const float hrVariability = hrData.bpmStd;

    // 基于心率变异性估算 HRV 指标（经验公式）
    hrv.rmssd = hrVariability * 8.0f;
    hrv.sdnn = hrVariability * 10.0f;
    hrv.stressIndex = hrv.rmssd > 0.0f ? 1000.0f / hrv.rmssd : 50.0f;

    // 自主神经平衡：基于 RMSSD 的归一化值
    if (hrv.rmssd > 0.0f) {
        const float normalized = (hrv.rmssd - 20.0f) / 30.0f;
        hrv.autonomicBalance = 0.3f + 0.4f * constrain_value(normalized, 0.0f, 1.0f);
    } else {
        hrv.autonomicBalance = 0.5f;
    }

    hrv.isValid = true;
    return hrv;
}

/**
 * @brief 重置处理器状态
 * 
 * 清空所有历史数据和统计量，恢复到初始状态
 * 用于重新开始测量或切换被测对象
 */
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

/**
 * @brief 计算心率变异性（相邻点差分的 RMS）
 * 
 * @return float 变异性指标（单位：bpm）
 * 
 * 算法：
 * 1. 遍历所有相邻点对
 * 2. 计算差值的平方和
 * 3. 求均方根（RMS）
 * 
 * 意义：反映心率的短期波动程度
 */
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

/**
 * @brief 计算心率趋势（前后半段均值对比）
 * 
 * @return float 趋势值（bpm）
 *         > 0：心率上升
 *         < 0：心率下降
 *         ≈ 0：保持稳定
 * 
 * 算法：
 * 1. 将历史数据分为前后两半
 * 2. 分别计算均值
 * 3. 趋势 = 后半段均值 - 前半段均值
 */
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
