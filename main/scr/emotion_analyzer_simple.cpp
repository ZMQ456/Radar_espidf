/**
 * @file emotion_analyzer_simple.cpp
 * @brief 简易情绪分析器实现
 * 
 * 功能概述：
 * - 基于生理信号（心率、呼吸率、HRV）分析用户情绪状态
 * - 支持 8 种情绪识别：平静、快乐、兴奋、焦虑、愤怒、悲伤、压力、放松
 * - 提供情绪强度、置信度、价效值、唤醒度等多维度指标
 * 
 * 核心算法：
 * 1. 多特征融合：心率、呼吸率、HRV、体动数据加权评分
 * 2. 高斯函数：计算接近基准值的程度
 * 3. Sigmoid 函数：计算偏离基准值的程度
 * 4. 滑动窗口平滑：5 秒窗口滤波，减少波动
 * 5. 概率归一化：确保所有情绪概率和为 1
 * 6. 自适应平滑：根据变化率动态调整平滑系数
 * 
 * 应用场景：
 * - 睡眠监测：评估情绪状态对睡眠质量的影响
 * - 压力管理：实时监测压力水平并预警
 * - 健康分析：长期追踪情绪变化趋势
 */
// ============================================================================
// 头文件包含和静态常量
// ============================================================================
#include "emotion_analyzer_simple.h"  // 情绪分析器接口声明

#include <algorithm>    // STL 算法（min, max）
#include <cmath>        // 数学函数（exp, sqrt, fabs）
#include <cstring>      // 内存操作（memset）
#include <sstream>      // 字符串流（JSON 输出）

#include "radar_platform.h"  // 平台接口（radar_now_ms, constrain_value）

// 初始化静态常量
const int SimpleEmotionAnalyzer::WINDOW_SIZE;  // 滑动窗口大小（5 秒）

// ============================================================================
// 构造函数和析构函数
// ============================================================================
/**
 * @brief 构造函数：初始化情绪分析器
 * 
 * @param histSize 情绪历史缓冲区大小（默认 100 个点）
 * 
 * 初始化内容：
 * - 分配情绪历史缓冲区
 * - 清零所有状态变量和窗口缓冲区
 * - 设置默认基线值（心率 72bpm，呼吸率 16bpm）
 * - 标记为冷启动状态（需要校准）
 * 
 * 缓冲区初始化：
 * - emotionHistory: 情绪历史记录（histSize 个点）
 * - hrWindow/rrWindow/hrvWindow: 5 秒滑动窗口
 * - emotionProbs/prevProbs: 当前和上一次的情绪概率
 */
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

    // 设置默认基线值（静息状态）
    baseline.hrResting = 72.0f;   // 正常静息心率
    baseline.rrResting = 16.0f;   // 正常静息呼吸率
    baseline.isColdStarting = true;  // 冷启动状态，需要校准
}

SimpleEmotionAnalyzer::~SimpleEmotionAnalyzer()
{
    delete[] emotionHistory;  // 释放情绪历史数组
}

EmotionResult SimpleEmotionAnalyzer::analyze(const HeartRateData &hrData,
                                             const RespirationData &rrData,
                                             const HRVEstimate &hrvData,
                                             const BodyMovementData &movementData)
{
    EmotionResult result = {};
    result.timestamp = radar_now_ms();

    if (!hrData.isValid && !rrData.isValid) {
        return result;  // 数据无效，返回空结果
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

    // 步骤 4: 概率归一化
    normalizeProbabilities();

    // 步骤 5: 增强最高概率情绪
    int maxIdx = 0;
    for (int i = 1; i < 9; ++i) {
        if (emotionProbs[i] > emotionProbs[maxIdx]) {
            maxIdx = i;
        }
    }
    emotionProbs[maxIdx] *= 1.3f;  // 增强 30%
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

    // 步骤 8: 计算情绪强度
    const float hrFactor = std::fabs(hrData.bpmSmoothed - baseline.hrResting) / 40.0f;
    const float hrvFactor = hrvData.isValid ? (1.0f - sigmoid(hrvData.rmssd, 0.02f, 40.0f)) : 0.5f;
    const float rrFactor = rrData.isValid ? std::fabs(rrData.rateSmoothed - baseline.rrResting) / 10.0f : 0.3f;
    result.intensity = 0.4f
        + 0.3f * constrain_value(hrFactor, 0.0f, 1.0f)
        + 0.2f * constrain_value(hrvFactor, 0.0f, 1.0f)
        + 0.1f * constrain_value(rrFactor, 0.0f, 1.0f);
    result.intensity = constrain_value(result.intensity, 0.3f, 1.0f);

    // 步骤 9: 计算多维度指标
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

    // 步骤 10: 更新历史记录
    emotionHistory[historyIndex] = result.primaryEmotion;
    historyIndex = (historyIndex + 1) % historySize;
    if (historyCount < historySize) {
        historyCount++;
    }
    lastResult = result;
    return result;
}

// ============================================================================
// 基线设置和校准
// ============================================================================
/**
 * @brief 设置用户基线参数
 * 
 * @param bl 用户基线结构体（包含静息心率、呼吸率等）
 * 
 * 用途：
 * - 手动设置用户的生理基准值
 * - 用于个性化情绪分析（不同人基线不同）
 * - 覆盖自动校准的结果
 * 
 * 使用示例：
 * @code
 * UserBaseline bl;
 * bl.hrResting = 65.0f;  // 该用户静息心率较低
 * bl.rrResting = 14.0f;  // 该用户呼吸较慢
 * analyzer.setBaseline(bl);
 * @endcode
 */
void SimpleEmotionAnalyzer::setBaseline(const UserBaseline &bl)
{
    baseline = bl;  // 直接赋值
}

/**
 * @brief 校准用户基线（自动学习）
 * 
 * @param hrData 心率数据
 * @param rrData 呼吸率数据
 * @param movementData 体动数据
 * 
 * 校准策略：
 * 1. 冷启动阶段：采集 COLD_START_SAMPLES 个样本（约 10 个），取平均
 * 2. 正常运行阶段：使用指数移动平均（EMA）缓慢更新
 *    - 新值权重：0.3
 *    - 旧值权重：0.7
 * 
 * 校准条件：
 * - 体动较小（< 30），确保处于静息状态
 * - 数据有效
 * 
 * 注意：
 * - 冷启动完成后，isCalibrated 标记为 true
 * - 校准时间戳用于评估基线的时效性
 */
void SimpleEmotionAnalyzer::calibrateBaseline(const HeartRateData &hrData,
                                              const RespirationData &rrData,
                                              const BodyMovementData &movementData)
{
    // 体动过大时不校准（可能是运动状态）
    if (!movementData.isValid || movementData.movementSmoothed > 30.0f) {
        return;
    }

    // 冷启动阶段：采集足够样本
    if (baseline.isColdStarting && baseline.coldStartCount < UserBaseline::COLD_START_SAMPLES) {
        if (hrData.isValid) baseline.coldStartHrSum += hrData.bpmSmoothed;
        if (rrData.isValid) baseline.coldStartRrSum += rrData.rateSmoothed;
        baseline.coldStartCount++;

        // 样本足够，计算平均值
        if (baseline.coldStartCount >= UserBaseline::COLD_START_SAMPLES) {
            if (hrData.isValid && baseline.coldStartHrSum > 0.0f) baseline.hrResting = baseline.coldStartHrSum / baseline.coldStartCount;
            if (rrData.isValid && baseline.coldStartRrSum > 0.0f) baseline.rrResting = baseline.coldStartRrSum / baseline.coldStartCount;
            baseline.isColdStarting = false;
            baseline.isCalibrated = true;
            baseline.calibrationTime = radar_now_ms();
        }
        return;
    }

    // 正常运行阶段：EMA 缓慢更新
    if (hrData.isValid) baseline.hrResting = baseline.hrResting * 0.7f + hrData.bpmSmoothed * 0.3f;
    if (rrData.isValid) baseline.rrResting = baseline.rrResting * 0.7f + rrData.rateSmoothed * 0.3f;
    baseline.isCalibrated = true;
    baseline.calibrationTime = radar_now_ms();
}

/**
 * @brief 获取最近 N 秒的主导情绪
 * 
 * @param seconds 时间范围（秒）
 * @return EmotionType 出现频率最高的情绪
 * 
 * 算法：
 * 1. 遍历历史缓冲区（最多 seconds/2 个条目）
 * 2. 统计每种情绪的出现次数
 * 3. 返回出现次数最多的情绪
 * 
 * 用途：
 * - 评估一段时间内的整体情绪状态
 * - 消除瞬时波动的影响
 * - 用于趋势分析和报告生成
 * 
 * 示例：
 * - getRecentDominantEmotion(60)：获取最近 1 分钟的主导情绪
 */
EmotionType SimpleEmotionAnalyzer::getRecentDominantEmotion(int seconds)
{
    int counts[9] = {0};  // 9 种情绪的计数器
    const int entries = std::min(historyCount, seconds / 2);  // 采样点数
    
    // 统计历史情绪
    for (int i = 0; i < entries; ++i) {
        const int idx = (historyIndex - 1 - i + historySize) % historySize;
        counts[emotionHistory[idx]]++;
    }

    // 找出出现次数最多的情绪
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

/**
 * @brief 重置分析器状态
 * 
 * 清空内容：
 * - 情绪概率数组
 * - 历史缓冲区
 * - 滑动窗口（HR/RR/HRV）
 * - 上一次结果
 * - 所有索引和计数器
 * 
 * 用途：
 * - 切换被测对象
 * - 重新开始分析
 * - 清除异常状态
 */
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

// ============================================================================
// 情绪评分计算函数（8 种情绪）
// ============================================================================
/**
 * @brief 计算"平静"情绪得分
 * 
 * @param hr 心率数据
 * @param rr 呼吸率数据
 * @param hrv HRV 数据
 * @param movement 体动数据
 * @return float 平静得分（0.0-1.0）
 * 
 * 评分标准：
 * - 心率接近基线（权重 0.45）：高斯函数，均值 0，标准差 0.3
 * - 呼吸率接近基线（权重 0.27）+ 规律性（权重 0.16）
 * - HRV 较高（权重 0.10）：Sigmoid 函数
 * - 体动较小（权重 0.08）：高斯函数，峰值 0.15
 * 
 * 特征：
 * - 各项生理指标接近静息状态
 * - 呼吸规律
 * - 身体静止
 */
float SimpleEmotionAnalyzer::calculateCalmScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.45f * gaussian(std::fabs(normalizeHR(hr.bpmSmoothed, baseline.hrResting)), 0.0f, 0.3f);
    if (rr.isValid) score += 0.27f * gaussian(std::fabs(normalizeRR(rr.rateSmoothed, baseline.rrResting)), 0.0f, 0.5f) + 0.16f * rr.regularity;
    if (hrv.isValid) score += 0.10f * sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.7f);
    if (movement.isValid) score += 0.08f * gaussian(normalizeMovement(movement.movementSmoothed), 0.15f, 0.2f);
    return constrain_value(score, 0.0f, 1.0f);
}

/**
 * @brief 计算"快乐"情绪得分
 * 
 * @param hr 心率数据
 * @param rr 呼吸率数据
 * @param hrv HRV 数据
 * @param movement 体动数据
 * @return float 快乐得分（0.0-1.0）
 * 
 * 评分标准：
 * - 心率略高于基线（权重 0.45）：高斯函数，中心 0.3（适度升高）
 * - 呼吸率接近基线（权重 0.34）：高斯函数
 * - HRV 较高（权重 0.10）：Sigmoid 函数
 * - 体动适中（权重 0.08）：高斯函数，峰值 0.45
 * 
 * 特征：
 * - 心率轻微上升（积极情绪）
 * - 呼吸平稳
 * - 可能有轻微体动（如微笑、小动作）
 */
float SimpleEmotionAnalyzer::calculateHappyScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.45f * gaussian(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 0.3f, 0.25f);
    if (rr.isValid) score += 0.34f * gaussian(std::fabs(normalizeRR(rr.rateSmoothed, baseline.rrResting)), 0.0f, 0.5f);
    if (hrv.isValid) score += 0.10f * sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.9f);
    if (movement.isValid) score += 0.08f * gaussian(normalizeMovement(movement.movementSmoothed), 0.45f, 0.25f);
    return constrain_value(score, 0.0f, 1.0f);
}

/**
 * @brief 计算"兴奋"情绪得分
 * 
 * @param hr 心率数据
 * @param rr 呼吸率数据
 * @param hrv HRV 数据
 * @param movement 体动数据
 * @return float 兴奋得分（0.0-1.0）
 * 
 * 评分标准：
 * - 心率明显升高（权重 0.45）：Sigmoid 函数，快速上升
 * - 呼吸率升高（权重 0.34）：Sigmoid 函数
 * - HRV 降低（权重 0.10）：高斯函数，中心 0.7
 * - 体动增加（权重 0.10）：Sigmoid 函数
 * 
 * 特征：
 * - 心率和呼吸都加快
 * - 交感神经兴奋（HRV 降低）
 * - 身体活动增加
 */
float SimpleEmotionAnalyzer::calculateExcitedScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.45f * sigmoid(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.6f);
    if (rr.isValid) score += 0.34f * sigmoid(normalizeRR(rr.rateSmoothed, baseline.rrResting), 2.0f, 0.5f);
    if (hrv.isValid) score += 0.10f * gaussian(normalizeHRV(hrv.rmssd), 0.7f, 0.4f);
    if (movement.isValid) score += 0.10f * sigmoid(normalizeMovement(movement.movementSmoothed), 2.0f, 0.6f);
    return constrain_value(score, 0.0f, 1.0f);
}

/**
 * @brief 计算"焦虑"情绪得分
 * 
 * @param hr 心率数据
 * @param rr 呼吸率数据
 * @param hrv HRV 数据
 * @param movement 体动数据
 * @return float 焦虑得分（0.0-1.0）
 * 
 * 评分标准：
 * - 心率升高（权重 0.36）：Sigmoid 函数，阈值 0.4
 * - HRV 降低（权重 0.25）：1 - Sigmoid（HRV 越低分越高）
 * - 呼吸不规律（权重 0.16）：1 - regularity
 * - 体动增加（权重 0.10）：高斯函数，峰值 0.55
 * 
 * 特征：
 * - 心率加快
 * - HRV 降低（压力反应）
 * - 呼吸不规律
 * - 坐立不安
 */
float SimpleEmotionAnalyzer::calculateAnxiousScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.36f * sigmoid(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.4f);
    if (hrv.isValid) score += 0.25f * (1.0f - sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.6f));
    if (rr.isValid) score += 0.16f * (1.0f - rr.regularity);
    if (movement.isValid) score += 0.10f * gaussian(normalizeMovement(movement.movementSmoothed), 0.55f, 0.3f);
    return constrain_value(score, 0.0f, 1.0f);
}

/**
 * @brief 计算"愤怒"情绪得分
 * 
 * @param hr 心率数据
 * @param rr 呼吸率数据
 * @param hrv HRV 数据
 * @param movement 体动数据
 * @return float 愤怒得分（0.0-1.0）
 * 
 * 评分标准：
 * - 心率明显升高（权重 0.36）：Sigmoid 函数，阈值 0.75
 * - HRV 降低（权重 0.25）：1 - Sigmoid
 * - 呼吸不规律（权重 0.20）：1 - regularity
 * - 体动增加（权重 0.10）：Sigmoid 函数
 * 
 * 特征：
 * - 心率显著加快（比焦虑更高）
 * - HRV 降低
 * - 呼吸急促不规律
 * - 身体动作幅度大
 */
float SimpleEmotionAnalyzer::calculateAngryScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.36f * sigmoid(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.75f);
    if (hrv.isValid) score += 0.25f * (1.0f - sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.5f));
    if (rr.isValid) score += 0.20f * (1.0f - rr.regularity);
    if (movement.isValid) score += 0.10f * sigmoid(normalizeMovement(movement.movementSmoothed), 2.0f, 0.7f);
    return constrain_value(score, 0.0f, 1.0f);
}

/**
 * @brief 计算"悲伤"情绪得分
 * 
 * @param hr 心率数据
 * @param rr 呼吸率数据
 * @param hrv HRV 数据
 * @param movement 体动数据
 * @return float 悲伤得分（0.0-1.0）
 * 
 * 评分标准：
 * - 心率降低（权重 0.40）：Sigmoid 函数（负值，越低分越高）
 * - 呼吸率降低（权重 0.28）：Sigmoid 函数（负值）
 * - HRV 降低（权重 0.10）：1 - Sigmoid
 * - 体动减少（权重 0.10）：1 - Sigmoid
 * 
 * 特征：
 * - 心率低于基线
 * - 呼吸缓慢
 * - 活动减少
 */
float SimpleEmotionAnalyzer::calculateSadScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.40f * sigmoid(-normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.2f);
    if (rr.isValid) score += 0.28f * sigmoid(-normalizeRR(rr.rateSmoothed, baseline.rrResting), 2.0f, 0.3f);
    if (hrv.isValid) score += 0.10f * (1.0f - sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.9f));
    if (movement.isValid) score += 0.10f * (1.0f - sigmoid(normalizeMovement(movement.movementSmoothed), 2.0f, 0.2f));
    return constrain_value(score, 0.0f, 1.0f);
}

/**
 * @brief 计算"压力"情绪得分
 * 
 * @param hr 心率数据
 * @param rr 呼吸率数据
 * @param hrv HRV 数据
 * @param movement 体动数据
 * @return float 压力得分（0.0-1.0）
 * 
 * 评分标准：
 * - 心率升高（权重 0.36）：Sigmoid 函数
 * - HRV 降低（权重 0.34）：1 - Sigmoid（主要指标）
 * - 呼吸率升高（权重 0.20）：Sigmoid 函数
 * - 体动增加（权重 0.08）：高斯函数
 * 
 * 特征：
 * - 交感神经兴奋（HRV 显著降低）
 * - 心率和呼吸加快
 * - 可能有紧张性体动
 */
float SimpleEmotionAnalyzer::calculateStressedScore(const HeartRateData &hr, const RespirationData &rr, const HRVEstimate &hrv, const BodyMovementData &movement)
{
    float score = 0.0f;
    if (hr.isValid) score += 0.36f * sigmoid(normalizeHR(hr.bpmSmoothed, baseline.hrResting), 2.0f, 0.4f);
    if (hrv.isValid) score += 0.34f * (1.0f - sigmoid(normalizeHRV(hrv.rmssd), 2.0f, 0.6f));
    if (rr.isValid) score += 0.20f * sigmoid(normalizeRR(rr.rateSmoothed, baseline.rrResting), 2.0f, 0.5f);
    if (movement.isValid) score += 0.08f * gaussian(normalizeMovement(movement.movementSmoothed), 0.4f, 0.25f);
    return constrain_value(score, 0.0f, 1.0f);
}

/**
 * @brief 计算"放松"情绪得分
 * 
 * @param hr 心率数据
 * @param rr 呼吸率数据
 * @param hrv HRV 数据
 * @param movement 体动数据
 * @return float 放松得分（0.0-1.0）
 * 
 * 评分标准：
 * - 心率降低（权重 0.31）：Sigmoid 函数（负值）
 * - HRV 升高（权重 0.28）：Sigmoid 函数（HRV 越高分越高）
 * - 呼吸率降低（权重 0.16）+ 规律性（权重 0.10）
 * - 体动减少（权重 0.10）：1 - Sigmoid
 * 
 * 特征：
 * - 心率低于基线
 * - HRV 升高（副交感神经活跃）
 * - 呼吸缓慢且规律
 * - 身体静止
 */
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
