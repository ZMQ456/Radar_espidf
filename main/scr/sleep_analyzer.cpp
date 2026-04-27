/**
 * @file sleep_analyzer.cpp
 * @brief 睡眠分析器模块
 * 
 * 功能概述：
 * - 基于多模态生理数据（心率、呼吸率、HRV、体动）进行睡眠分期
 * - 实时计算睡眠状态（清醒、浅睡、深睡、REM、离床等）
 * - 评估睡眠质量（睡眠评分、睡眠周期、睡眠潜伏期）
 * - 提供睡眠统计信息（各阶段时长、觉醒次数、睡眠效率）
 * 
 * 睡眠状态（9 种）：
 * 1. SLEEP_NO_PERSON: 无人
 * 2. SLEEP_IN_BED: 在床上（未入睡）
 * 3. SLEEP_AWAKE: 清醒
 * 4. SLEEP_LIGHT_SLEEP: 浅睡
 * 5. SLEEP_DEEP_SLEEP: 深睡
 * 6. SLEEP_REM_SLEEP: 快速眼动睡眠
 * 7. SLEEP_OUT_OF_BED: 离床
 * 8. SLEEP_GETTING_UP: 起床中
 * 9. SLEEP_SESSION_END: 睡眠结束
 * 
 * 核心算法：
 * 1. 基线校准：动态校准个人心率、呼吸率基线
 * 2. 睡眠评分：计算各睡眠状态的置信度分数
 * 3. 状态机：基于评分和滞回比较器进行状态转换
 * 4. 平滑滤波：EMA（指数移动平均）平滑评分
 * 5. 时间确认：状态转换需要持续确认时间
 * 
 * 评分维度：
 * - 困倦度评分：心率降低、HRV 升高、呼吸稳定、体动减少
 * - 深睡评分：心率最低、HRV 最高、体动最少
 * - 浅睡评分：心率中等、HRV 中等、体动适中
 * - 清醒评分：心率升高、体动增加、呼吸变异性增加
 * - REM 评分：心率升高、HRV 降低、呼吸不稳定、体动少
 * 
 * 依赖模块：
 * - radar_platform.h：时间函数（radar_now_ms）
 * - 生理数据：HeartRateData、RespirationData、HRVEstimate、BodyMovementData
 */
#include "sleep_analyzer.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

#include "radar_platform.h"

// ============================================================================
// 常量定义
// ============================================================================
// 阈值参数
const float SleepAnalyzer::SLEEPINESS_THRESHOLD = 0.6f;              // 困倦度阈值（超过则进入浅睡）
const float SleepAnalyzer::BASELINE_MOVEMENT_THRESHOLD = 0.2f;       // 基线校准时的最大体动阈值
const float SleepAnalyzer::BASELINE_HR_STABILITY_THRESHOLD = 5.0f;   // 心率稳定性阈值
const float SleepAnalyzer::BASELINE_RR_STABILITY_THRESHOLD = 2.0f;   // 呼吸率稳定性阈值
const float SleepAnalyzer::EMA_ALPHA = 0.2f;                         // EMA 平滑系数（0.2=20% 新数据）
const float SleepAnalyzer::CONFIDENCE_MARGIN = 0.1f;                 // 置信度边缘（最佳评分的领先优势）
const float SleepAnalyzer::BASELINE_BETA = 0.01f;                    // 基线更新系数（1% 新数据）

// 滞回比较器阈值（防止状态频繁切换）
const float SleepAnalyzer::HYSTERESIS_ENTER_DEEP = 0.7f;             // 进入深睡的评分阈值
const float SleepAnalyzer::HYSTERESIS_EXIT_DEEP = 0.5f;              // 退出深睡的评分阈值
const float SleepAnalyzer::HYSTERESIS_ENTER_REM = 0.6f;              // 进入 REM 的评分阈值
const float SleepAnalyzer::HYSTERESIS_EXIT_REM = 0.4f;               // 退出 REM 的评分阈值

// 睡眠状态名称表（用于日志输出）
static const char *const SLEEP_STATE_NAMES[] = {
    "no_person",       // 0: 无人
    "in_bed",          // 1: 在床上
    "awake",           // 2: 清醒
    "light_sleep",     // 3: 浅睡
    "deep_sleep",      // 4: 深睡
    "rem_sleep",       // 5: REM 睡眠
    "out_of_bed",      // 6: 离床
    "getting_up",      // 7: 起床中
    "session_end"      // 8: 睡眠结束
};

// ============================================================================
// 构造函数和初始化
// ============================================================================
/**
 * @brief 构造函数
 * 
 * 处理流程：
 * 1. 调用 reset() 初始化所有状态和统计
 */
SleepAnalyzer::SleepAnalyzer()
{
    reset();
}

/**
 * @brief 重置所有状态和统计
 * 
 * 处理流程：
 * 1. 重置睡眠状态（currentState, pendingState）
 * 2. 清零统计数据（stats）
 * 3. 清零评分（score）
 * 4. 清零睡眠周期（cycle）
 * 5. 重置时间戳和计时器
 * 6. 重置基线参数（HR, RR, 校准状态）
 * 7. 重置当前评分和中间变量
 * 
 * 用途：
 * - 对象创建时初始化
 * - 手动重置分析器
 * - 开始新的睡眠会话
 */
void SleepAnalyzer::reset()
{
    // 步骤 1: 重置睡眠状态
    currentState = SLEEP_NO_PERSON;
    pendingState = SLEEP_NO_PERSON;
    
    // 步骤 2-4: 清零统计、评分、周期
    memset(&stats, 0, sizeof(stats));
    memset(&score, 0, sizeof(score));
    memset(&cycle, 0, sizeof(cycle));

    // 步骤 5: 重置时间戳和计时器
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

    // 步骤 6: 重置基线参数
    baselineHR = 70.0f;       // 默认心率基线 70 bpm
    baselineRR = 16.0f;       // 默认呼吸率基线 16 rpm
    baselineCalibrated = false;
    baselineSampleCount = 0;
    baselineHRSum = 0.0f;
    baselineRRSum = 0.0f;
    lastBaselineHR = 0.0f;
    lastBaselineRR = 0.0f;
    hrStabilitySum = 0.0f;
    rrStabilitySum = 0.0f;
    stabilitySampleCount = 0;

    // 步骤 7: 重置当前评分和中间变量
    currentSleepiness = 0.0f;
    currentDeepScore = 0.0f;
    currentLightScore = 0.0f;
    currentAwakeScore = 0.0f;
    currentRemScore = 0.0f;
    lastRRValue = 0.0f;
    wasAsleep = false;
}

// ============================================================================
// 辅助函数
// ============================================================================
/**
 * @brief 评估人体存在状态
 * 
 * @param hrData 心率数据
 * @param rrData 呼吸率数据
 * @param movementData 体动数据
 * @return PresenceData 存在检测结果
 * 
 * 判断逻辑：
 * 1. 如果心率或呼吸率有效 → 有人（置信度 0.9）
 * 2. 如果体动 > 5 → 有人（置信度 0.7）
 * 3. 否则 → 无人
 * 
 * 返回内容：
 * - isPresent: 是否有人
 * - confidence: 置信度（0.0-1.0）
 * - distance: 距离（未使用，默认 -1）
 * - motionEnergy: 运动能量（体动活动水平）
 */
PresenceData SleepAnalyzer::evaluatePresence(const HeartRateData &hrData,
                                             const RespirationData &rrData,
                                             const BodyMovementData &movementData) const
{
    PresenceData p = {};
    p.distance = -1.0f;  // 距离未使用
    p.motionEnergy = movementData.isValid ? movementData.activityLevel : 0.0f;

    // 判断 1: 心率或呼吸率有效 → 有人
    if (hrData.isValid || rrData.isValid) {
        p.isPresent = true;
        p.confidence = 0.9f;  // 高置信度
    }

    // 判断 2: 体动 > 5 → 有人
    if (movementData.isValid && movementData.movement > 5) {
        p.isPresent = true;
        p.confidence = std::max(p.confidence, 0.7f);  // 中等置信度
    }

    return p;
}

/**
 * @brief Sigmoid 激活函数
 * 
 * @param x 输入值
 * @return float sigmoid(x) = 1 / (1 + e^(-x))
 * 
 * 特性：
 * - 输出范围：(0, 1)
 * - 中心点：x=0 时输出 0.5
 * - 单调递增
 * 
 * 用途：
 * - 将线性组合映射到 (0, 1) 区间
 * - 用于睡眠评分计算
 */
float SleepAnalyzer::sigmoid(float x) const
{
    return 1.0f / (1.0f + std::exp(-x));
}

/**
 * @brief 指数移动平均（EMA）平滑
 * 
 * @param input 新输入值
 * @param last 上次平滑值
 * @param alpha 平滑系数（0.0-1.0）
 * @return float 平滑后的值
 * 
 * 公式：
 * output = alpha * input + (1 - alpha) * last
 * 
 * 特性：
 * - alpha 越大，越偏向新数据（响应快，噪声大）
 * - alpha 越小，越偏向旧数据（响应慢，平滑好）
 * - 本系统 alpha=0.2（20% 新数据，80% 历史）
 * 
 * 用途：
 * - 平滑睡眠评分，避免剧烈波动
 * - 平滑困倦度、深睡、浅睡、清醒、REM 评分
 */
float SleepAnalyzer::emaSmooth(float input, float last, float alpha) const
{
    return alpha * input + (1.0f - alpha) * last;
}

/**
 * @brief 基线更新函数
 * 
 * @param current 当前基线值
 * @param input 新输入值
 * @param beta 更新系数（0.0-1.0）
 * @return float 更新后的基线值
 * 
 * 公式：
 * newBaseline = (1 - beta) * current + beta * input
 * 
 * 特性：
 * - beta 越小，基线越稳定（更新慢）
 * - beta 越大，基线越敏感（更新快）
 * - 本系统 beta=0.01（1% 新数据，99% 历史）
 * 
 * 用途：
 * - 动态校准个人心率、呼吸率基线
 * - 适应个体差异和长期变化
 */
float SleepAnalyzer::updateBaseline(float current, float input, float beta) const
{
    return (1.0f - beta) * current + beta * input;
}

/**
 * @brief 判断是否为最佳评分
 * 
 * @param scoreValue 当前评分
 * @param s2 第 2 个评分
 * @param s3 第 3 个评分
 * @param s4 第 4 个评分
 * @param margin 置信度边缘
 * @return true 当前评分是最佳（领先所有其他评分至少 margin）
 * @return false 不是最佳
 * 
 * 判断条件：
 * scoreValue > s2 + margin &&
 * scoreValue > s3 + margin &&
 * scoreValue > s4 + margin
 * 
 * 用途：
 * - 判断哪个睡眠状态的评分最高
 * - 确保最佳评分有明显优势（避免误判）
 */
bool SleepAnalyzer::isBestScore(float scoreValue, float s2, float s3, float s4, float margin) const
{
    return (scoreValue > s2 + margin && scoreValue > s3 + margin && scoreValue > s4 + margin);
}

/**
 * @brief 归一化心率（相对于基线）
 * 
 * @param hr 心率值（bpm）
 * @return float 归一化值（-1.0 到 1.0）
 * 
 * 公式：
 * normalized = (hr - baselineHR) / 20.0
 * 
 * 含义：
 * - 0.0: 等于基线
 * - 1.0: 比基线高 20 bpm 以上
 * - -1.0: 比基线低 20 bpm 以下
 * 
 * 用途：
 * - 睡眠评分计算（心率降低→困倦/深睡）
 * - 消除个体差异（使用相对值）
 */
float SleepAnalyzer::normalizeHR(float hr) const
{
    if (!baselineCalibrated) return 0.0f;  // 未校准则返回 0
    return constrain_value((hr - baselineHR) / 20.0f, -1.0f, 1.0f);
}

/**
 * @brief 归一化呼吸率（相对于基线）
 * 
 * @param rr 呼吸率值（rpm）
 * @return float 归一化值（-1.0 到 1.0）
 * 
 * 公式：
 * normalized = (rr - baselineRR) / 4.0
 * 
 * 含义：
 * - 0.0: 等于基线
 * - 1.0: 比基线高 4 rpm 以上
 * - -1.0: 比基线低 4 rpm 以下
 * 
 * 用途：
 * - 睡眠评分计算（呼吸稳定→困倦/深睡）
 */
float SleepAnalyzer::normalizeRR(float rr) const
{
    if (!baselineCalibrated) return 0.0f;  // 未校准则返回 0
    return constrain_value((rr - baselineRR) / 4.0f, -1.0f, 1.0f);
}

/**
 * @brief 归一化 HRV（心率变异性）
 * 
 * @param hrv RMSSD 值（ms）
 * @return float 归一化值（0.0 到 1.0）
 * 
 * 公式：
 * normalized = hrv / 50.0
 * 
 * 含义：
 * - 0.0: HRV 为 0（极低）
 * - 1.0: HRV >= 50ms（很高）
 * - 典型范围：0.2-0.8
 * 
 * 用途：
 * - 睡眠评分计算（HRV 升高→困倦/深睡）
 */
float SleepAnalyzer::normalizeHRV(float hrv) const
{
    return constrain_value(hrv / 50.0f, 0.0f, 1.0f);
}

/**
 * @brief 归一化体动
 * 
 * @param movement 体动值（0-100）
 * @return float 归一化值（0.0 到 1.0）
 * 
 * 公式：
 * normalized = movement / 100.0
 * 
 * 含义：
 * - 0.0: 无体动
 * - 1.0: 最大体动（100）
 * 
 * 用途：
 * - 睡眠评分计算（体动减少→困倦/深睡）
 */
float SleepAnalyzer::normalizeMovement(float movement) const
{
    return constrain_value(movement / 100.0f, 0.0f, 1.0f);
}

/**
 * @brief 校准生理数据基线
 * 
 * @param hrData 心率数据
 * @param rrData 呼吸率数据
 * @param movementData 体动数据
 * 
 * 功能概述：
 * - 在清醒/静卧状态下采集个人基线数据
 * - 动态更新心率和呼吸率基线
 * - 确保基线稳定性和准确性
 * 
 * 校准条件（必须同时满足）：
 * 1. 心率和呼吸率数据有效
 * 2. 当前状态为清醒（SLEEP_AWAKE）或静卧（SLEEP_IN_BED）
 * 3. 体动低于阈值（< 0.2，表示静止状态）
 * 
 * 校准流程：
 * 【已校准状态】
 * 1. 检查稳定性：计算心率和呼吸率的变化量
 * 2. 累积 5 个样本后评估稳定性
 * 3. 如果平均变化量 > 阈值 → 不稳定，跳过本次更新
 * 4. 如果稳定 → 使用 EMA 更新基线（beta=0.01）
 * 
 * 【未校准状态】
 * 1. 累积 30 个样本
 * 2. 计算平均值作为初始基线
 * 3. 标记为已校准
 * 
 * 设计思想：
 * - 初始校准：30 个样本取平均（约 30 秒）
 * - 动态更新：EMA 平滑更新（1% 新数据）
 * - 稳定性检查：确保基线不受短暂波动影响
 * - 状态依赖：只在清醒/静卧时校准（排除睡眠状态）
 * - 体动过滤：只在静止时校准（排除运动干扰）
 * 
 * 注意事项：
 * - 基线值影响所有归一化计算
 * - 基线不准确会导致睡眠评分偏差
 * - 需要用户保持清醒和静止约 30 秒完成初始校准
 */
void SleepAnalyzer::calibrateBaseline(const HeartRateData &hrData,
                                      const RespirationData &rrData,
                                      const BodyMovementData &movementData)
{
    // 前置条件检查
    if (!hrData.isValid || !rrData.isValid) return;  // 数据无效，跳过
    if (currentState != SLEEP_AWAKE && currentState != SLEEP_IN_BED) return;  // 非清醒/静卧状态，跳过
    if (movementData.isValid && normalizeMovement(movementData.movement) > BASELINE_MOVEMENT_THRESHOLD) return;  // 体动过大，跳过

    // 情况 1: 已校准状态
    if (baselineCalibrated) {
        // 步骤 1: 计算稳定性指标
        const float hrDiff = std::fabs(hrData.bpmSmoothed - lastBaselineHR);
        const float rrDiff = std::fabs(rrData.rateSmoothed - lastBaselineRR);
        hrStabilitySum += hrDiff;
        rrStabilitySum += rrDiff;
        stabilitySampleCount++;

        // 步骤 2: 每 5 个样本评估一次稳定性
        if (stabilitySampleCount >= 5) {
            const float avgHrDiff = hrStabilitySum / stabilitySampleCount;
            const float avgRrDiff = rrStabilitySum / stabilitySampleCount;
            hrStabilitySum = 0.0f;
            rrStabilitySum = 0.0f;
            stabilitySampleCount = 0;
            
            // 步骤 3: 稳定性检查
            if (avgHrDiff > BASELINE_HR_STABILITY_THRESHOLD || 
                avgRrDiff > BASELINE_RR_STABILITY_THRESHOLD) {
                return;  // 不稳定，跳过本次更新
            }
        }

        // 步骤 4: 使用 EMA 更新基线（beta=0.01）
        baselineHR = updateBaseline(baselineHR, hrData.bpmSmoothed, BASELINE_BETA);
        baselineRR = updateBaseline(baselineRR, rrData.rateSmoothed, BASELINE_BETA);
        lastBaselineHR = hrData.bpmSmoothed;
        lastBaselineRR = rrData.rateSmoothed;
        return;
    }

    // 情况 2: 未校准状态（初始校准）
    baselineHRSum += hrData.bpmSmoothed;
    baselineRRSum += rrData.rateSmoothed;
    baselineSampleCount++;

    // 步骤 1: 累积 30 个样本
    if (baselineSampleCount >= 30) {
        // 步骤 2: 保存旧基线（用于稳定性检查）
        lastBaselineHR = baselineHR;
        lastBaselineRR = baselineRR;
        
        // 步骤 3: 计算平均值作为初始基线
        baselineHR = baselineHRSum / baselineSampleCount;
        baselineRR = baselineRRSum / baselineSampleCount;
        
        // 步骤 4: 标记为已校准
        baselineCalibrated = true;
        
        // 步骤 5: 清零累加器
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

/**
 * @brief 尝试转换到目标睡眠状态
 * 
 * @param target 目标状态
 * @param confirmMs 确认时间（毫秒）
 * @return true 转换成功（已达到确认时间）
 * @return false 尚未转换成功（仍在确认中）
 * 
 * 功能概述：
 * - 实现带滞后的状态转换机制
 * - 防止状态频繁切换
 * - 需要持续确认时间才能完成转换
 * 
 * 处理流程：
 * 1. 如果 pendingState != target → 设置新目标，重置计时器，返回 false
 * 2. 如果已达到确认时间 → 执行转换，更新 currentState，返回 true
 * 3. 否则 → 继续等待，返回 false
 * 
 * 设计思想：
 * - 两步转换：pendingState → currentState
 * - 时间确认：必须持续满足条件 confirmMs 毫秒
 * - 防止抖动：避免评分波动导致状态频繁切换
 * 
 * 示例：
 * - 浅睡→深睡：需要持续确认 60 秒（DEEP_SLEEP_CONFIRM_SECONDS）
 * - 浅睡→清醒：需要持续确认 30 秒（AWAKE_SLOW_CONFIRM_SECONDS）
 */
bool SleepAnalyzer::tryTransitionTo(SleepState target, uint64_t confirmMs)
{
    // 步骤 1: 如果是新目标，设置 pendingState 并重置计时器
    if (pendingState != target) {
        pendingState = target;
        pendingStateTime = radar_now_ms();
        return false;  // 刚开始转换，未完成
    }
    
    // 步骤 2: 检查是否已达到确认时间
    if (radar_now_ms() - pendingStateTime >= confirmMs) {
        // 执行转换
        currentState = target;
        stateEnterTime = radar_now_ms();
        pendingState = target;
        return true;  // 转换成功
    }
    
    return false;  // 仍在确认中
}

/**
 * @brief 更新睡眠周期
 * 
 * 功能概述：
 * - 追踪深睡和 REM 阶段
 * - 统计完整的睡眠周期数
 * - 记录周期时间戳
 * 
 * 睡眠周期定义：
 * 1. 一个完整周期 = 深睡 + REM
 * 2. 典型周期长度：90-120 分钟
 * 3. 每晚典型周期数：4-6 个
 * 
 * 处理流程：
 * 1. 检测进入深睡：标记 inDeepPhase，记录周期开始时间
 * 2. 检测进入 REM：标记 inRemPhase
 * 3. 检测离开深睡（进入浅睡）：清除 inDeepPhase，记录深睡结束时间
 * 4. 检测离开 REM（进入浅睡）：清除 inRemPhase，记录 REM 结束时间
 * 5. 检测周期结束（清醒/离床）：周期数 +1，重置阶段标记
 * 
 * 周期统计：
 * - cycleCount: 完整周期数
 * - cycleStartTime: 当前周期开始时间
 * - inDeepPhase: 是否处于深睡阶段
 * - inRemPhase: 是否处于 REM 阶段
 * - lastDeepEndTime: 深睡结束时间
 * - lastRemEndTime: REM 结束时间
 * 
 * 用途：
 * - 评估睡眠结构完整性
 * - 计算睡眠周期评分
 * - 监测睡眠质量
 */
void SleepAnalyzer::updateSleepCycle()
{
    // 步骤 1: 检测进入深睡
    if (currentState == SLEEP_DEEP_SLEEP && !cycle.inDeepPhase) {
        cycle.inDeepPhase = true;
        if (cycle.cycleStartTime == 0) cycle.cycleStartTime = radar_now_ms();
    }
    
    // 步骤 2: 检测进入 REM
    if (currentState == SLEEP_REM_SLEEP && !cycle.inRemPhase) cycle.inRemPhase = true;
    
    // 步骤 3: 检测离开深睡（进入浅睡）
    if (currentState == SLEEP_LIGHT_SLEEP && cycle.inDeepPhase) {
        cycle.inDeepPhase = false;
        cycle.lastDeepEndTime = radar_now_ms();
    }
    
    // 步骤 4: 检测离开 REM（进入浅睡）
    if (currentState == SLEEP_LIGHT_SLEEP && cycle.inRemPhase) {
        cycle.inRemPhase = false;
        cycle.lastRemEndTime = radar_now_ms();
    }
    
    // 步骤 5: 检测周期结束（清醒/离床）
    if ((currentState == SLEEP_AWAKE || currentState == SLEEP_OUT_OF_BED) &&
        (cycle.inDeepPhase || cycle.inRemPhase) &&
        cycle.cycleStartTime > 0) {
        cycle.cycleCount++;  // 周期数 +1
        cycle.inDeepPhase = false;
        cycle.inRemPhase = false;
        cycle.cycleStartTime = radar_now_ms();
        stats.sleepCycles = cycle.cycleCount;  // 更新统计
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
