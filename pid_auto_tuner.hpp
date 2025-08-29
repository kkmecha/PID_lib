#pragma once

#include "mbed.h"
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

// PIDゲインを保持する構造体
struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

// テスト結果の評価指標を保持する構造体
struct TestResult {
    double rise_time_90 = -1.0;      // 90%到達時間 (-1は到達しなかった場合)
    double settling_time = -1.0;     // 整定時間 (-1は収束しなかった場合)
    double overshoot_percent = 0.0;  // オーバーシュート率 (%)
    double steady_state_error = 0.0; // 定常偏差
};

// 評価スコアの重み付けを設定する構造体
struct ScoringWeights {
    double rise_time = 1.0;
    double settling_time = 1.0;
    double overshoot = 1.5;
    double steady_state_error = 0.5;
};

/**
 * @class MbedPidAutotuner
 * @brief Mbed OS向けに最適化されたPIDオートチューニングクラス
 */
class MbedPidAutotuner {
public:
    // テスト実行用のCallbackの型定義
    using TestFunction = mbed::Callback<TestResult(const PidGains &)>;

    /**
     * @brief コンストラクタ
     */
    MbedPidAutotuner() {
        m_golden_ratio = (1.0 + sqrt(5.0)) / 2.0;
    }

    /**
     * @brief 探索範囲と精度を設定する
     */
    void setSearchRanges(double kp_low, double kp_high,
                           double ki_low, double ki_high,
                           double kd_low, double kd_high) {
        m_kp_low = kp_low;
        m_kp_high = kp_high;
        m_ki_low = ki_low;
        m_ki_high = ki_high;
        m_kd_low = kd_low;
        m_kd_high = kd_high;
    }

    /**
     * @brief 探索の収束判定に使われる精度を設定する
     */
    void setPrecision(double precision) {
        m_precision = precision;
    }

    /**
     * @brief PIDゲインの自動チューニングを実行するメイン関数
     * @param testFunc 外部から提供される、テストと評価を行うためのCallback
     * @param weights 各評価指標のスコアリングにおける重み
     * @return 見つかった最適なPIDゲイン
     */
    PidGains tune(TestFunction testFunc, const ScoringWeights &weights) {
        printf("===== PID Autotuning Started (Mbed Optimized) =====\r\n");
        m_test_func = testFunc;
        m_weights = weights;

        m_bestGains = {};
        m_bestGains.kp = tuneSingleParameter(m_kp_low, m_kp_high, 'p');
        m_bestGains.ki = tuneSingleParameter(m_ki_low, m_ki_high, 'i');
        m_bestGains.kd = tuneSingleParameter(m_kd_low, m_kd_high, 'd');

        printf("===== PID Autotuning Finished =====\r\n");
        return m_bestGains;
    }

private:
    /**
     * @brief 黄金分割探索で単一パラメータを最適化する
     */
    double tuneSingleParameter(double low, double high, char param_char) {
        PidGains current_gains = m_bestGains;
        if (high <= low) {
            return low; // 探索範囲が無効ならスキップ
        }

        printf("\n--- Tuning K%c in range [%.3f, %.3f] ---\r\n", param_char, low, high);

        double c = high - (high - low) / m_golden_ratio;
        double d = low + (high - low) / m_golden_ratio;

        updateGains(current_gains, param_char, c);
        double score_c = calculateScore(m_test_func(current_gains), current_gains);

        updateGains(current_gains, param_char, d);
        double score_d = calculateScore(m_test_func(current_gains), current_gains);

        while ((high - low) > m_precision) {
            if (score_c < score_d) {
                high = d;
                d = c;
                score_d = score_c;
                c = high - (high - low) / m_golden_ratio;
                updateGains(current_gains, param_char, c);
                score_c = calculateScore(m_test_func(current_gains), current_gains);
            } else {
                low = c;
                c = d;
                score_c = score_d;
                d = low + (high - low) / m_golden_ratio;
                updateGains(current_gains, param_char, d);
                score_d = calculateScore(m_test_func(current_gains), current_gains);
            }
        }

        double optimal_value = (high + low) / 2.0;
        printf("--- Optimal K%c found: %.4f ---\r\n", param_char, optimal_value);
        return optimal_value;
    }

    /**
     * @brief TestResultから最終的なスコアを計算する
     */
    double calculateScore(const TestResult &result, const PidGains &gains) {
        double score = 0.0;

        // 収束しなかったり、90%に到達しなかった場合は大きなペナルティ
        if (result.settling_time < 0 || result.rise_time_90 < 0) {
            return std::numeric_limits<double>::max();
        }

        score += result.rise_time_90 * m_weights.rise_time;
        score += result.settling_time * m_weights.settling_time;
        score += result.overshoot_percent * m_weights.overshoot;
        score += result.steady_state_error * m_weights.steady_state_error;

        printf("Test [Kp:%.3f Ki:%.3f Kd:%.3f] -> Score:%.3f (Rise:%.3fs Settle:%.3fs OS:%.1f%%)\r\n",
               gains.kp, gains.ki, gains.kd, score, result.rise_time_90, result.settling_time, result.overshoot_percent);

        return score;
    }

    /**
     * @brief ゲイン構造体の一部を更新するヘルパー関数
     */
    void updateGains(PidGains &gains, char param_char, double value) {
        if (param_char == 'p') {
            gains.kp = value;
        } else if (param_char == 'i') {
            gains.ki = value;
        } else {
            gains.kd = value;
        }
    }

    // Member Variables
    PidGains m_bestGains{};
    TestFunction m_test_func;
    ScoringWeights m_weights;
    double m_golden_ratio;
    double m_kp_low = 0.0, m_kp_high = 1.0;
    double m_ki_low = 0.0, m_ki_high = 1.0;
    double m_kd_low = 0.0, m_kd_high = 1.0;
    double m_precision = 0.01;
};