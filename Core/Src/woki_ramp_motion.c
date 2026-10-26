#include "woki_ramp_motion.h"
#include "woki_joint_config.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAMP_COUNTS_PER_DEG (4096.0f / 360.0f)
#define RAMP_KNEE_COUNTS_PER_DEG WOKI_KNEE_COUNTS_PER_DEG

// 기존 3.31초 전환 기준 속도
#define RAMP_COMMAND_RATE_RAW_S 175.0f
// 목표 도착 및 지속 오류 허용 범위
#define RAMP_SETTLE_TOLERANCE_DEG 6.0f
#define RAMP_TRACKING_TOLERANCE_DEG 10.0f
// 정착 및 오류 판정 시간
#define RAMP_SETTLE_MS WOKI_RAMP_SAMPLE_MS
#define RAMP_TRACKING_TIMEOUT_MS (20U * WOKI_RAMP_SAMPLE_MS)
#define RAMP_MOTION_TIMEOUT_MS 30000U
#define RAMP_MAX_UPDATE_GAP_MS (10U * WOKI_RAMP_SAMPLE_MS)
#define RAMP_INPUT_MAX_AGE_MS (10U * WOKI_RAMP_SAMPLE_MS)
#define RAMP_SETTLED_VELOCITY_RAW 10U
#define RAMP_LEVEL_TOLERANCE_DEG 5.0f
#define RAMP_MAX_LATERAL_DEG 16.0f
#define RAMP_MAX_LONGITUDINAL_DEG 10.5f

// 경사로 명령 범위: 종경사 0~10.5도, 횡경사 -16~16도
// 배열 순서: FL·FR·RL·RR의 ROLL·PITCH·KNEE
// 실제 관절 한계와 별도, 복합경사 전체 조합의 안전 보장 제외
static const uint32_t ramp_path_min_raw[12] = {
    1762U, 2048U, 2691U, 1794U, 1559U, 1096U,
    1794U, 2048U, 2691U, 1762U, 1766U, 1096U
};
static const uint32_t ramp_path_max_raw[12] = {
    2302U, 2537U, 3000U, 2334U, 2048U, 1405U,
    2334U, 2330U, 3000U, 2302U, 2048U, 1405U
};
// 보정 중 Knee 고정, 진입·복귀 시 별도 이동 범위 적용
// 실물 이동 확인값으로 목표 범위 확대 제외
static const uint32_t ramp_active_min_raw[12] = {
    1762U, 2330U, 3000U, 1794U, 1559U, 1096U,
    1794U, 2124U, 3000U, 1762U, 1766U, 1096U
};
static const uint32_t ramp_active_max_raw[12] = {
    2302U, 2537U, 3000U, 2334U, 1766U, 1096U,
    2334U, 2330U, 3000U, 2302U, 1972U, 1096U
};

static uint8_t ramp_command_in_range(const int32_t raw[12], uint8_t active)
{
    const uint32_t *minimum = active ? ramp_active_min_raw : ramp_path_min_raw;
    const uint32_t *maximum = active ? ramp_active_max_raw : ramp_path_max_raw;
    uint32_t i;
    for (i = 0U; i < 12U; ++i)
        if (raw[i] < WokiJoint_LegacyPosition(i, (float)minimum[i])
                || raw[i] > WokiJoint_LegacyPosition(i, (float)maximum[i])) return 0U;
    return 1U;
}

static uint8_t positive(float x) { return isfinite(x) && x > 0.0f; }
static uint8_t bounded_ms(uint32_t x) { return x > 0U && x < 0x80000000U; }

void WokiRampMotion_DefaultConfig(WokiRampMotionConfig *c)
{
    uint32_t leg, i;
    if (c == NULL) return;
    memset(c, 0, sizeof(*c));
    // 1:5 기어의 실물 확인 전에는 기존 1:2 검증을 재사용하지 않음
    c->mapping_verified = WOKI_KNEE_REFERENCE_VERIFIED;
    c->limits_and_motion_verified = WOKI_KNEE_REFERENCE_VERIFIED;
    c->transition_path_verified = WOKI_KNEE_REFERENCE_VERIFIED;
    c->motor_mode_profile_verified = WOKI_KNEE_REFERENCE_VERIFIED;
    c->emergency_stop_path_verified = 1U;
    // 시뮬 Knee 13도 기준: 왼쪽 3000, 오른쪽 1096
    // LEG 내부각·설계 각도와 별도 좌표 기준
    for (leg = 0; leg < 4U; ++leg) {
        c->knee_ref_usd_deg[leg] = 13.0f;
        c->knee_ref_raw[leg] = WokiJoint_FromLegacyRaw(leg * 3U + 2U,
                (leg % 2U == 0U) ? 3000.0f : 1096.0f);
    }
    // 관절 범위와 오차 기준 적용
    for (i = 0U; i < 12U; ++i) {
        const float counts_per_deg = (i % 3U == 2U)
                ? RAMP_KNEE_COUNTS_PER_DEG : RAMP_COUNTS_PER_DEG;
        c->min_raw[i] = WokiJoint_LegacyPosition(i, (float)ramp_path_min_raw[i]);
        c->max_raw[i] = WokiJoint_LegacyPosition(i, (float)ramp_path_max_raw[i]);
        // KNEE 엔코더 속도만 환산, 기존 관절 변화 속도·시간 정책 유지
        const float scale = WokiJoint_IsKneeIndex(i) ? WOKI_KNEE_LEGACY_TO_CURRENT : 1.0f;
        c->command_rate_raw_s[i] = RAMP_COMMAND_RATE_RAW_S * scale;
        c->settle_tolerance_raw[i] = (uint32_t)
                (RAMP_SETTLE_TOLERANCE_DEG * counts_per_deg + 0.5f);
        c->tracking_tolerance_raw[i] = (uint32_t)
                (RAMP_TRACKING_TOLERANCE_DEG * counts_per_deg + 0.5f);
        c->settled_velocity_raw[i] = (uint32_t)WokiJoint_RoundRaw(
                (float)RAMP_SETTLED_VELOCITY_RAW * scale);
    }
    c->settle_ms = RAMP_SETTLE_MS;
    c->tracking_timeout_ms = RAMP_TRACKING_TIMEOUT_MS;
    c->motion_timeout_ms = RAMP_MOTION_TIMEOUT_MS;
    c->max_update_gap_ms = RAMP_MAX_UPDATE_GAP_MS;
    c->imu_max_age_ms = RAMP_INPUT_MAX_AGE_MS;
    c->motor_max_age_ms = RAMP_INPUT_MAX_AGE_MS;
    c->entry_exit_level_deg = RAMP_LEVEL_TOLERANCE_DEG;
    c->max_lateral_deg = RAMP_MAX_LATERAL_DEG;
    c->max_longitudinal_deg = RAMP_MAX_LONGITUDINAL_DEG;
}

uint8_t WokiRampMotion_ConfigValid(const WokiRampMotionConfig *c)
{
    uint32_t i;
    if (c == NULL || c->mapping_verified != 1U
            || c->limits_and_motion_verified != 1U
            || c->transition_path_verified != 1U
            || c->motor_mode_profile_verified != 1U
            || c->emergency_stop_path_verified != 1U) return 0U;
    if (!bounded_ms(c->settle_ms) || !bounded_ms(c->tracking_timeout_ms)
            || !bounded_ms(c->motion_timeout_ms) || !bounded_ms(c->max_update_gap_ms)
            || !bounded_ms(c->imu_max_age_ms) || !bounded_ms(c->motor_max_age_ms)) return 0U;
    if (c->max_update_gap_ms < WOKI_RAMP_SAMPLE_MS
            || c->settle_ms < WOKI_RAMP_SAMPLE_MS
            || c->tracking_timeout_ms < c->settle_ms
            || c->motion_timeout_ms <= c->settle_ms) return 0U;
    if (!positive(c->entry_exit_level_deg) || !positive(c->max_lateral_deg)
            || !positive(c->max_longitudinal_deg)
            || c->max_lateral_deg > 16.0f || c->max_longitudinal_deg > 10.5f
            || c->entry_exit_level_deg > c->max_lateral_deg
            || c->entry_exit_level_deg > c->max_longitudinal_deg) return 0U;
    for (i = 0; i < 4U; ++i) {
        if (!isfinite(c->knee_ref_usd_deg[i]) || !isfinite(c->knee_ref_raw[i])
                || c->knee_ref_raw[i] < (float)WokiJoint_MinRaw(i * 3U + 2U)
                || c->knee_ref_raw[i] > (float)WokiJoint_MaxRaw(i * 3U + 2U))
            return 0U;
    }
    for (i = 0; i < 12U; ++i) {
        if (!WokiJoint_PositionValid(i, c->min_raw[i])
                || !WokiJoint_PositionValid(i, c->max_raw[i])
                || c->min_raw[i] >= c->max_raw[i]
                || !positive(c->command_rate_raw_s[i])
                || c->command_rate_raw_s[i] > (float)(WokiJoint_MaxRaw(i) - WokiJoint_MinRaw(i)) * 1000.0f
                || c->settle_tolerance_raw[i] == 0U
                || c->settle_tolerance_raw[i] >= c->tracking_tolerance_raw[i]
                || c->tracking_tolerance_raw[i] == 0U
                || c->tracking_tolerance_raw[i] >= (uint32_t)(c->max_raw[i] - c->min_raw[i])
                || c->settled_velocity_raw[i] == 0U
                || c->settled_velocity_raw[i] >= 0x80000000U) return 0U;
    }
    return 1U;
}

uint8_t WokiRampMotion_Encode(const WokiRampMotionConfig *c,
        const WokiSlopeOutput *target, int32_t raw[12])
{
    float values[12];
    uint32_t leg, i;
    if (c == NULL || target == NULL || raw == NULL) return 0U;
    for (leg = 0; leg < 4U; ++leg) {
        const float roll_sign = (leg < 2U) ? -1.0f : 1.0f;
        const float pitch_sign = (leg % 2U == 0U) ? 1.0f : -1.0f;
        const float knee_sign = -pitch_sign;
        values[3U * leg] = 2048.0f + roll_sign * RAMP_COUNTS_PER_DEG
                * target->hip_roll_target_deg[leg];
        values[3U * leg + 1U] = 2048.0f + pitch_sign * RAMP_COUNTS_PER_DEG
                * target->hip_pitch_target_deg[leg];
        values[3U * leg + 2U] = c->knee_ref_raw[leg] + knee_sign
                * RAMP_KNEE_COUNTS_PER_DEG
                * (target->knee_target_deg[leg] - c->knee_ref_usd_deg[leg]);
    }
    // 엔코더 표현 범위 초과 시 명령 거부
    for (i = 0; i < 12U; ++i)
        if (!isfinite(values[i]) || values[i] < (float)WokiJoint_MinRaw(i)
                || values[i] > (float)WokiJoint_MaxRaw(i))
            return 0U;
    for (i = 0; i < 12U; ++i) raw[i] = WokiJoint_RoundRaw(values[i]);
    return 1U;
}

void WokiRampMotion_Reset(WokiRampMotion *m)
{
    if (m != NULL) memset(m, 0, sizeof(*m));
}

void WokiRampMotion_Fail(WokiRampMotion *m, WokiRampMotionFault fault)
{
    if (m == NULL) return;
    m->phase = WOKI_RAMP_MOTION_FAULT;
    m->fault = fault;
    m->drive_ready = 0U;
    // 이전 목표 유지, 토크 해제·현재각 재지정 제외
}

static uint8_t in_range(const WokiRampMotion *m, const int32_t raw[12])
{
    uint32_t i;
    if (!ramp_command_in_range(raw, 0U)) return 0U;
    // 경사로 목표각 범위 검사
    for (i = 0; i < 12U; ++i)
        if (raw[i] < m->config.min_raw[i] || raw[i] > m->config.max_raw[i])
            return 0U;
    return 1U;
}

static WokiRampMotionFault frame_fault(const WokiRampMotion *m,
        const WokiRampMotionFrame *f)
{
    if (f == NULL) return WOKI_RAMP_FAULT_INPUT;
    if (f->emergency_stop) return WOKI_RAMP_FAULT_STOP;
    if (!f->imu_valid || !f->imu_mapping_confirmed
            || f->imu_age_ms > m->config.imu_max_age_ms
            || !isfinite(f->body_roll_deg) || !isfinite(f->body_pitch_deg))
        return WOKI_RAMP_FAULT_IMU;
    if (f->motor_valid_mask != 0x0FFFU || f->motor_error_mask != 0U
            || f->motor_age_ms > m->config.motor_max_age_ms)
        return WOKI_RAMP_FAULT_MOTOR;
    return WOKI_RAMP_FAULT_NONE;
}

static WokiRampMotionFault return_frame_fault(const WokiRampMotion *m,
        const WokiRampMotionFrame *f)
{
    if (f == NULL) return WOKI_RAMP_FAULT_INPUT;
    if (f->emergency_stop) return WOKI_RAMP_FAULT_STOP;
    if (f->motor_valid_mask != 0x0FFFU || f->motor_error_mask != 0U
            || f->motor_age_ms > m->config.motor_max_age_ms)
        return WOKI_RAMP_FAULT_MOTOR;
    return WOKI_RAMP_FAULT_NONE;
}

static uint8_t start_path(WokiRampMotion *m, const int32_t target[12],
        uint32_t now_ms, WokiRampMotionPhase phase)
{
    uint32_t i;
    float duration_ms = (float)WOKI_RAMP_SAMPLE_MS;
    if (!in_range(m, target)) { WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_RANGE); return 0U; }
    for (i = 0; i < 12U; ++i) {
        const float required_ms = 1875.0f * fabsf((float)target[i] - m->command_raw[i])
                / m->config.command_rate_raw_s[i];
        if (!isfinite(required_ms)) { WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_TIMING); return 0U; }
        if (required_ms > duration_ms) duration_ms = required_ms;
    }
    if (duration_ms + (float)m->config.settle_ms >= (float)m->config.motion_timeout_ms) {
        WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_TIMEOUT); return 0U;
    }
    m->duration_ms = (uint32_t)ceilf(duration_ms);
    for (i = 0; i < 12U; ++i) {
        m->start_raw[i] = m->command_raw[i];
        m->target_raw[i] = (float)target[i];
    }
    m->phase = phase;
    m->phase_start_ms = now_ms;
    m->path_paused_ms = 0U;
    m->last_ms = now_ms;
    m->settle_timer_active = 0U;
    m->error_timer_active = 0U;
    m->drive_ready = 0U;
    m->fault = WOKI_RAMP_FAULT_NONE;
    return 1U;
}

uint8_t WokiRampMotion_Begin(WokiRampMotion *m,
        const WokiRampMotionConfig *c, const int32_t wheel_raw[12],
        const WokiRampMotionFrame *f)
{
    WokiSlopeOutput neutral;
    int32_t target[12];
    uint32_t i;
    WokiRampMotionFault fault;
    if (m == NULL || c == NULL || wheel_raw == NULL || f == NULL) return 0U;
    // 동작 중·오류 상태의 자동 초기화 차단
    if (m->phase != WOKI_RAMP_MOTION_OFF && m->phase != WOKI_RAMP_MOTION_LOCKED) return 0U;
    if (!WokiRampMotion_ConfigValid(c)) {
        m->phase = WOKI_RAMP_MOTION_LOCKED;
        m->fault = WOKI_RAMP_FAULT_NOT_VERIFIED;
        return 0U;
    }
    m->config = *c;
    m->configured = 1U;
    fault = frame_fault(m, f);
    if (fault != WOKI_RAMP_FAULT_NONE) { WokiRampMotion_Fail(m, fault); return 0U; }
    if (!in_range(m, wheel_raw)) { WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_RANGE); return 0U; }
    memset(&neutral, 0, sizeof(neutral));
    for (i = 0; i < 12U; ++i) {
        m->wheel_raw[i] = wheel_raw[i];
        m->command_raw[i] = (float)wheel_raw[i];
    }
    for (i = 0; i < 4U; ++i) {
        neutral.hip_pitch_target_deg[i] = 24.8f; // 기존 수식의 준비 PITCH 각도
        neutral.knee_target_deg[i] = 13.0f;
    }
    if (!WokiRampMotion_Encode(c, &neutral, target)
            || !ramp_command_in_range(target, 1U)) {
        WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_RANGE); return 0U;
    }
    return start_path(m, target, f->now_ms, WOKI_RAMP_MOTION_ENTERING);
}

uint8_t WokiRampMotion_RequestReturn(WokiRampMotion *m, const WokiRampMotionFrame *f)
{
    if (m == NULL || f == NULL || !m->configured) return 0U;
    if (m->phase == WOKI_RAMP_MOTION_EXITING || m->phase == WOKI_RAMP_MOTION_RETURNED) return 1U;
    if (m->phase != WOKI_RAMP_MOTION_ACTIVE
            && m->phase != WOKI_RAMP_MOTION_ENTERING
            && m->phase != WOKI_RAMP_MOTION_FAULT)
		return 0U;
    if (f->emergency_stop) {
        WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_STOP);
        return 0U;
    }
    // 현재 보정 자세에서 Wheel 자세로 바로 복귀
    m->return_requested = 1U;
    m->drive_ready = 0U;
    m->error_timer_active = 0U;
    m->settle_timer_active = 0U;
    m->fault = WOKI_RAMP_FAULT_NONE;
    return start_path(m, m->wheel_raw, f->now_ms,
            WOKI_RAMP_MOTION_EXITING);
}

uint8_t WokiRampMotion_Update(WokiRampMotion *m,
        const WokiRampMotionFrame *f, int32_t raw[12])
{
    WokiRampMotionFault fault;
    uint32_t i, dt, elapsed;
    int32_t desired[12];
    uint8_t tracking_ok = 1U, quiet = 1U, path_done = 0U;
    float candidate[12];
    if (m == NULL || f == NULL || raw == NULL || !m->configured) return 0U;
    if (m->phase != WOKI_RAMP_MOTION_ENTERING && m->phase != WOKI_RAMP_MOTION_ACTIVE
            && m->phase != WOKI_RAMP_MOTION_EXITING) return 0U;
	if (m->phase != WOKI_RAMP_MOTION_ACTIVE) m->drive_ready = 0U;
    fault = (m->phase == WOKI_RAMP_MOTION_EXITING)
            ? return_frame_fault(m, f) : frame_fault(m, f);
    if (fault != WOKI_RAMP_FAULT_NONE) { WokiRampMotion_Fail(m, fault); return 0U; }
    dt = f->now_ms - m->last_ms;
    if (dt > m->config.max_update_gap_ms) {
        WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_TIMING); return 0U;
    }
    if (dt == 0U) return 0U;
    m->last_ms = f->now_ms;
    if ((m->phase != WOKI_RAMP_MOTION_ACTIVE || m->return_requested)
            && f->now_ms - m->phase_start_ms >= m->config.motion_timeout_ms) {
        WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_TIMEOUT); return 0U;
    }
    for (i = 0; i < 12U; ++i) {
        const float tracking_error =
                fabsf((float)f->actual_raw[i] - m->command_raw[i]);
        if (tracking_error > (float)m->config.tracking_tolerance_raw[i])
            tracking_ok = 0U;
        if (fabsf((float)f->velocity_raw[i])
                > (float)m->config.settled_velocity_raw[i]) quiet = 0U;
    }
    // 준비자세 진입 중 추종 오차로 시작 차단 제외
    if (!tracking_ok && m->phase != WOKI_RAMP_MOTION_ENTERING
            && m->phase != WOKI_RAMP_MOTION_EXITING) {
        if (!m->error_timer_active) { m->error_timer_active = 1U; m->error_since_ms = f->now_ms; }
        if (m->phase == WOKI_RAMP_MOTION_ACTIVE)
            m->drive_ready = (f->now_ms - m->error_since_ms
                    < m->config.tracking_timeout_ms) ? 1U : 0U;
        // 추종 오차 초과 시 이전 지지 목표 유지
        // 오차 회복 후 자동 재개
        if (m->phase != WOKI_RAMP_MOTION_ACTIVE) m->path_paused_ms += dt;
        m->settle_timer_active = 0U;
        return 0U;
    } else m->error_timer_active = 0U;
    if (m->phase == WOKI_RAMP_MOTION_ACTIVE) {
        if (!f->formula_valid || !isfinite(f->formula.lateral_raw_deg)
                || !isfinite(f->formula.longitudinal_raw_deg)) {
            WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_INPUT); return 0U;
        }
        if (!WokiRampMotion_Encode(&m->config, &f->formula, desired)
                || !in_range(m, desired)
                || !ramp_command_in_range(desired, 1U)) {
            WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_RANGE); return 0U;
        }
        for (i = 0; i < 12U; ++i) {
            const float max_step = m->config.command_rate_raw_s[i] * (float)dt / 1000.0f;
            const float delta = (float)desired[i] - m->command_raw[i];
            candidate[i] = m->command_raw[i]
                    + (delta > max_step ? max_step : (delta < -max_step ? -max_step : delta));
        }
        m->drive_ready = tracking_ok;
    } else {
        float t, h;
        elapsed = f->now_ms - m->phase_start_ms;
        if (elapsed >= m->config.motion_timeout_ms) {
            WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_TIMEOUT); return 0U;
        }
        t = (float)(elapsed - m->path_paused_ms) / (float)m->duration_ms;
        if (t >= 1.0f) { t = 1.0f; path_done = 1U; }
        h = t*t*t * (10.0f + t * (-15.0f + 6.0f*t));
        for (i = 0; i < 12U; ++i)
            candidate[i] = m->start_raw[i] + h * (m->target_raw[i] - m->start_raw[i]);
        // 최종 목표각 기준 정착 확인
        for (i = 0; i < 12U; ++i)
            if (fabsf((float)f->actual_raw[i] - m->target_raw[i])
                    > (float)m->config.settle_tolerance_raw[i]) path_done = 0U;
        if (path_done && quiet) {
            if (!m->settle_timer_active) { m->settle_timer_active = 1U; m->settled_since_ms = f->now_ms; }
            if (f->now_ms - m->settled_since_ms >= m->config.settle_ms) {
                m->phase = (m->phase == WOKI_RAMP_MOTION_ENTERING)
                        ? WOKI_RAMP_MOTION_ACTIVE : WOKI_RAMP_MOTION_RETURNED;
                if (m->phase == WOKI_RAMP_MOTION_ACTIVE && m->return_requested) {
                    m->phase_start_ms = f->now_ms;
                    m->settle_timer_active = 0U;
                }
                // 다음 정상 갱신 후 주행 허용, 바퀴 자동 출발 제외
            }
        } else m->settle_timer_active = 0U;
    }
    for (i = 0; i < 12U; ++i) {
        if (!isfinite(candidate[i]) || candidate[i] < (float)m->config.min_raw[i]
                || candidate[i] > (float)m->config.max_raw[i]
                || candidate[i] < (float)WokiJoint_LegacyPosition(i, (float)ramp_path_min_raw[i])
                || candidate[i] > (float)WokiJoint_LegacyPosition(i, (float)ramp_path_max_raw[i])) {
            WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_RANGE); return 0U;
        }
        desired[i] = WokiJoint_RoundRaw(candidate[i]);
    }
    // 12개 목표 전체 검사 후 적용, 부분 적용·강제 제한 제외
    if (!ramp_command_in_range(desired, m->phase == WOKI_RAMP_MOTION_ACTIVE)) {
        WokiRampMotion_Fail(m, WOKI_RAMP_FAULT_RANGE); return 0U;
    }
    for (i = 0; i < 12U; ++i) {
        m->command_raw[i] = candidate[i];
        raw[i] = desired[i];
    }
    return 1U;
}
