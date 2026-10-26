// 경사로 자세 전환 및 엔코더 변환
// 통신·모터 게인 변경 제외
// 확인한 경사로 설정값 사용
#ifndef WOKI_RAMP_MOTION_H
#define WOKI_RAMP_MOTION_H
#include <stdint.h>
#include "woki_slope_formula.h"

#define WOKI_RAMP_JOINT_COUNT 12U
#define WOKI_RAMP_SAMPLE_MS 50U

typedef enum {
    WOKI_RAMP_MOTION_OFF = 0,
    WOKI_RAMP_MOTION_LOCKED,
    WOKI_RAMP_MOTION_ENTERING,
    WOKI_RAMP_MOTION_ACTIVE,
    WOKI_RAMP_MOTION_EXITING,
    WOKI_RAMP_MOTION_RETURNED,
    WOKI_RAMP_MOTION_FAULT
} WokiRampMotionPhase;

typedef enum {
    WOKI_RAMP_FAULT_NONE = 0,
    WOKI_RAMP_FAULT_NOT_VERIFIED,
    WOKI_RAMP_FAULT_INPUT,
    WOKI_RAMP_FAULT_IMU,
    WOKI_RAMP_FAULT_MOTOR,
    WOKI_RAMP_FAULT_STOP,
    WOKI_RAMP_FAULT_TIMING,
    WOKI_RAMP_FAULT_RANGE,
    WOKI_RAMP_FAULT_TRACKING,
    WOKI_RAMP_FAULT_TIMEOUT,
    WOKI_RAMP_FAULT_TERRAIN,
    WOKI_RAMP_FAULT_START_POSE,
    WOKI_RAMP_FAULT_TX,
    WOKI_RAMP_FAULT_EXIT_NOT_LEVEL
} WokiRampMotionFault;

typedef struct {
    uint8_t mapping_verified;
    uint8_t limits_and_motion_verified;
    uint8_t transition_path_verified;
    uint8_t motor_mode_profile_verified;
    uint8_t emergency_stop_path_verified;
    // 경사로 수식의 시뮬 관절각 기준, LEG 내부각과 별도
    float knee_ref_usd_deg[4];
    float knee_ref_raw[4];
    // 모터 저장 한계와 기구·경로 허용 범위의 공통 구간
    int32_t min_raw[12], max_raw[12];
    float command_rate_raw_s[12];  // 목표 위치 변화 속도 (엔코더 값/초)
    uint32_t settle_tolerance_raw[12]; // 목표 도착 허용 오차
    uint32_t tracking_tolerance_raw[12]; // 지속 오류 판정 허용 오차
    uint32_t settled_velocity_raw[12]; // 정착 판정 속도 한계 (모터 속도 원시값의 절댓값)
    uint32_t settle_ms, tracking_timeout_ms, motion_timeout_ms;
    uint32_t max_update_gap_ms, imu_max_age_ms, motor_max_age_ms;
    float entry_exit_level_deg;
    float max_lateral_deg, max_longitudinal_deg;
} WokiRampMotionConfig;

typedef struct {
    uint32_t now_ms, imu_age_ms, motor_age_ms;
    uint8_t imu_valid, imu_mapping_confirmed, emergency_stop;
    uint16_t motor_valid_mask, motor_error_mask;
    int32_t actual_raw[12], velocity_raw[12];
    float body_roll_deg, body_pitch_deg;
    uint8_t formula_valid;
    WokiSlopeOutput formula;
} WokiRampMotionFrame;

typedef struct {
    WokiRampMotionPhase phase;
    WokiRampMotionFault fault;
    uint8_t configured, drive_ready, error_timer_active, settle_timer_active;
    uint8_t return_requested;
    uint32_t last_ms, phase_start_ms, duration_ms, error_since_ms, settled_since_ms;
    uint32_t path_paused_ms;
    float command_raw[12], start_raw[12], target_raw[12];
    int32_t wheel_raw[12];
    WokiRampMotionConfig config;
} WokiRampMotion;

void WokiRampMotion_DefaultConfig(WokiRampMotionConfig *config);
uint8_t WokiRampMotion_ConfigValid(const WokiRampMotionConfig *config);
uint8_t WokiRampMotion_Encode(const WokiRampMotionConfig *config,
        const WokiSlopeOutput *target, int32_t raw[12]);
void WokiRampMotion_Reset(WokiRampMotion *motion);
uint8_t WokiRampMotion_Begin(WokiRampMotion *motion,
        const WokiRampMotionConfig *config, const int32_t wheel_raw[12],
        const WokiRampMotionFrame *frame);
uint8_t WokiRampMotion_RequestReturn(WokiRampMotion *motion,
        const WokiRampMotionFrame *frame);
uint8_t WokiRampMotion_Update(WokiRampMotion *motion,
        const WokiRampMotionFrame *frame, int32_t target_raw[12]);
void WokiRampMotion_Fail(WokiRampMotion *motion, WokiRampMotionFault fault);
#endif
