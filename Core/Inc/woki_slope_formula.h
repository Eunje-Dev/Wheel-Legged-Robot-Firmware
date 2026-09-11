#ifndef WOKI_SLOPE_FORMULA_H
#define WOKI_SLOPE_FORMULA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    WOKI_SLOPE_FL = 0,
    WOKI_SLOPE_FR = 1,
    WOKI_SLOPE_RL = 2,
    WOKI_SLOPE_RR = 3,
    WOKI_SLOPE_LEG_COUNT = 4
};

typedef struct {
    // 시뮬 관절각 기준, 모터 엔코더 값과 별도
    float hip_roll_deg[WOKI_SLOPE_LEG_COUNT];
    float hip_pitch_deg[WOKI_SLOPE_LEG_COUNT];

    // IMU 축·영점 보정 후 중력 기준 몸통 자세각
    float base_roll_deg;
    float base_pitch_deg;
} WokiSlopeInput;

typedef struct {
    float lateral_from_joint_deg;
    float lateral_raw_deg;
    float lateral_command_deg;

    float wheel_chord_body_deg;
    float longitudinal_raw_deg;
    float longitudinal_command_deg;
    float pitch_delta_deg;

    float hip_roll_target_deg[WOKI_SLOPE_LEG_COUNT];
    float hip_pitch_target_deg[WOKI_SLOPE_LEG_COUNT];
    float knee_target_deg[WOKI_SLOPE_LEG_COUNT];

    uint8_t lateral_clipped;
    uint8_t longitudinal_clipped;
} WokiSlopeOutput;

// 기구학 기반 경사 상태 및 목표각 계산
// 성공 1, 입력 오류 0 반환
// 모터 좌표 변환·IMU 보정·시뮬 지지 보상항 제외
// 모터 명령 전송·설정 변경 제외
uint8_t WokiSlope_Compute(const WokiSlopeInput *input, WokiSlopeOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* WOKI_SLOPE_FORMULA_H */
