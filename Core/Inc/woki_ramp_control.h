// 경사로 입력·계산 및 목표 위치 전송
// IMU·안전 설정 확인 후 사용

#ifndef INC_WOKI_RAMP_CONTROL_H_
#define INC_WOKI_RAMP_CONTROL_H_

#include <stdint.h>

#include "woki_slope_formula.h"
#include "woki_ramp_motion.h"

// 경사로 관절 출력 허용
#ifndef WOKI_RAMP_ENABLE_ACTUATOR_WRITES
#define WOKI_RAMP_ENABLE_ACTUATOR_WRITES 1
#endif
#if WOKI_RAMP_ENABLE_ACTUATOR_WRITES != 0 && WOKI_RAMP_ENABLE_ACTUATOR_WRITES != 1
#error "WOKI_RAMP_ENABLE_ACTUATOR_WRITES must be 0 or 1"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	WOKI_RAMP_PREVIEW_DISABLED = 0,
	WOKI_RAMP_PREVIEW_WAITING_FOR_IMU,
	WOKI_RAMP_PREVIEW_WAITING_FOR_MOTOR_STATE,
	WOKI_RAMP_PREVIEW_WAITING_FOR_CONFIRMED_IMU_MAP,
	WOKI_RAMP_PREVIEW_FORMULA_READY,
	WOKI_RAMP_PREVIEW_FORMULA_READY_UNVERIFIED_IMU
} WokiRampPreviewState;

typedef struct {
	uint8_t enabled;
	uint8_t drive_ready;
	uint8_t imu_valid;
	uint8_t imu_quaternion_used;
	uint8_t imu_mapping_confirmed; // IMU 부호·영점 확인 여부
	uint8_t motor_state_valid;
	uint8_t formula_valid; // 수식 계산 성공 여부, 실물 검증과 별도
	uint8_t actuator_write_attempted;
	uint16_t motor_valid_mask;
	uint32_t timestamp_ms;
	uint32_t sample_count;
	WokiRampPreviewState state;

	float sensor_roll_deg;
	float sensor_pitch_deg;
	float base_roll_deg;
	float base_pitch_deg;

	int32_t motor_position_raw[12];
	float hip_roll_deg[WOKI_SLOPE_LEG_COUNT];
	float hip_pitch_deg[WOKI_SLOPE_LEG_COUNT];
	WokiSlopeOutput formula_output;
	uint8_t raw_target_valid;
	uint8_t output_gate_enabled;
	uint8_t has_written_goal;
	uint32_t formula_target_raw[12];
	uint32_t commanded_target_raw[12];
	uint32_t tracking_error_raw[12];
	uint32_t max_tracking_error_raw[12];
	uint32_t last_write_ms;
	WokiRampMotionPhase motion_phase;
	WokiRampMotionFault motion_fault;
} WokiRampControlStatus;

void WokiRampControl_Init(void);
void WokiRampControl_EnablePreview(void);
void WokiRampControl_Disable(void);
void WokiRampControl_Task(void);

// 실측 확인한 IMU 부호·영점 설정
// 부호는 +1 또는 -1, 관절 출력 잠금은 별도
uint8_t WokiRampControl_SetConfirmedImuMap(int8_t roll_sign,
		int8_t pitch_sign, float flat_roll_deg, float flat_pitch_deg);

uint8_t WokiRampControl_IsActive(void);

// 준비자세 정착 및 입력·시간 조건 충족 시 주행 허용
uint8_t WokiRampControl_IsDriveReady(void);

WokiRampControlStatus WokiRampControl_GetStatus(void);

// 안전 설정·복귀 관리, 출력 잠금은 별도
uint8_t WokiRampControl_SetVerifiedSafetyConfig(const WokiRampMotionConfig *config);
uint8_t WokiRampControl_RequestWheelReturn(void);
uint8_t WokiRampControl_IsWheelReturnComplete(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_WOKI_RAMP_CONTROL_H_ */
