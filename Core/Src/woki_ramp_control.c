#include "woki_ramp_control.h"

#include <math.h>
#include <string.h>

#include "dxl_2_0.h"
#include "imu_driver.h"
#include "main.h"
#include "robot_motion.h"

extern volatile uint8_t user_stop_flag;

#define WOKI_RAMP_PREVIEW_PERIOD_MS          50U
#define WOKI_RAMP_ALL_MOTOR_VALID_MASK       0x0FFFU
#define WOKI_RAMP_DXL_ZERO_COUNT             2048.0f
#define WOKI_RAMP_DXL_COUNTS_PER_DEG         (4096.0f / 360.0f)
#define WOKI_RAMP_RAD_TO_DEG                 57.29577951308232088f
#define WOKI_RAMP_QUATERNION_MIN_NORM_SQ     1.0e-12f
#define WOKI_RAMP_IMU_FILTER_ALPHA           0.2f
#define WOKI_RAMP_IMU_DEADBAND_DEG           0.5f

// 모터 순서: FL·FR·RL·RR의 ROLL·PITCH·KNEE
static const uint8_t woki_ramp_roll_state_index[WOKI_SLOPE_LEG_COUNT] = {
	0U, 3U, 6U, 9U
};

static const uint8_t woki_ramp_pitch_state_index[WOKI_SLOPE_LEG_COUNT] = {
	1U, 4U, 7U, 10U
};

// PITCH 양의 각도 기준: 왼쪽 엔코더 증가, 오른쪽 감소
static const int8_t woki_ramp_pitch_raw_sign[WOKI_SLOPE_LEG_COUNT] = {
	1, -1, 1, -1
};

static WokiRampControlStatus woki_ramp_status;
static uint32_t woki_ramp_last_sample_ms;

static uint8_t woki_ramp_imu_map_confirmed;
static uint8_t woki_ramp_allow_unverified_imu_preview;
static int8_t woki_ramp_imu_roll_sign;
static int8_t woki_ramp_imu_pitch_sign;
static float woki_ramp_imu_flat_roll_deg;
static float woki_ramp_imu_flat_pitch_deg;
static WokiRampMotionConfig woki_ramp_safety_config;
static WokiRampMotion woki_ramp_motion;
static DXL_Leg_State_t woki_ramp_motor_snapshot;
static uint32_t woki_ramp_imu_timestamp_ms;
static uint8_t woki_ramp_imu_filter_ready;
static float woki_ramp_filtered_roll_deg;
static float woki_ramp_filtered_pitch_deg;
static const uint8_t woki_ramp_motor_ids[12] = {
    1, 2, 3, 11, 12, 13, 21, 22, 23, 31, 32, 33
};

static void WokiRamp_UpdateMotion(void);
static WokiRampMotionFrame WokiRamp_MakeFrame(void);
static void WokiRamp_UpdateTrackingError(const WokiRampMotionFrame *frame);

// 작은 IMU 흔들림 제거
static float WokiRamp_ApplyDeadband(float angle_deg)
{
	if (fabsf(angle_deg) <= WOKI_RAMP_IMU_DEADBAND_DEG) return 0.0f;
	return angle_deg > 0.0f
			? angle_deg - WOKI_RAMP_IMU_DEADBAND_DEG
			: angle_deg + WOKI_RAMP_IMU_DEADBAND_DEG;
}

static float WokiRamp_WrapDeg(float angle_deg)
{
	while (angle_deg <= -180.0f) {
		angle_deg += 360.0f;
	}
	while (angle_deg > 180.0f) {
		angle_deg -= 360.0f;
	}
	return angle_deg;
}

static uint8_t WokiRamp_QuaternionToSensorRollPitch(const IMU_Data_t *imu,
		float *roll_deg, float *pitch_deg)
{
	float x;
	float y;
	float z;
	float w;
	float norm_sq;
	float inverse_norm;
	float sin_roll;
	float cos_roll;
	float sin_pitch;

	if (imu == NULL || roll_deg == NULL || pitch_deg == NULL) {
		return 0U;
	}

	x = imu->quaternion_x;
	y = imu->quaternion_y;
	z = imu->quaternion_z;
	w = imu->quaternion_w;
	norm_sq = x * x + y * y + z * z + w * w;

	if (!isfinite(norm_sq) || norm_sq < WOKI_RAMP_QUATERNION_MIN_NORM_SQ) {
		return 0U;
	}

	inverse_norm = 1.0f / sqrtf(norm_sq);
	x *= inverse_norm;
	y *= inverse_norm;
	z *= inverse_norm;
	w *= inverse_norm;

	sin_roll = 2.0f * (w * x + y * z);
	cos_roll = 1.0f - 2.0f * (x * x + y * y);
	*roll_deg = atan2f(sin_roll, cos_roll) * WOKI_RAMP_RAD_TO_DEG;

	sin_pitch = 2.0f * (w * y - z * x);
	if (sin_pitch > 1.0f) {
		sin_pitch = 1.0f;
	} else if (sin_pitch < -1.0f) {
		sin_pitch = -1.0f;
	}
	*pitch_deg = asinf(sin_pitch) * WOKI_RAMP_RAD_TO_DEG;

	return (isfinite(*roll_deg) && isfinite(*pitch_deg)) ? 1U : 0U;
}

static uint8_t WokiRamp_GetSensorRollPitch(const IMU_Data_t *imu,
		float *roll_deg, float *pitch_deg, uint8_t *quaternion_used)
{
	if (imu == NULL || roll_deg == NULL || pitch_deg == NULL
			|| quaternion_used == NULL || imu->valid == 0U) {
		return 0U;
	}

	if (imu->quaternion_valid != 0U) {
		if (!WokiRamp_QuaternionToSensorRollPitch(imu, roll_deg, pitch_deg)) {
			return 0U;
		}
		*quaternion_used = 1U;
		return 1U;
	}

	if (!isfinite(imu->roll) || !isfinite(imu->pitch)) {
		return 0U;
	}

	*roll_deg = imu->roll;
	*pitch_deg = imu->pitch;
	*quaternion_used = 0U;
	return 1U;
}

static void WokiRamp_DecodeJointPreview(const DXL_Leg_State_t *state)
{
	uint32_t leg;

	for (leg = 0U; leg < WOKI_SLOPE_LEG_COUNT; ++leg) {
		const int32_t roll_raw =
				state->motors[woki_ramp_roll_state_index[leg]].position_raw;
		const int32_t pitch_raw =
				state->motors[woki_ramp_pitch_state_index[leg]].position_raw;

		// 앞·뒤 ROLL 장착 방향 반영
		woki_ramp_status.hip_roll_deg[leg] =
				((leg < 2U) ? -1.0f : 1.0f)
						* ((float) roll_raw - WOKI_RAMP_DXL_ZERO_COUNT)
						/ WOKI_RAMP_DXL_COUNTS_PER_DEG;

		woki_ramp_status.hip_pitch_deg[leg] =
				(float) woki_ramp_pitch_raw_sign[leg]
						* ((float) pitch_raw - WOKI_RAMP_DXL_ZERO_COUNT)
						/ WOKI_RAMP_DXL_COUNTS_PER_DEG;
	}
}

void WokiRampControl_Init(void)
{
	memset(&woki_ramp_status, 0, sizeof(woki_ramp_status));
	woki_ramp_status.state = WOKI_RAMP_PREVIEW_DISABLED;
	woki_ramp_last_sample_ms = HAL_GetTick();

	// 확인된 IMU 부호·영점
	woki_ramp_imu_map_confirmed = 1U;
	woki_ramp_allow_unverified_imu_preview = 0U;
	woki_ramp_imu_roll_sign = 1;
	woki_ramp_imu_pitch_sign = 1;
	woki_ramp_imu_flat_roll_deg = -0.226f;
	woki_ramp_imu_flat_pitch_deg = -0.500f;
	WokiRampMotion_DefaultConfig(&woki_ramp_safety_config);
	WokiRampMotion_Reset(&woki_ramp_motion);
	memset(&woki_ramp_motor_snapshot, 0, sizeof(woki_ramp_motor_snapshot));
	woki_ramp_imu_timestamp_ms = 0U;
	woki_ramp_imu_filter_ready = 0U;
	woki_ramp_filtered_roll_deg = 0.0f;
	woki_ramp_filtered_pitch_deg = 0.0f;
	woki_ramp_status.output_gate_enabled = WOKI_RAMP_ENABLE_ACTUATOR_WRITES;
}

void WokiRampControl_EnablePreview(void)
{
    if (woki_ramp_status.has_written_goal != 0U) return;
    WokiRampMotion_Reset(&woki_ramp_motion);
    memset(woki_ramp_status.tracking_error_raw, 0,
            sizeof(woki_ramp_status.tracking_error_raw));
    memset(woki_ramp_status.max_tracking_error_raw, 0,
            sizeof(woki_ramp_status.max_tracking_error_raw));
	woki_ramp_imu_filter_ready = 0U;
	woki_ramp_status.enabled = 1U;
	woki_ramp_status.drive_ready = 0U;
	woki_ramp_status.formula_valid = 0U;
	woki_ramp_status.actuator_write_attempted = 0U;
	woki_ramp_status.state = WOKI_RAMP_PREVIEW_WAITING_FOR_IMU;
	woki_ramp_last_sample_ms = HAL_GetTick() - WOKI_RAMP_PREVIEW_PERIOD_MS;
}

void WokiRampControl_Disable(void)
{
    if (woki_ramp_status.has_written_goal
            && woki_ramp_motion.phase != WOKI_RAMP_MOTION_RETURNED) {
        WokiRampMotion_Fail(&woki_ramp_motion, WOKI_RAMP_FAULT_STOP);
        woki_ramp_status.drive_ready = 0U;
        woki_ramp_status.motion_phase = woki_ramp_motion.phase;
        woki_ramp_status.motion_fault = woki_ramp_motion.fault;
        return; // Wheel 복귀 전 보정 해제 차단 및 지지 목표 유지
    }
    woki_ramp_status.has_written_goal = 0U;
    WokiRampMotion_Reset(&woki_ramp_motion);
    woki_ramp_status.motion_phase = WOKI_RAMP_MOTION_OFF;
    woki_ramp_status.motion_fault = WOKI_RAMP_FAULT_NONE;
	woki_ramp_status.enabled = 0U;
	woki_ramp_status.drive_ready = 0U;
	woki_ramp_status.formula_valid = 0U;
	woki_ramp_status.state = WOKI_RAMP_PREVIEW_DISABLED;
	woki_ramp_imu_filter_ready = 0U;
}

uint8_t WokiRampControl_SetConfirmedImuMap(int8_t roll_sign,
		int8_t pitch_sign, float flat_roll_deg, float flat_pitch_deg)
{
	if (woki_ramp_status.has_written_goal != 0U) return 0U;
	if ((roll_sign != 1 && roll_sign != -1)
			|| (pitch_sign != 1 && pitch_sign != -1)
			|| !isfinite(flat_roll_deg) || !isfinite(flat_pitch_deg)) {
		return 0U;
	}

	woki_ramp_imu_roll_sign = roll_sign;
	woki_ramp_imu_pitch_sign = pitch_sign;
	woki_ramp_imu_flat_roll_deg = flat_roll_deg;
	woki_ramp_imu_flat_pitch_deg = flat_pitch_deg;
	woki_ramp_imu_filter_ready = 0U;
	woki_ramp_allow_unverified_imu_preview = 0U;
	woki_ramp_imu_map_confirmed = 1U;
	woki_ramp_status.imu_mapping_confirmed = 1U;
	return 1U;
}

void WokiRampControl_Task(void)
{
	uint32_t now_ms;
	uint32_t motor;
	IMU_Data_t imu;
	DXL_Leg_State_t motor_state;
	WokiSlopeInput formula_input;
	uint8_t imu_ok;
	uint8_t imu_fresh;
	uint16_t motor_valid_mask;

	if (woki_ramp_status.enabled == 0U) {
		return;
	}

	if (user_stop_flag && woki_ramp_status.has_written_goal) {
        WokiRampMotion_Fail(&woki_ramp_motion, WOKI_RAMP_FAULT_STOP);
        woki_ramp_status.drive_ready = 0U;
    }
	now_ms = HAL_GetTick();
	if ((uint32_t) (now_ms - woki_ramp_last_sample_ms)
			< WOKI_RAMP_PREVIEW_PERIOD_MS) {
		return;
	}
	woki_ramp_last_sample_ms = now_ms;
	woki_ramp_status.timestamp_ms = now_ms;
	woki_ramp_status.sample_count++;
	woki_ramp_status.actuator_write_attempted = 0U;

	imu = IMU_Get_Latest_Data();
	imu_fresh = WokiRamp_GetSensorRollPitch(&imu,
			&woki_ramp_status.sensor_roll_deg,
			&woki_ramp_status.sensor_pitch_deg,
			&woki_ramp_status.imu_quaternion_used);
	if (imu_fresh != 0U) {
		woki_ramp_imu_timestamp_ms = imu.timestamp_ms;
		imu_ok = 1U;
	} else if (woki_ramp_status.imu_valid != 0U
			&& (uint32_t)(now_ms - woki_ramp_imu_timestamp_ms)
					<= woki_ramp_safety_config.imu_max_age_ms) {
		imu_ok = 1U;
	} else {
		imu_ok = 0U;
	}
	woki_ramp_status.imu_valid = imu_ok;

	memset(&motor_state, 0, sizeof(motor_state));
	// 10ms 이내 최근 모터 조회 결과 공유
	motor_valid_mask = dxl_get_recent_leg_state(&motor_state, 10U);
	if (motor_valid_mask == WOKI_RAMP_ALL_MOTOR_VALID_MASK) {
		woki_ramp_motor_snapshot = motor_state;
	} else if (woki_ramp_motor_snapshot.valid_mask
			== WOKI_RAMP_ALL_MOTOR_VALID_MASK
			&& (uint32_t)(now_ms - woki_ramp_motor_snapshot.timestamp_ms)
					<= woki_ramp_safety_config.motor_max_age_ms) {
		motor_state = woki_ramp_motor_snapshot;
		motor_valid_mask = WOKI_RAMP_ALL_MOTOR_VALID_MASK;
	}
	woki_ramp_status.motor_valid_mask = motor_valid_mask;
	woki_ramp_status.motor_state_valid =
			(woki_ramp_status.motor_valid_mask
					== WOKI_RAMP_ALL_MOTOR_VALID_MASK) ? 1U : 0U;

	if (woki_ramp_status.motor_state_valid != 0U) {
		for (motor = 0U; motor < DXL_LEG_MOTOR_COUNT; ++motor) {
			woki_ramp_status.motor_position_raw[motor] =
					motor_state.motors[motor].position_raw;
		}
		woki_ramp_motor_snapshot = motor_state;
		WokiRamp_DecodeJointPreview(&motor_state);
	}

	woki_ramp_status.formula_valid = 0U;
	woki_ramp_status.raw_target_valid = 0U;
	woki_ramp_status.drive_ready = 0U;
	woki_ramp_status.imu_mapping_confirmed =
			woki_ramp_imu_map_confirmed;

	if (imu_ok == 0U) {
		woki_ramp_status.state = WOKI_RAMP_PREVIEW_WAITING_FOR_IMU;
		if (woki_ramp_motion.phase != WOKI_RAMP_MOTION_OFF
				&& woki_ramp_motion.phase != WOKI_RAMP_MOTION_LOCKED)
			WokiRamp_UpdateMotion();
		return;
	}

	if (woki_ramp_status.motor_state_valid == 0U) {
		woki_ramp_status.state =
				WOKI_RAMP_PREVIEW_WAITING_FOR_MOTOR_STATE;
		if (woki_ramp_motion.phase != WOKI_RAMP_MOTION_OFF
				&& woki_ramp_motion.phase != WOKI_RAMP_MOTION_LOCKED)
			WokiRamp_UpdateMotion();
		return;
	}

	if (woki_ramp_imu_map_confirmed == 0U
			&& woki_ramp_allow_unverified_imu_preview == 0U) {
		woki_ramp_status.state =
				WOKI_RAMP_PREVIEW_WAITING_FOR_CONFIRMED_IMU_MAP;
		WokiRamp_UpdateMotion();
		return;
	}

	if (imu_fresh != 0U) {
		const float roll_deg = (float)woki_ramp_imu_roll_sign
				* WokiRamp_WrapDeg(woki_ramp_status.sensor_roll_deg
						- woki_ramp_imu_flat_roll_deg);
		const float pitch_deg = (float)woki_ramp_imu_pitch_sign
				* WokiRamp_WrapDeg(woki_ramp_status.sensor_pitch_deg
						- woki_ramp_imu_flat_pitch_deg);
		if (woki_ramp_imu_filter_ready == 0U) {
			woki_ramp_filtered_roll_deg = roll_deg;
			woki_ramp_filtered_pitch_deg = pitch_deg;
			woki_ramp_imu_filter_ready = 1U;
		} else {
			woki_ramp_filtered_roll_deg += WOKI_RAMP_IMU_FILTER_ALPHA
					* WokiRamp_WrapDeg(roll_deg - woki_ramp_filtered_roll_deg);
			woki_ramp_filtered_pitch_deg += WOKI_RAMP_IMU_FILTER_ALPHA
					* WokiRamp_WrapDeg(pitch_deg - woki_ramp_filtered_pitch_deg);
		}
		woki_ramp_status.base_roll_deg = WokiRamp_ApplyDeadband(
				woki_ramp_filtered_roll_deg);
		woki_ramp_status.base_pitch_deg = WokiRamp_ApplyDeadband(
				woki_ramp_filtered_pitch_deg);
	}

	for (motor = 0U; motor < WOKI_SLOPE_LEG_COUNT; ++motor) {
		formula_input.hip_roll_deg[motor] =
				woki_ramp_status.hip_roll_deg[motor];
		formula_input.hip_pitch_deg[motor] =
				woki_ramp_status.hip_pitch_deg[motor];
	}
	formula_input.base_roll_deg = woki_ramp_status.base_roll_deg;
	formula_input.base_pitch_deg = woki_ramp_status.base_pitch_deg;

	if (!WokiSlope_Compute(&formula_input,
			&woki_ramp_status.formula_output)) {
		WokiRamp_UpdateMotion();
		return;
	}

	woki_ramp_status.formula_valid = 1U;
	woki_ramp_status.state = (woki_ramp_imu_map_confirmed != 0U)
			? WOKI_RAMP_PREVIEW_FORMULA_READY
			: WOKI_RAMP_PREVIEW_FORMULA_READY_UNVERIFIED_IMU;

	woki_ramp_status.raw_target_valid = WokiRampMotion_Encode(
			&woki_ramp_safety_config, &woki_ramp_status.formula_output,
			woki_ramp_status.formula_target_raw);
	WokiRamp_UpdateMotion();
}

uint8_t WokiRampControl_IsActive(void)
{
	return woki_ramp_status.enabled;
}

uint8_t WokiRampControl_IsDriveReady(void)
{
#if WOKI_RAMP_ENABLE_ACTUATOR_WRITES
    const uint32_t now = HAL_GetTick();
    return woki_ramp_status.enabled && woki_ramp_status.has_written_goal
            && woki_ramp_motion.phase == WOKI_RAMP_MOTION_ACTIVE
            && woki_ramp_motion.drive_ready && !user_stop_flag
            && woki_ramp_imu_map_confirmed
            && woki_ramp_status.formula_valid && woki_ramp_status.motor_state_valid
            && (uint32_t)(now - woki_ramp_motion.last_ms)
                    <= woki_ramp_safety_config.max_update_gap_ms
            && (uint32_t)(now - woki_ramp_imu_timestamp_ms)
                    <= woki_ramp_safety_config.imu_max_age_ms
            && (uint32_t)(now - woki_ramp_motor_snapshot.timestamp_ms)
                    <= woki_ramp_safety_config.motor_max_age_ms;
#else
    return 0U;
#endif
}

WokiRampControlStatus WokiRampControl_GetStatus(void)
{
	return woki_ramp_status;
}

uint8_t WokiRampControl_SetVerifiedSafetyConfig(const WokiRampMotionConfig *config)
{
    if (woki_ramp_status.enabled || woki_ramp_status.has_written_goal
            || !WokiRampMotion_ConfigValid(config)) return 0U;
    woki_ramp_safety_config = *config;
    return 1U;
}

static WokiRampMotionFrame WokiRamp_MakeFrame(void)
{
    WokiRampMotionFrame f;
    uint32_t i;
    memset(&f, 0, sizeof(f));
    f.now_ms = HAL_GetTick();
    f.imu_age_ms = f.now_ms - woki_ramp_imu_timestamp_ms;
    f.motor_age_ms = f.now_ms - woki_ramp_motor_snapshot.timestamp_ms;
    f.imu_valid = woki_ramp_status.imu_valid;
    f.imu_mapping_confirmed = woki_ramp_imu_map_confirmed;
    f.emergency_stop = user_stop_flag;
    f.motor_valid_mask = woki_ramp_status.motor_valid_mask;
    f.body_roll_deg = woki_ramp_status.base_roll_deg;
    f.body_pitch_deg = woki_ramp_status.base_pitch_deg;
    f.formula_valid = woki_ramp_status.formula_valid;
    f.formula = woki_ramp_status.formula_output;
    for (i = 0; i < 12U; ++i) {
        f.actual_raw[i] = woki_ramp_motor_snapshot.motors[i].position_raw;
        f.velocity_raw[i] = woki_ramp_motor_snapshot.motors[i].velocity_raw;
        if (!woki_ramp_motor_snapshot.motors[i].valid
                || woki_ramp_motor_snapshot.motors[i].status_error
                || woki_ramp_motor_snapshot.motors[i].id != woki_ramp_motor_ids[i])
            f.motor_error_mask |= (uint16_t)(1U << i);
    }
    return f;
}

// 현재 오차와 최대 오차 저장
static void WokiRamp_UpdateTrackingError(const WokiRampMotionFrame *f)
{
    uint32_t i;
    if (f == NULL
            || (woki_ramp_motion.phase != WOKI_RAMP_MOTION_ENTERING
                    && woki_ramp_motion.phase != WOKI_RAMP_MOTION_ACTIVE
                    && woki_ramp_motion.phase != WOKI_RAMP_MOTION_EXITING))
        return;
    for (i = 0U; i < 12U; ++i) {
        const float difference = fabsf((float)f->actual_raw[i]
                - woki_ramp_motion.command_raw[i]);
        const uint32_t error_raw = (uint32_t)(difference + 0.5f);
        woki_ramp_status.tracking_error_raw[i] = error_raw;
        if (error_raw > woki_ramp_status.max_tracking_error_raw[i])
            woki_ramp_status.max_tracking_error_raw[i] = error_raw;
    }
}

static void WokiRamp_UpdateMotion(void)
{
#if WOKI_RAMP_ENABLE_ACTUATOR_WRITES
    WokiRampMotionFrame f = WokiRamp_MakeFrame();
    uint32_t target[12];
    if (woki_ramp_motion.phase == WOKI_RAMP_MOTION_OFF
            || woki_ramp_motion.phase == WOKI_RAMP_MOTION_LOCKED) {
        if (!WokiRampMotion_ConfigValid(&woki_ramp_safety_config)
                || !woki_ramp_imu_map_confirmed) {
            woki_ramp_motion.phase = WOKI_RAMP_MOTION_LOCKED;
            woki_ramp_motion.fault = WOKI_RAMP_FAULT_NOT_VERIFIED;
        } else {
            Robot_Get_Wheel_Pose_Targets(target);
            (void) WokiRampMotion_Begin(&woki_ramp_motion,
                    &woki_ramp_safety_config, target, &f);
        }
    }
    WokiRamp_UpdateTrackingError(&f);
    if (WokiRampMotion_Update(&woki_ramp_motion, &f, target)) {
        if (user_stop_flag) {
            WokiRampMotion_Fail(&woki_ramp_motion, WOKI_RAMP_FAULT_STOP);
        } else {
            // 목표 위치만 전송, 모터 설정 유지
            // 송신 성공과 실제 모터 응답 확인은 별도
            woki_ramp_status.actuator_write_attempted = 1U;
            woki_ramp_status.has_written_goal = 1U; // 부분 송신에 따른 관절 동작 가능성 반영
            if (!dxl_sync_write_goal_positions_checked(woki_ramp_motor_ids, target, 12U)) {
                WokiRampMotion_Fail(&woki_ramp_motion, WOKI_RAMP_FAULT_TX);
            } else {
                memcpy(woki_ramp_status.commanded_target_raw, target, sizeof(target));
                woki_ramp_status.last_write_ms = HAL_GetTick();
            }
        }
    }
#else
    woki_ramp_motion.phase = WOKI_RAMP_MOTION_LOCKED;
    woki_ramp_motion.fault = WOKI_RAMP_FAULT_NOT_VERIFIED;
#endif
    woki_ramp_status.motion_phase = woki_ramp_motion.phase;
    woki_ramp_status.motion_fault = woki_ramp_motion.fault;
    woki_ramp_status.drive_ready = WokiRampControl_IsDriveReady();
}

uint8_t WokiRampControl_RequestWheelReturn(void)
{
    WokiRampMotionFrame f;
    if (!woki_ramp_status.has_written_goal) {
        WokiRampControl_Disable();
        return 1U; // 목표 전송 전이면 기존 Wheel 자세 유지
    }
    f = WokiRamp_MakeFrame();
    if (!WokiRampMotion_RequestReturn(&woki_ramp_motion, &f)) {
        woki_ramp_status.motion_phase = woki_ramp_motion.phase;
        woki_ramp_status.motion_fault = woki_ramp_motion.fault;
        return 0U;
    }
    woki_ramp_status.drive_ready = 0U;
    woki_ramp_status.motion_phase = woki_ramp_motion.phase;
    return 1U;
}

uint8_t WokiRampControl_IsWheelReturnComplete(void)
{
    return (!woki_ramp_status.enabled && !woki_ramp_status.has_written_goal)
            || woki_ramp_motion.phase == WOKI_RAMP_MOTION_RETURNED;
}
