/*
 * robot_motion.c
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */
#include "robot_motion.h"
#include "dxl_2_0.h"
#include "wheel_to_leg_trajectory_data.h"
#include "woki_ramp_control.h"
#include "woki_walk_demo_trajectory.h"
#include "woki_joint_config.h"
#include "woki_power_guard.h"

//AX Wheel Mode 초기 설정 저장
static uint8_t ax_wheel_mode_ready = 0;
// 시작 자세 Mode_NONE 저장
static volatile RobotMode_t robot_mode = MODE_NONE;

extern volatile uint8_t user_stop_flag;
// Wheel Mode 전진·후진 가속 설정
#define WHEEL_DRIVE_MAX_SPEED          1023
#define WHEEL_DRIVE_ACCEL_PERIOD_MS    10U
#define WHEEL_DRIVE_ACCEL_STEP         10

// Wheel Mode 전진·후진 가속 상태
static int16_t wheel_drive_current_speed = 0;
static int16_t wheel_drive_target_speed = 0;
static uint32_t wheel_drive_last_update_ms = 0U;
static uint8_t wheel_drive_accel_active = 0U;

// Wheel·경사로 관절 프로파일
#define WHEEL_PROFILE_ACCELERATION 5U
#define WHEEL_PROFILE_VELOCITY     10U
#define RAMP_PROFILE_ACCELERATION  5U
#define RAMP_PROFILE_VELOCITY      15U

// 촬영용 보행 설정
#define WALK_DEMO_ENTER_TIME_MS  2000U
#define WALK_DEMO_SPEED_PERCENT  1000U
#define WALK_DEMO_ANGLE_SCALE       3
#define WALK_DEMO_ID3_INDEX         2U
#define WALK_DEMO_ID3_MIN_RAW       970U

volatile RobotWalkDemoStatus_t robot_walk_demo_status;

static int32_t walk_demo_start_raw[12];
static int32_t walk_demo_target_raw[12];
// 원본 1:2 목표를 누적한 뒤 환산, 변화량 반올림 누적 방지
static int32_t walk_demo_legacy_target_raw[12];
static uint32_t walk_demo_phase_start_ms = 0U;
static uint32_t walk_demo_last_update_ms = 0U;
static uint32_t walk_demo_speed_accumulator = 0U;
static uint8_t walk_demo_target_valid = 0U;

// 원본 0 변화량 구간을 같은 샘플 수로 재생
static uint32_t walk_demo_delta_event_index = 0U;
static uint16_t walk_demo_zero_remaining = 0U;

// 수식 기반 Wheel-to-Leg 무부하 시험 제어 주기 및 1.5배 시간 설정
#define WHEEL_TO_LEG_CONTROL_PERIOD_MS  10U
#define WHEEL_TO_LEG_APPROACH_TIME_MS   3310U
#define WHEEL_TO_LEG_STAND_TIME_MS      9000U
#define WHEEL_TO_LEG_TOTAL_TIME_MS      (WHEEL_TO_LEG_APPROACH_TIME_MS + WHEEL_TO_LEG_STAND_TIME_MS)

// Leg-to-Wheel 제어 주기 및 구간 시간
#define LEG_TO_WHEEL_CONTROL_PERIOD_MS    10U
#define LEG_TO_WHEEL_FOOT_FIXED_TIME_MS   3500U
#define LEG_TO_WHEEL_WHEEL_FIXED_TIME_MS  2500U
#define LEG_TO_WHEEL_TOTAL_TIME_MS        (LEG_TO_WHEEL_FOOT_FIXED_TIME_MS + LEG_TO_WHEEL_WHEEL_FIXED_TIME_MS)

// Wheel-to-Leg 계산 각도
// Knee 시작각: 설계 각도 40도 - 좌표 차이 8.44도
#define WHEEL_TO_LEG_KNEE_START_DEG          31.56f
#define WHEEL_TO_LEG_CONTACT_PITCH_DEG       38.1406444612f
#define WHEEL_TO_LEG_CONTACT_KNEE_DEG        33.2637871388f
#define WHEEL_TO_LEG_FINAL_FRONT_PITCH_DEG   30.1343974072f
#define WHEEL_TO_LEG_FINAL_FRONT_KNEE_DEG    94.6463063272f

// MX 영점 및 각도-엔코더 환산값
#define WHEEL_TO_LEG_PITCH_ZERO_POSITION       2048.0f
#define WHEEL_TO_LEG_PITCH_POSITION_PER_DEG    (4096.0f / 360.0f)
#define WHEEL_TO_LEG_KNEE_POSITION_PER_DEG     WOKI_KNEE_COUNTS_PER_DEG

static void Robot_Run_Leg_To_Wheel_Trajectory(void);
static uint8_t Robot_Wheel_To_Leg_Stop_Requested(void);
static RobotMode_t ramp_pending_mode = MODE_NONE;
static void Robot_Finish_Ramp_Return(void);

// 로봇 동작 초기화
void Robot_Motion_Init(void) {
	RobotPower_Init(); // 매 부팅마다 시작 허가 해제, 이전 세션 허가를 재사용하지 않음
	WokiRampControl_Init();
	robot_walk_demo_status.active = 0U;
	robot_walk_demo_status.state = WALK_DEMO_IDLE;
	robot_walk_demo_status.sample_index = 0U;
	robot_walk_demo_status.elapsed_ms = 0U;
	robot_walk_demo_status.tx_fail_count = 0U;
	walk_demo_target_valid = 0U;
}

// 주차·잠금 상태에서 이전 보행·경사로·가속 명령이 목표를 다시 덮어쓰지 않도록 정리
// 관절 토크를 끄지 않으며, 바퀴에는 속도 0만 전송
void Robot_Power_StopLocalMotion(void) {
	Robot_Wheel_Stop();
	robot_walk_demo_status.active = 0U;
	robot_walk_demo_status.state = WALK_DEMO_STOPPED;
	walk_demo_target_valid = 0U;
	ramp_pending_mode = MODE_NONE;
	// 전원 기준을 잃은 세션의 경사로 경로는 재개하지 않음. RAM만 초기화, 모터 목표는 유지
	WokiRampControl_Init();
	robot_mode = MODE_NONE;
}

// 경사로 입력·계산 및 자세 전환 처리
void Robot_Ramp_Control_Task(void) {
	if (!RobotPower_MotionAllowed()) return;
	if (robot_mode == MODE_RAMP) {
		WokiRampControl_Task();
        // 주행 허용 해제 시 정속 주행도 정지
        if (!WokiRampControl_IsDriveReady()
                && (wheel_drive_current_speed != 0 || wheel_drive_target_speed != 0
                        || wheel_drive_accel_active)) Robot_Wheel_Stop();
        if (ramp_pending_mode != MODE_NONE
                && !WokiRampControl_IsWheelReturnComplete()) {
            (void) WokiRampControl_RequestWheelReturn();
        }
        Robot_Finish_Ramp_Return();
	}
}

// 로봇 현재 모드 확인
RobotMode_t Robot_Get_Mode(void) {
	return robot_mode;
}

// AX 바퀴 정지
void Robot_Wheel_Stop(void) {
	int16_t wheel_speed[4] = { 0, 0, 0, 0 };

	wheel_drive_current_speed = 0;
	wheel_drive_target_speed = 0;
	wheel_drive_last_update_ms = HAL_GetTick();
	wheel_drive_accel_active = 0U;

	send_sync_write_1(DXL_1_MOVING_SPEED, 2, wheel_speed);
}

// 다리 관절 프로파일 설정
static void Robot_Set_Leg_Profile(uint32_t acceleration, uint32_t velocity) {
	uint8_t ids[12] = {
			legs[1].roll, legs[1].pitch, legs[1].knee,
			legs[2].roll, legs[2].pitch, legs[2].knee,
			legs[3].roll, legs[3].pitch, legs[3].knee,
			legs[4].roll, legs[4].pitch, legs[4].knee
	};
	uint32_t acceleration_values[12];
	uint32_t velocity_values[12];

	for (uint32_t i = 0U; i < 12U; ++i) {
		const float scale = WokiJoint_IsKneeIndex(i)
				? WOKI_KNEE_LEGACY_TO_CURRENT : 1.0f;
		acceleration_values[i] = (uint32_t)WokiJoint_RoundRaw((float)acceleration * scale);
		velocity_values[i] = (uint32_t)WokiJoint_RoundRaw((float)velocity * scale);
	}

	dxl_sync_write_4byte(ids, acceleration_values, 12,
			DXL_2_Profile_Acceleration);
	dxl_sync_write_4byte(ids, velocity_values, 12,
			DXL_2_Profile_Velocity);
}

// AX Wheel Mode 초기 설정
static void Robot_AX_Wheel_Mode_Setup(void) {
	if (ax_wheel_mode_ready) {
		return;
	}

	for (int i = 1; i <= 4; i++) {
		uint8_t id = wheels[i].wheel;

		dxl_write_1_0(id, DXL_1_Torque_Enable, 1, 0);
		HAL_Delay(20);

		dxl_write_1_0(id, DXL_1_CW_Angle_Limit, 2, 0);
		HAL_Delay(20);

		dxl_write_1_0(id, DXL_1_CCW_Angle_Limit, 2, 0);
		HAL_Delay(20);

		dxl_write_1_0(id, DXL_1_Goal_Torque_Limit, 2, 1023);
		HAL_Delay(20);

		dxl_write_1_0(id, DXL_1_Torque_Enable, 1, 1);
		HAL_Delay(20);
	}

	ax_wheel_mode_ready = 1;
}

// Wheel Mode 전진·후진 가속 처리
void Robot_Wheel_Drive_Task(void) {
	if (!RobotPower_MotionAllowed()) return;
	uint32_t current_time;
	int16_t wheel_data[4];

	if (!wheel_drive_accel_active) {
		return;
	}

	if (robot_mode != MODE_WHEEL
			&& (robot_mode != MODE_RAMP || !WokiRampControl_IsDriveReady())) {
		wheel_drive_current_speed = 0;
		wheel_drive_target_speed = 0;
		wheel_drive_accel_active = 0U;
		return;
	}

	current_time = HAL_GetTick();

	if ((uint32_t) (current_time - wheel_drive_last_update_ms)
			< WHEEL_DRIVE_ACCEL_PERIOD_MS) {
		return;
	}

	wheel_drive_last_update_ms = current_time;

	if (wheel_drive_current_speed < wheel_drive_target_speed) {
		wheel_drive_current_speed += WHEEL_DRIVE_ACCEL_STEP;

		if (wheel_drive_current_speed > wheel_drive_target_speed) {
			wheel_drive_current_speed = wheel_drive_target_speed;
		}
	} else if (wheel_drive_current_speed > wheel_drive_target_speed) {
		wheel_drive_current_speed -= WHEEL_DRIVE_ACCEL_STEP;

		if (wheel_drive_current_speed < wheel_drive_target_speed) {
			wheel_drive_current_speed = wheel_drive_target_speed;
		}
	}

	wheel_data[0] = wheel_drive_current_speed;
	wheel_data[1] = -wheel_drive_current_speed;
	wheel_data[2] = wheel_drive_current_speed;
	wheel_data[3] = -wheel_drive_current_speed;

	send_sync_write_1(DXL_1_MOVING_SPEED, 2, wheel_data);

	if (wheel_drive_current_speed == wheel_drive_target_speed) {
		wheel_drive_accel_active = 0U;
	}
}

// Wheel Mode 전진
void Robot_Wheel_Forward(void) {
	if (!RobotPower_MotionAllowed()) return;
	if (robot_mode != MODE_WHEEL
			&& (robot_mode != MODE_RAMP || !WokiRampControl_IsDriveReady())) {
		Robot_Wheel_Stop();
		return;
	}

    if (robot_mode == MODE_RAMP && wheel_drive_target_speed == WHEEL_DRIVE_MAX_SPEED) {
        return; // 경사로 전진 명령 반복 시 가속 상태 유지
    }
	Robot_AX_Wheel_Mode_Setup();

	wheel_drive_target_speed = WHEEL_DRIVE_MAX_SPEED;
	wheel_drive_last_update_ms =
			HAL_GetTick() - WHEEL_DRIVE_ACCEL_PERIOD_MS;
	wheel_drive_accel_active = 1U;
}

// Wheel Mode 후진
void Robot_Wheel_Backward(void) {
	if (!RobotPower_MotionAllowed()) return;
	if (robot_mode != MODE_WHEEL
			&& (robot_mode != MODE_RAMP || !WokiRampControl_IsDriveReady())) {
		Robot_Wheel_Stop();
		return;
	}

    if (robot_mode == MODE_RAMP && wheel_drive_target_speed == -WHEEL_DRIVE_MAX_SPEED) {
        return; // 경사로 후진 명령 반복 시 가속 상태 유지
    }
	Robot_AX_Wheel_Mode_Setup();

	wheel_drive_target_speed = -WHEEL_DRIVE_MAX_SPEED;
	wheel_drive_last_update_ms =
			HAL_GetTick() - WHEEL_DRIVE_ACCEL_PERIOD_MS;
	wheel_drive_accel_active = 1U;
}

// 좌회전 탱크턴
void Robot_Tank_Left(void) {
	if (!RobotPower_MotionAllowed()) return;
	if (robot_mode != MODE_WHEEL) {
		Robot_Wheel_Stop();
		return;
	}

	wheel_drive_current_speed = 0;
	wheel_drive_target_speed = 0;
	wheel_drive_accel_active = 0U;

	const int16_t wheel_speed = 1023;

	int16_t wheel_data[4] = { -wheel_speed,  // ID 4  : 정면 기준 오른쪽 회전
			-wheel_speed,  // ID 14 : 정면 기준 오른쪽 회전
			-wheel_speed,  // ID 24 : 정면 기준 오른쪽 회전
			-wheel_speed   // ID 34 : 정면 기준 오른쪽 회전
			};

	Robot_AX_Wheel_Mode_Setup();
	send_sync_write_1(DXL_1_MOVING_SPEED, 2, wheel_data);
}

// 우회전 탱크턴
void Robot_Tank_Right(void) {
	if (!RobotPower_MotionAllowed()) return;
	if (robot_mode != MODE_WHEEL) {
		Robot_Wheel_Stop();
		return;
	}

	wheel_drive_current_speed = 0;
	wheel_drive_target_speed = 0;
	wheel_drive_accel_active = 0U;

	const int16_t wheel_speed = 1023;

	int16_t wheel_data[4] = { wheel_speed,  // ID 4  : 정면 기준 왼쪽 회전
			wheel_speed,  // ID 14 : 정면 기준 왼쪽 회전
			wheel_speed,  // ID 24 : 정면 기준 왼쪽 회전
			wheel_speed   // ID 34 : 정면 기준 왼쪽 회전
			};

	Robot_AX_Wheel_Mode_Setup();
	send_sync_write_1(DXL_1_MOVING_SPEED, 2, wheel_data);
}

// Wheel Mode 전환
void Robot_Set_Wheel_Mode(void) {
	if (!dxl_knee_position_is_ready()) return;
	if (robot_mode == MODE_WALK_DEMO) {
		return;
	}

	if (robot_mode == MODE_RAMP) {
		if (!RobotPower_CheckRunningReference()) return;
		Robot_Wheel_Stop();
        ramp_pending_mode = MODE_WHEEL;
        (void) WokiRampControl_RequestWheelReturn();
        Robot_Finish_Ramp_Return();
		return;
	}

	if (robot_mode == MODE_WHEEL || robot_mode == MODE_TO_WHEEL) {
		return;
	}

	if (robot_mode == MODE_TO_LEG) {
		return;
	}

	// 실제 전환을 시작할 때만 확인, 무시되는 중복 명령은 기존처럼 바로 반환
	if (robot_mode == MODE_NONE && !RobotPower_MoveInitialWheelPose()) return;
	if (robot_mode != MODE_NONE && !RobotPower_CheckRunningReference()) return;
	RobotMode_t previous_mode = robot_mode;
	robot_mode = MODE_TO_WHEEL;

	Robot_AX_Wheel_Mode_Setup();
	Robot_Wheel_Stop();

	if (previous_mode == MODE_LEG) {
		Robot_Run_Leg_To_Wheel_Trajectory();
	}
	// 최초 진입은 위에서 Wheel 실제 도착까지 완료, 중복 목표·토크 OFF/ON 없음

	// 전환 중 정지·초기 도착 확인 실패 시 Wheel Mode 완료 처리 방지
	if (user_stop_flag || !dxl_knee_position_is_ready()) {
		robot_mode = MODE_NONE;
		return;
	}

	Robot_Wheel_Stop();

	robot_mode = MODE_WHEEL;
	walk_demo_target_valid = 0U;
}

// 경사로 보정 모드 진입
void Robot_Set_Ramp_Mode(void) {
	// Wheel↔Leg 시험 허가만으로 미검증 경사로 동작까지 허용하지 않음
	if (WOKI_KNEE_REFERENCE_VERIFIED != 1U) return;
	if (!dxl_knee_position_is_ready()) return;
	if (robot_mode == MODE_RAMP) {
		Robot_Wheel_Stop();
		return;
	}

	if (robot_mode != MODE_WHEEL) {
		Robot_Wheel_Stop();
		return;
	}

	if (!RobotPower_CheckRunningReference()) return;
	Robot_Wheel_Stop();
	Robot_Set_Leg_Profile(RAMP_PROFILE_ACCELERATION,
			RAMP_PROFILE_VELOCITY);
	ramp_pending_mode = MODE_NONE;
	WokiRampControl_EnablePreview();
	robot_mode = MODE_RAMP;
}

// Leg Mode 전환
void Robot_Set_Leg_Mode(void) {
	if (!dxl_knee_position_is_ready()) return;
	// 시작 허가 직후에는 기존 Wheel 진입을 먼저 수행, 주차 자세에서 Leg 경로로 점프 방지
	if (robot_mode == MODE_NONE) return;
	if (robot_mode == MODE_WALK_DEMO) {
		return;
	}

	if (robot_mode == MODE_LEG || robot_mode == MODE_TO_LEG) {
		return;
	}

	if (robot_mode == MODE_TO_WHEEL) {
		return;
	}

	if (robot_mode == MODE_RAMP) {
		if (!RobotPower_CheckRunningReference()) return;
		Robot_Wheel_Stop();
        ramp_pending_mode = MODE_LEG;
        (void) WokiRampControl_RequestWheelReturn();
        Robot_Finish_Ramp_Return();
        return;
	}

	if (!RobotPower_CheckRunningReference()) return;
	robot_mode = MODE_TO_LEG;

	Robot_Wheel_Stop();
	Robot_Set_Leg_Pose();

	// 전환 중 비상정지 시 Leg Mode 완료 처리 방지
	if (user_stop_flag) {
		robot_mode = MODE_NONE;
		return;
	}

	robot_mode = MODE_LEG;
	walk_demo_target_valid = 0U;

}


static void Robot_Finish_Ramp_Return(void)
{
    RobotMode_t requested;
    if (ramp_pending_mode == MODE_NONE || !WokiRampControl_IsWheelReturnComplete())
        return;
    requested = ramp_pending_mode;
    ramp_pending_mode = MODE_NONE;
    Robot_Wheel_Stop();
    WokiRampControl_Disable();
	Robot_Set_Leg_Profile(WHEEL_PROFILE_ACCELERATION,
			WHEEL_PROFILE_VELOCITY);
    robot_mode = MODE_WHEEL;
    if (requested == MODE_LEG) Robot_Set_Leg_Mode();
}

// 기본 Wheel 자세 및 경사로 복귀 목표
void Robot_Get_Wheel_Pose_Targets(int32_t target[12])
{
    const uint32_t wheel_pose[12] = {
        2048, 2048, 2691,  // leg1: FL
        2048, 2048, 1405,  // leg2: FR
        2048, 2048, 2691,  // leg3: RL
        2048, 2048, 1405   // leg4: RR
    };
    uint32_t i;
    if (target == 0) return;
    for (i = 0; i < 12U; ++i)
        target[i] = WokiJoint_IsKneeIndex(i)
                ? WokiJoint_RoundRaw(WokiJoint_KneeReference(i / 3U))
                : (int32_t)wheel_pose[i];
}

// 최초 Wheel 자세 이동, 현재 위치에서 직접 저속 이동 및 실제 도착 확인
void Robot_Set_Wheel_Pose(void) {
	if (!dxl_knee_position_is_ready()) return;
	if (robot_mode != MODE_NONE) return;
	(void)RobotPower_MoveInitialWheelPose();
}

// MX 목표 위치 범위 제한
static int32_t Robot_Wheel_To_Leg_Clamp_Position(float position) {
	if (position < 0.0f) {
		return 0;
	}

	if (position > 4095.0f) {
		return 4095;
	}

	return WokiJoint_RoundRaw(position);
}

// 최소저크 시간 진행률 계산
static float Robot_Wheel_To_Leg_Minimum_Jerk(float progress) {
	if (progress < 0.0f) {
		progress = 0.0f;
	} else if (progress > 1.0f) {
		progress = 1.0f;
	}

	return 10.0f * progress * progress * progress
			- 15.0f * progress * progress * progress * progress
			+ 6.0f * progress * progress * progress * progress * progress;
}

// Hip Pitch 각도를 MX 목표 위치로 변환
static int32_t Robot_Wheel_To_Leg_Pitch_To_Position(float pitch_deg,
		uint8_t left_side) {
	float position_delta = pitch_deg * WHEEL_TO_LEG_PITCH_POSITION_PER_DEG;
	float position = left_side ?
			WHEEL_TO_LEG_PITCH_ZERO_POSITION + position_delta :
			WHEEL_TO_LEG_PITCH_ZERO_POSITION - position_delta;

	return Robot_Wheel_To_Leg_Clamp_Position(position);
}

// Knee 출력축 각도를 MX 목표 위치로 변환
static int32_t Robot_Wheel_To_Leg_Knee_To_Position(float knee_deg,
		uint8_t leg) {
	float position_delta = (knee_deg - WHEEL_TO_LEG_KNEE_START_DEG)
			* WHEEL_TO_LEG_KNEE_POSITION_PER_DEG;

	float position = WokiJoint_KneeReference(leg)
			+ (leg % 2U == 0U ? -position_delta : position_delta);
	const int32_t minimum = WokiJoint_MinRaw(leg * 3U + 2U);
	const int32_t maximum = WokiJoint_MaxRaw(leg * 3U + 2U);
	// 기존 관절각 허용 범위만 환산, KNEE를 0~4095로 자르지 않음
	if (position < (float)minimum) return minimum;
	if (position > (float)maximum) return maximum;
	return WokiJoint_RoundRaw(position);
}

// 접촉 후 뒷다리 121점 경로 보간
static void Robot_Wheel_To_Leg_Get_Rear_Post_Path(float progress, float *pitch_deg,
		float *knee_deg) {
	if (progress <= 0.0f) {
		*pitch_deg = wheel_to_leg_rear_post_path_deg[0][0];
		*knee_deg = wheel_to_leg_rear_post_path_deg[0][1];
		return;
	}

	if (progress >= 1.0f) {
		*pitch_deg =
				wheel_to_leg_rear_post_path_deg[WHEEL_TO_LEG_POST_PATH_POINT_COUNT - 1U][0];
		*knee_deg =
				wheel_to_leg_rear_post_path_deg[WHEEL_TO_LEG_POST_PATH_POINT_COUNT - 1U][1];
		return;
	}

	float point_position = progress
			* (float) (WHEEL_TO_LEG_POST_PATH_POINT_COUNT - 1U);
	uint32_t point_index = (uint32_t) point_position;
	float point_ratio = point_position - (float) point_index;

	*pitch_deg = wheel_to_leg_rear_post_path_deg[point_index][0]
			+ (wheel_to_leg_rear_post_path_deg[point_index + 1U][0]
					- wheel_to_leg_rear_post_path_deg[point_index][0]) * point_ratio;

	*knee_deg = wheel_to_leg_rear_post_path_deg[point_index][1]
			+ (wheel_to_leg_rear_post_path_deg[point_index + 1U][1]
					- wheel_to_leg_rear_post_path_deg[point_index][1]) * point_ratio;
}

// 경과시간에 따른 Pitch와 Knee 목표 위치 계산
static void Robot_Wheel_To_Leg_Calculate_Target(uint32_t elapsed_ms,
		int32_t *pitch_position, int32_t *knee_position) {
	float front_pitch_deg;
	float front_knee_deg;
	float rear_pitch_deg;
	float rear_knee_deg;
	float path_progress;

	if (elapsed_ms <= WHEEL_TO_LEG_APPROACH_TIME_MS) {
		float time_progress = (float) elapsed_ms
				/ (float) WHEEL_TO_LEG_APPROACH_TIME_MS;

		path_progress = Robot_Wheel_To_Leg_Minimum_Jerk(time_progress);

		front_pitch_deg = WHEEL_TO_LEG_CONTACT_PITCH_DEG * path_progress;
		front_knee_deg = WHEEL_TO_LEG_KNEE_START_DEG
				+ (WHEEL_TO_LEG_CONTACT_KNEE_DEG - WHEEL_TO_LEG_KNEE_START_DEG)
						* path_progress;

		rear_pitch_deg = front_pitch_deg;
		rear_knee_deg = front_knee_deg;
	} else {
		uint32_t post_elapsed_ms = elapsed_ms - WHEEL_TO_LEG_APPROACH_TIME_MS;
		float time_progress = (float) post_elapsed_ms
				/ (float) WHEEL_TO_LEG_STAND_TIME_MS;

		path_progress = Robot_Wheel_To_Leg_Minimum_Jerk(time_progress);

		front_pitch_deg = WHEEL_TO_LEG_CONTACT_PITCH_DEG
				+ (WHEEL_TO_LEG_FINAL_FRONT_PITCH_DEG - WHEEL_TO_LEG_CONTACT_PITCH_DEG)
						* path_progress;

		front_knee_deg = WHEEL_TO_LEG_CONTACT_KNEE_DEG
				+ (WHEEL_TO_LEG_FINAL_FRONT_KNEE_DEG - WHEEL_TO_LEG_CONTACT_KNEE_DEG)
						* path_progress;

		Robot_Wheel_To_Leg_Get_Rear_Post_Path(path_progress, &rear_pitch_deg,
				&rear_knee_deg);
	}

	// Pitch 목표 위치 순서: FL, FR, RL, RR
	pitch_position[0] = Robot_Wheel_To_Leg_Pitch_To_Position(front_pitch_deg, 1);
	pitch_position[1] = Robot_Wheel_To_Leg_Pitch_To_Position(front_pitch_deg, 0);
	pitch_position[2] = Robot_Wheel_To_Leg_Pitch_To_Position(rear_pitch_deg, 1);
	pitch_position[3] = Robot_Wheel_To_Leg_Pitch_To_Position(rear_pitch_deg, 0);

	// Knee 목표 위치 순서: FL, FR, RL, RR
	knee_position[0] = Robot_Wheel_To_Leg_Knee_To_Position(front_knee_deg, 0U);
	knee_position[1] = Robot_Wheel_To_Leg_Knee_To_Position(front_knee_deg, 1U);
	knee_position[2] = Robot_Wheel_To_Leg_Knee_To_Position(rear_knee_deg, 2U);
	knee_position[3] = Robot_Wheel_To_Leg_Knee_To_Position(rear_knee_deg, 3U);
}

// Leg-to-Wheel Pitch·Knee 목표 위치 계산
static void Robot_Leg_To_Wheel_Calculate_Target(uint32_t elapsed_ms,
		int32_t *pitch_position, int32_t *knee_position) {
	float front_pitch_deg;
	float front_knee_deg;
	float rear_pitch_deg;
	float rear_knee_deg;
	float path_progress;

	if (elapsed_ms <= LEG_TO_WHEEL_FOOT_FIXED_TIME_MS) {
		float time_progress = (float) elapsed_ms
				/ (float) LEG_TO_WHEEL_FOOT_FIXED_TIME_MS;

		path_progress = Robot_Wheel_To_Leg_Minimum_Jerk(time_progress);

		front_pitch_deg = WHEEL_TO_LEG_FINAL_FRONT_PITCH_DEG
				+ (WHEEL_TO_LEG_CONTACT_PITCH_DEG
						- WHEEL_TO_LEG_FINAL_FRONT_PITCH_DEG) * path_progress;

		front_knee_deg = WHEEL_TO_LEG_FINAL_FRONT_KNEE_DEG
				+ (WHEEL_TO_LEG_CONTACT_KNEE_DEG
						- WHEEL_TO_LEG_FINAL_FRONT_KNEE_DEG) * path_progress;

		// 뒷다리 121점 경로 역방향 보간
		Robot_Wheel_To_Leg_Get_Rear_Post_Path(1.0f - path_progress,
				&rear_pitch_deg, &rear_knee_deg);
	} else {
		uint32_t wheel_elapsed_ms = elapsed_ms
				- LEG_TO_WHEEL_FOOT_FIXED_TIME_MS;
		float time_progress = (float) wheel_elapsed_ms
				/ (float) LEG_TO_WHEEL_WHEEL_FIXED_TIME_MS;

		path_progress = Robot_Wheel_To_Leg_Minimum_Jerk(time_progress);

		front_pitch_deg = WHEEL_TO_LEG_CONTACT_PITCH_DEG
				* (1.0f - path_progress);
		front_knee_deg = WHEEL_TO_LEG_CONTACT_KNEE_DEG
				+ (WHEEL_TO_LEG_KNEE_START_DEG
						- WHEEL_TO_LEG_CONTACT_KNEE_DEG) * path_progress;

		rear_pitch_deg = front_pitch_deg;
		rear_knee_deg = front_knee_deg;
	}

	// Pitch 목표 위치 순서: FL, FR, RL, RR
	pitch_position[0] = Robot_Wheel_To_Leg_Pitch_To_Position(front_pitch_deg, 1);
	pitch_position[1] = Robot_Wheel_To_Leg_Pitch_To_Position(front_pitch_deg, 0);
	pitch_position[2] = Robot_Wheel_To_Leg_Pitch_To_Position(rear_pitch_deg, 1);
	pitch_position[3] = Robot_Wheel_To_Leg_Pitch_To_Position(rear_pitch_deg, 0);

	// Knee 목표 위치 순서: FL, FR, RL, RR
	knee_position[0] = Robot_Wheel_To_Leg_Knee_To_Position(front_knee_deg, 0U);
	knee_position[1] = Robot_Wheel_To_Leg_Knee_To_Position(front_knee_deg, 1U);
	knee_position[2] = Robot_Wheel_To_Leg_Knee_To_Position(rear_knee_deg, 2U);
	knee_position[3] = Robot_Wheel_To_Leg_Knee_To_Position(rear_knee_deg, 3U);
}

// 실행 중 사용자 정지 요청 확인
static uint8_t Robot_Wheel_To_Leg_Stop_Requested(void) {
	if (!user_stop_flag) {
		return 0;
	}

	Robot_Wheel_Stop();
	DXL_Emergency_All_Off();

	return 1;
}

// Leg-to-Wheel 궤적 실행
static void Robot_Run_Leg_To_Wheel_Trajectory(void) {
	if (!dxl_knee_position_is_ready()) return;
	uint8_t roll_ids[4] = {
			legs[1].roll, legs[2].roll,
			legs[3].roll, legs[4].roll
			};

	uint8_t pitch_ids[4] = {
			legs[1].pitch, legs[2].pitch,
			legs[3].pitch, legs[4].pitch
			};

	uint8_t knee_ids[4] = {
			legs[1].knee, legs[2].knee,
			legs[3].knee, legs[4].knee
			};

	int32_t roll_pose[4] = {
			2048,  // ID 1  : FL Roll
			2048,  // ID 11 : FR Roll
			2048,  // ID 21 : RL Roll
			2048   // ID 31 : RR Roll
			};

	uint32_t profile_disabled[4] = { 0, 0, 0, 0 };
	uint8_t torque_on[4] = { 1, 1, 1, 1 };
	int32_t pitch_position[4];
	int32_t knee_position[4];

	// 전환 중 AX 바퀴 정지 유지
	Robot_Wheel_Stop();

	// 외부 궤적 사용을 위해 MX 내부 프로파일 해제
	dxl_sync_write_4byte(pitch_ids, profile_disabled, 4,
			DXL_2_Profile_Acceleration);
	dxl_sync_write_4byte(pitch_ids, profile_disabled, 4,
			DXL_2_Profile_Velocity);
	dxl_sync_write_4byte(knee_ids, profile_disabled, 4,
			DXL_2_Profile_Acceleration);
	dxl_sync_write_4byte(knee_ids, profile_disabled, 4,
			DXL_2_Profile_Velocity);

	// Roll, Pitch, Knee 모터 토크 활성화
	dxl_sync_write_1byte(roll_ids, torque_on, 4, DXL_2_Torque_Enable);
	dxl_sync_write_1byte(pitch_ids, torque_on, 4, DXL_2_Torque_Enable);
	dxl_sync_write_1byte(knee_ids, torque_on, 4, DXL_2_Torque_Enable);

	// Roll의 Wheel 자세 영점 유지
	(void)dxl_sync_write_goal_positions_checked(roll_ids, roll_pose, 4U);

	uint32_t start_tick = HAL_GetTick();

	// 궤적을 100Hz 주기로 실행
	for (uint32_t elapsed_ms = 0;
			elapsed_ms <= LEG_TO_WHEEL_TOTAL_TIME_MS;
			elapsed_ms += LEG_TO_WHEEL_CONTROL_PERIOD_MS) {
		uint32_t target_tick = start_tick + elapsed_ms;

		while ((int32_t) (HAL_GetTick() - target_tick) < 0) {
			if (Robot_Wheel_To_Leg_Stop_Requested()) {
				return;
			}

			HAL_Delay(1);
		}

		if (Robot_Wheel_To_Leg_Stop_Requested()) {
			return;
		}

		Robot_Leg_To_Wheel_Calculate_Target(elapsed_ms, pitch_position,
				knee_position);

		(void)dxl_sync_write_goal_positions_checked(pitch_ids, pitch_position, 4U);
		(void)dxl_sync_write_goal_positions_checked(knee_ids, knee_position, 4U);
	}
}

// Leg Mode 자세 설정
void Robot_Set_Leg_Pose(void) {
	if (!dxl_knee_position_is_ready()) return;
	if (robot_mode == MODE_NONE) return;
	uint8_t roll_ids[4] = {
			legs[1].roll, legs[2].roll,
			legs[3].roll, legs[4].roll
			};

	uint8_t pitch_ids[4] = {
			legs[1].pitch, legs[2].pitch,
			legs[3].pitch, legs[4].pitch
			};

	uint8_t knee_ids[4] = {
			legs[1].knee, legs[2].knee,
			legs[3].knee, legs[4].knee
			};

	int32_t roll_pose[4] = {
			2048,  // ID 1  : FL Roll
			2048,  // ID 11 : FR Roll
			2048,  // ID 21 : RL Roll
			2048   // ID 31 : RR Roll
			};

	uint32_t profile_disabled[4] = { 0, 0, 0, 0 };
	uint8_t torque_on[4] = { 1, 1, 1, 1 };
	int32_t pitch_position[4];
	int32_t knee_position[4];

	// 무부하 시험에서는 AX 바퀴 출력을 0으로 유지
	Robot_Wheel_Stop();

	// 외부 궤적 사용을 위해 MX 내부 프로파일 해제
	dxl_sync_write_4byte(pitch_ids, profile_disabled, 4,
			DXL_2_Profile_Acceleration);
	dxl_sync_write_4byte(pitch_ids, profile_disabled, 4,
			DXL_2_Profile_Velocity);
	dxl_sync_write_4byte(knee_ids, profile_disabled, 4,
			DXL_2_Profile_Acceleration);
	dxl_sync_write_4byte(knee_ids, profile_disabled, 4,
			DXL_2_Profile_Velocity);

	// Roll, Pitch, Knee 모터 토크 활성화
	dxl_sync_write_1byte(roll_ids, torque_on, 4, DXL_2_Torque_Enable);
	dxl_sync_write_1byte(pitch_ids, torque_on, 4, DXL_2_Torque_Enable);
	dxl_sync_write_1byte(knee_ids, torque_on, 4, DXL_2_Torque_Enable);

	// Roll 목표 위치 설정
	(void)dxl_sync_write_goal_positions_checked(roll_ids, roll_pose, 4U);

	uint32_t start_tick = HAL_GetTick();

	// 궤적을 100Hz 주기로 실행
	for (uint32_t elapsed_ms = 0;
			elapsed_ms <= WHEEL_TO_LEG_TOTAL_TIME_MS;
			elapsed_ms += WHEEL_TO_LEG_CONTROL_PERIOD_MS) {
		uint32_t target_tick = start_tick + elapsed_ms;

		while ((int32_t) (HAL_GetTick() - target_tick) < 0) {
			if (Robot_Wheel_To_Leg_Stop_Requested()) {
				return;
			}

			HAL_Delay(1);
		}

		if (Robot_Wheel_To_Leg_Stop_Requested()) {
			return;
		}

		Robot_Wheel_To_Leg_Calculate_Target(elapsed_ms, pitch_position,
				knee_position);

		(void)dxl_sync_write_goal_positions_checked(pitch_ids, pitch_position, 4U);
		(void)dxl_sync_write_goal_positions_checked(knee_ids, knee_position, 4U);
	}
}

// 촬영용 보행 모터 순서 설정
static void Robot_Walk_Demo_Get_Ids(uint8_t ids[12]) {
	ids[0] = legs[1].roll;
	ids[1] = legs[1].pitch;
	ids[2] = legs[1].knee;
	ids[3] = legs[2].roll;
	ids[4] = legs[2].pitch;
	ids[5] = legs[2].knee;
	ids[6] = legs[3].roll;
	ids[7] = legs[3].pitch;
	ids[8] = legs[3].knee;
	ids[9] = legs[4].roll;
	ids[10] = legs[4].pitch;
	ids[11] = legs[4].knee;
}

// Leg 자세 목표 위치 계산
static void Robot_Walk_Demo_Get_Leg_Target(int32_t target[12]) {
	int32_t pitch_position[4];
	int32_t knee_position[4];

	Robot_Wheel_To_Leg_Calculate_Target(WHEEL_TO_LEG_TOTAL_TIME_MS,
			pitch_position, knee_position);

	for (uint32_t i = 0U; i < 4U; ++i) {
		target[i * 3U] = 2048U;
		target[i * 3U + 1U] = pitch_position[i];
		target[i * 3U + 2U] = knee_position[i];
	}
}

// 촬영용 보행 목표 위치 전송
static void Robot_Walk_Demo_Send_Target(void) {
	uint8_t ids[12];
	int32_t send_target[12];

	Robot_Walk_Demo_Get_Ids(ids);
	for (uint32_t i = 0U; i < 12U; ++i) {
		send_target[i] = walk_demo_target_raw[i];
	}
	const int32_t id3_minimum = WokiJoint_LegacyPosition(WALK_DEMO_ID3_INDEX,
			(float)WALK_DEMO_ID3_MIN_RAW);
	if (send_target[WALK_DEMO_ID3_INDEX] < id3_minimum) {
		send_target[WALK_DEMO_ID3_INDEX] = id3_minimum;
	}
	if (!dxl_sync_write_goal_positions_checked(ids, send_target, 12U)) {
		robot_walk_demo_status.tx_fail_count++;
	}
}

// 촬영용 3스텝 시작
uint8_t Robot_Start_Walk_Demo(void) {
	// Wheel↔Leg 시험 중에는 촬영용 보행을 시작하지 않음
	if (WOKI_KNEE_REFERENCE_VERIFIED != 1U) return 0U;
	if (!dxl_knee_position_is_ready()) return 0U;
	uint8_t ids[12];
	uint8_t torque_on[12];
	uint32_t profile_disabled[12];

	if (robot_mode != MODE_LEG || robot_walk_demo_status.active) {
		return 0U;
	}
	if (!RobotPower_CheckRunningReference()) return 0U;

	Robot_Wheel_Stop();
	Robot_Walk_Demo_Get_Ids(ids);

	if (!walk_demo_target_valid) {
		Robot_Walk_Demo_Get_Leg_Target(walk_demo_target_raw);
		walk_demo_target_valid = 1U;
	}

	for (uint32_t i = 0U; i < 12U; ++i) {
		walk_demo_start_raw[i] = walk_demo_target_raw[i];
		profile_disabled[i] = 0U;
		torque_on[i] = 1U;
	}

	// 외부 궤적 사용을 위해 MX 내부 프로파일 해제
	dxl_sync_write_4byte(ids, profile_disabled, 12U,
			DXL_2_Profile_Acceleration);
	dxl_sync_write_4byte(ids, profile_disabled, 12U,
			DXL_2_Profile_Velocity);
	dxl_sync_write_1byte(ids, torque_on, 12U, DXL_2_Torque_Enable);

	walk_demo_phase_start_ms = HAL_GetTick();
	walk_demo_last_update_ms = walk_demo_phase_start_ms;
	walk_demo_speed_accumulator = 0U;
	robot_walk_demo_status.active = 1U;
	robot_walk_demo_status.state = WALK_DEMO_ENTERING;
	robot_walk_demo_status.sample_index = 0U;
	robot_walk_demo_status.elapsed_ms = 0U;
	robot_walk_demo_status.tx_fail_count = 0U;
	robot_mode = MODE_WALK_DEMO;

	return 1U;
}

// 촬영용 3스텝 정지
void Robot_Stop_Walk_Demo(void) {
	if (robot_mode != MODE_WALK_DEMO) {
		return;
	}

	Robot_Wheel_Stop();
	robot_walk_demo_status.active = 0U;
	robot_walk_demo_status.state = WALK_DEMO_STOPPED;
	robot_mode = MODE_LEG;
}

// 압축된 변화량에서 원본 샘플 한 개 적용
static void Robot_Walk_Demo_Apply_Next_Delta(void) {
	if (walk_demo_zero_remaining > 0U) {
		walk_demo_zero_remaining--;
		return;
	}
	if (walk_demo_delta_event_index >= WOKI_WALK_DEMO_DELTA_EVENT_COUNT) {
		return; // 마지막 목표 유지 구간
	}

	const WokiWalkDemoDeltaEvent *event =
			&woki_walk_demo_delta_events[walk_demo_delta_event_index];
	for (uint32_t i = 0U; i < 12U; ++i) {
		walk_demo_legacy_target_raw[i] +=
				(int32_t)event->delta_raw[i] * WALK_DEMO_ANGLE_SCALE;
		walk_demo_target_raw[i] = WokiJoint_LegacyPosition(i,
				(float)walk_demo_legacy_target_raw[i]);
	}
	walk_demo_delta_event_index++;
	if (walk_demo_delta_event_index < WOKI_WALK_DEMO_DELTA_EVENT_COUNT) {
		walk_demo_zero_remaining =
				woki_walk_demo_delta_events[walk_demo_delta_event_index].zero_before;
	}
}

// 촬영용 3스텝 제어
void Robot_Walk_Demo_Task(void) {
	if (!RobotPower_MotionAllowed()) return;
	uint32_t current_time;

	if (robot_mode != MODE_WALK_DEMO || !robot_walk_demo_status.active) {
		return;
	}

	current_time = HAL_GetTick();
	if ((uint32_t) (current_time - walk_demo_last_update_ms)
			< WOKI_WALK_DEMO_CONTROL_PERIOD_MS) {
		return;
	}
	walk_demo_last_update_ms = current_time;

	if (robot_walk_demo_status.state == WALK_DEMO_ENTERING) {
		uint32_t elapsed_ms = current_time - walk_demo_phase_start_ms;
		float progress;

		if (elapsed_ms >= WALK_DEMO_ENTER_TIME_MS) {
			for (uint32_t i = 0U; i < 12U; ++i) {
				walk_demo_legacy_target_raw[i] = woki_walk_demo_first_raw[i];
				walk_demo_target_raw[i] = WokiJoint_LegacyPosition(i,
						(float)walk_demo_legacy_target_raw[i]);
			}
			Robot_Walk_Demo_Send_Target();
			walk_demo_phase_start_ms = current_time;
			walk_demo_speed_accumulator = 0U;
			walk_demo_delta_event_index = 0U;
			walk_demo_zero_remaining = woki_walk_demo_delta_events[0].zero_before;
			robot_walk_demo_status.state = WALK_DEMO_PLAYING;
			robot_walk_demo_status.elapsed_ms = 0U;
			return;
		}

		progress = Robot_Wheel_To_Leg_Minimum_Jerk(
				(float) elapsed_ms / (float) WALK_DEMO_ENTER_TIME_MS);
		for (uint32_t i = 0U; i < 12U; ++i) {
			float position = (float) walk_demo_start_raw[i]
					+ (WokiJoint_FromLegacyRaw(i, (float)woki_walk_demo_first_raw[i])
							- (float) walk_demo_start_raw[i]) * progress;
			walk_demo_target_raw[i] = WokiJoint_RoundRaw(position);
		}
		robot_walk_demo_status.elapsed_ms = elapsed_ms;
		Robot_Walk_Demo_Send_Target();
		return;
	}

	if (robot_walk_demo_status.state != WALK_DEMO_PLAYING) {
		return;
	}

	walk_demo_speed_accumulator += WALK_DEMO_SPEED_PERCENT;
	uint32_t advance_count = walk_demo_speed_accumulator / 100U;
	walk_demo_speed_accumulator %= 100U;

	while (advance_count > 0U
			&& robot_walk_demo_status.sample_index + 1U
					< WOKI_WALK_DEMO_SAMPLE_COUNT) {
		Robot_Walk_Demo_Apply_Next_Delta();
		robot_walk_demo_status.sample_index++;
		advance_count--;
	}

	robot_walk_demo_status.elapsed_ms =
			robot_walk_demo_status.sample_index
					* WOKI_WALK_DEMO_CONTROL_PERIOD_MS;
	Robot_Walk_Demo_Send_Target();

	if (robot_walk_demo_status.sample_index + 1U
			>= WOKI_WALK_DEMO_SAMPLE_COUNT) {
		robot_walk_demo_status.active = 0U;
		robot_walk_demo_status.state = WALK_DEMO_FINISHED;
		robot_mode = MODE_LEG;
	}
}
