/*
 * robot_motion.c
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */
#include "robot_motion.h"
#include "dxl_2_0.h"
#include "wheel_to_leg_trajectory_data.h"

//AX Wheel Mode 초기 설정 저장
static uint8_t ax_wheel_mode_ready = 0;
// 전원 인가 후 최초 Wheel 자세 전환 여부
static uint8_t first_wheel_pose = 1;
// 시작 자세 Mode_NONE 저장
static volatile RobotMode_t robot_mode = MODE_NONE;

extern volatile uint8_t user_stop_flag;

#define WHEEL_POSE_WAIT_TIME_MS 3000

// 수식 기반 Wheel-to-Leg 무부하 시험 제어 주기 및 1.5배 시간 설정
#define WHEEL_TO_LEG_CONTROL_PERIOD_MS  10U
#define WHEEL_TO_LEG_APPROACH_TIME_MS   3310U
#define WHEEL_TO_LEG_STAND_TIME_MS      9000U
#define WHEEL_TO_LEG_TOTAL_TIME_MS      (WHEEL_TO_LEG_APPROACH_TIME_MS + WHEEL_TO_LEG_STAND_TIME_MS)

// Wheel-to-Leg 계산 각도
#define WHEEL_TO_LEG_KNEE_START_DEG          22.0f
#define WHEEL_TO_LEG_CONTACT_PITCH_DEG       38.1406444612f
#define WHEEL_TO_LEG_CONTACT_KNEE_DEG        33.2637871388f
#define WHEEL_TO_LEG_FINAL_FRONT_PITCH_DEG   30.1343974072f
#define WHEEL_TO_LEG_FINAL_FRONT_KNEE_DEG    94.6463063272f

// MX 영점 및 각도-엔코더 환산값
#define WHEEL_TO_LEG_PITCH_ZERO_POSITION       2048.0f
#define WHEEL_TO_LEG_LEFT_KNEE_ZERO_POSITION   2909.0f
#define WHEEL_TO_LEG_RIGHT_KNEE_ZERO_POSITION  1187.0f
#define WHEEL_TO_LEG_PITCH_POSITION_PER_DEG    (4096.0f / 360.0f)
#define WHEEL_TO_LEG_KNEE_POSITION_PER_DEG     (2.0f * 4096.0f / 360.0f)

// 로봇 동작 초기화
void Robot_Motion_Init(void) {
}

// 로봇 현재 모드 확인
RobotMode_t Robot_Get_Mode(void) {
	return robot_mode;
}

// AX 바퀴 정지
void Robot_Wheel_Stop(void) {
	int16_t wheel_speed[4] = { 0, 0, 0, 0 };

	send_sync_write_1(DXL_1_MOVING_SPEED, 2, wheel_speed);
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

// Wheel Mode 전진
void Robot_Wheel_Forward(void) {
	if (robot_mode != MODE_WHEEL) {
		Robot_Wheel_Stop();
		return;
	}

	const int16_t wheel_speed = 750;

	int16_t wheel_data[4] = { wheel_speed,   // ID 4  : 정면 기준 왼쪽 회전
			-wheel_speed,  // ID 14 : 정면 기준 오른쪽 회전
			wheel_speed,   // ID 24 : 정면 기준 왼쪽 회전
			-wheel_speed   // ID 34 : 정면 기준 오른쪽 회전
			};

	Robot_AX_Wheel_Mode_Setup();
	send_sync_write_1(DXL_1_MOVING_SPEED, 2, wheel_data);
}

// Wheel Mode 후진
void Robot_Wheel_Backward(void) {
	if (robot_mode != MODE_WHEEL) {
		Robot_Wheel_Stop();
		return;
	}

	const int16_t wheel_speed = 750;

	int16_t wheel_data[4] = { -wheel_speed,  // ID 4  : 정면 기준 오른쪽 회전
			wheel_speed,   // ID 14 : 정면 기준 왼쪽 회전
			-wheel_speed,  // ID 24 : 정면 기준 오른쪽 회전
			wheel_speed    // ID 34 : 정면 기준 왼쪽 회전
			};

	Robot_AX_Wheel_Mode_Setup();
	send_sync_write_1(DXL_1_MOVING_SPEED, 2, wheel_data);
}

// 좌회전 탱크턴
void Robot_Tank_Left(void) {
	if (robot_mode != MODE_WHEEL) {
		Robot_Wheel_Stop();
		return;
	}

	const int16_t wheel_speed = 600;

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
	if (robot_mode != MODE_WHEEL) {
		Robot_Wheel_Stop();
		return;
	}

	const int16_t wheel_speed = 600;

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
	if (robot_mode == MODE_WHEEL || robot_mode == MODE_TO_WHEEL) {
		return;
	}

	if (robot_mode == MODE_TO_LEG) {
		return;
	}

	robot_mode = MODE_TO_WHEEL;

	Robot_AX_Wheel_Mode_Setup();
	Robot_Wheel_Stop();

	Robot_Set_Wheel_Pose();

	Robot_Wheel_Stop();

	robot_mode = MODE_WHEEL;
}

// Leg Mode 전환
void Robot_Set_Leg_Mode(void) {
	if (robot_mode == MODE_LEG || robot_mode == MODE_TO_LEG) {
		return;
	}

	if (robot_mode == MODE_TO_WHEEL) {
		return;
	}

	robot_mode = MODE_TO_LEG;

	Robot_Wheel_Stop();
	Robot_Set_Leg_Pose();

	// 전환 중 비상정지 시 Leg Mode 완료 처리 방지
	if (user_stop_flag) {
		robot_mode = MODE_NONE;
		return;
	}

	robot_mode = MODE_LEG;

}

// Wheel Mode 자세 설정
void Robot_Set_Wheel_Pose(void) {
	uint8_t ids[12] = { legs[1].roll, legs[1].pitch, legs[1].knee, legs[2].roll,
			legs[2].pitch, legs[2].knee, legs[3].roll, legs[3].pitch,
			legs[3].knee, legs[4].roll, legs[4].pitch, legs[4].knee };

	uint32_t wheel_pose[12] = {
	        2048, 2048, 2909,  // leg1: FL
	        1963, 2048, 1187,  // leg2: FR, Roll 임시 영점 1963
	        2048, 2048, 2909,  // leg3: RL
	        2048, 2048, 1187   // leg4: RR
	        };

	uint32_t profile_acc[12];
	uint32_t profile_vel[12];

	uint32_t acc_value = first_wheel_pose ? 2 : 5;
	uint32_t vel_value = first_wheel_pose ? 5 : 10;

	for (int i = 0; i < 12; i++) {
	    profile_acc[i] = acc_value;
	    profile_vel[i] = vel_value;
	}

	uint8_t torque_off[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	uint8_t torque_on[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

	dxl_sync_write_1byte(ids, torque_off, 12, DXL_2_Torque_Enable);
	HAL_Delay(20);

	dxl_sync_write_4byte(ids, profile_acc, 12, DXL_2_Profile_Acceleration);
	HAL_Delay(20);

	dxl_sync_write_4byte(ids, profile_vel, 12, DXL_2_Profile_Velocity);
	HAL_Delay(20);

	dxl_sync_write_1byte(ids, torque_on, 12, DXL_2_Torque_Enable);
	HAL_Delay(100);

	dxl_sync_write_4byte(ids, wheel_pose, 12, DXL_2_Goal_Position);
	HAL_Delay(WHEEL_POSE_WAIT_TIME_MS);

	// 이후 Wheel 전환부터는 기존 프로파일 사용
	first_wheel_pose = 0;
}

// MX 목표 위치 범위 제한
static uint32_t Robot_Wheel_To_Leg_Clamp_Position(float position) {
	if (position < 0.0f) {
		return 0;
	}

	if (position > 4095.0f) {
		return 4095;
	}

	return (uint32_t) (position + 0.5f);
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
static uint32_t Robot_Wheel_To_Leg_Pitch_To_Position(float pitch_deg,
		uint8_t left_side) {
	float position_delta = pitch_deg * WHEEL_TO_LEG_PITCH_POSITION_PER_DEG;
	float position = left_side ?
			WHEEL_TO_LEG_PITCH_ZERO_POSITION + position_delta :
			WHEEL_TO_LEG_PITCH_ZERO_POSITION - position_delta;

	return Robot_Wheel_To_Leg_Clamp_Position(position);
}

// Knee 출력축 각도를 MX 목표 위치로 변환
static uint32_t Robot_Wheel_To_Leg_Knee_To_Position(float knee_deg,
		uint8_t left_side) {
	float position_delta = (knee_deg - WHEEL_TO_LEG_KNEE_START_DEG)
			* WHEEL_TO_LEG_KNEE_POSITION_PER_DEG;

	float position = left_side ?
			WHEEL_TO_LEG_LEFT_KNEE_ZERO_POSITION - position_delta :
			WHEEL_TO_LEG_RIGHT_KNEE_ZERO_POSITION + position_delta;

	return Robot_Wheel_To_Leg_Clamp_Position(position);
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
		uint32_t *pitch_position, uint32_t *knee_position) {
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
	knee_position[0] = Robot_Wheel_To_Leg_Knee_To_Position(front_knee_deg, 1);
	knee_position[1] = Robot_Wheel_To_Leg_Knee_To_Position(front_knee_deg, 0);
	knee_position[2] = Robot_Wheel_To_Leg_Knee_To_Position(rear_knee_deg, 1);
	knee_position[3] = Robot_Wheel_To_Leg_Knee_To_Position(rear_knee_deg, 0);
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

// Leg Mode 자세 설정
void Robot_Set_Leg_Pose(void) {
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

	uint32_t roll_pose[4] = {
			2048,  // ID 1  : FL Roll
			1963,  // ID 11 : FR Roll 임시 영점
			2048,  // ID 21 : RL Roll
			2048   // ID 31 : RR Roll
			};

	uint32_t profile_disabled[4] = { 0, 0, 0, 0 };
	uint8_t torque_on[4] = { 1, 1, 1, 1 };
	uint32_t pitch_position[4];
	uint32_t knee_position[4];

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
	dxl_sync_write_4byte(roll_ids, roll_pose, 4, DXL_2_Goal_Position);

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

		dxl_sync_write_4byte(pitch_ids, pitch_position, 4,
				DXL_2_Goal_Position);
		dxl_sync_write_4byte(knee_ids, knee_position, 4,
				DXL_2_Goal_Position);
	}
}
