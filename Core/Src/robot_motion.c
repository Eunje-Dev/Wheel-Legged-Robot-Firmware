/*
 * robot_motion.c
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */
#include "robot_motion.h"
#include "dxl_2_0.h"

//AX Wheel Mode 초기 설정 저장
static uint8_t ax_wheel_mode_ready = 0;
// 전원 인가 후 최초 Wheel 자세 전환 여부
static uint8_t first_wheel_pose = 1;
// 시작 자세 Mode_NONE 저장
static volatile RobotMode_t robot_mode = MODE_NONE;

extern volatile uint8_t user_stop_flag;

#define WHEEL_POSE_WAIT_TIME_MS   3000
#define LEG_PITCH_ASSIST_TIME_MS 3000
#define LEG_KNEE_WAIT_TIME_MS    5000

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

	robot_mode = MODE_LEG;

}

// Wheel Mode 자세 설정
void Robot_Set_Wheel_Pose(void) {
	uint8_t ids[12] = { legs[1].roll, legs[1].pitch, legs[1].knee, legs[2].roll,
			legs[2].pitch, legs[2].knee, legs[3].roll, legs[3].pitch,
			legs[3].knee, legs[4].roll, legs[4].pitch, legs[4].knee };

	uint32_t wheel_pose[12] = { 2048, 2597, 950,   // leg1: FL
			1631, 2546, 2880,  // leg2: FR
			2050, 3626, 1755,  // leg3: RL
			2048, 3565, 1286   // leg4: RR
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

// LEG 전환 중 AX 바퀴 후진 보조
static void Robot_Wheel_Backward_Assist(uint32_t duration_ms) {
	const int16_t wheel_speed = 300;

	int16_t wheel_data[4] = { -wheel_speed,  // ID 4  : 후진 방향
			wheel_speed,   // ID 14 : 후진 방향
			-wheel_speed,  // ID 24 : 후진 방향
			wheel_speed    // ID 34 : 후진 방향
			};

	uint32_t start_tick = HAL_GetTick();

	// 후진 명령 1회 전송
	send_sync_write_1(DXL_1_MOVING_SPEED, 2, wheel_data);

	// 지정 시간 동안 후진 유지
	while (HAL_GetTick() - start_tick < duration_ms) {
		if (user_stop_flag) {
			Robot_Wheel_Stop();
			DXL_Emergency_All_Off();
			return;
		}

		HAL_Delay(20);
	}
}

// Leg Mode 자세 설정
void Robot_Set_Leg_Pose(void) {
	uint8_t pitch_ids[4] = { legs[1].pitch, legs[2].pitch, legs[3].pitch,
			legs[4].pitch };

	uint32_t pitch_pose[4] = { 3001,  // ID 2
			2163,  // ID 12
			4095,  // ID 22
			3059   // ID 32
			};

	uint8_t knee_ids[4] = { legs[1].knee, legs[2].knee, legs[3].knee,
			legs[4].knee };

	uint32_t knee_pose[4] = { 207,   // ID 3
			3560,  // ID 13
			946,   // ID 23
			2013   // ID 33
			};

	// Leg Mode에서 제어하는 4개 모터의 토크 활성화 값
	uint8_t torque_on[4] = { 1, 1, 1, 1 };

	// HIP(PITCH) 모터 속도/가속도 설정
	uint32_t pitch_profile_acc[4] = { 5, 5, 5, 5 };
	uint32_t pitch_profile_vel[4] = { 10, 10, 10, 10 };

	// KNEE 모터 속도/가속도 설정
	uint32_t knee_profile_acc[4] = { 5, 5, 5, 5 };
	uint32_t knee_profile_vel[4] = { 15, 15, 15, 15 };

	// HIP(PITCH) 모터 속도/가속도 설정
	dxl_sync_write_4byte(pitch_ids, pitch_profile_acc, 4, DXL_2_Profile_Acceleration);
	HAL_Delay(20);

	dxl_sync_write_4byte(pitch_ids, pitch_profile_vel, 4, DXL_2_Profile_Velocity);
	HAL_Delay(20);

	// HIP(PITCH) 토크 ON 후 목표 위치 전송
	dxl_sync_write_1byte(pitch_ids, torque_on, 4, DXL_2_Torque_Enable);
	HAL_Delay(100);

	dxl_sync_write_4byte(pitch_ids, pitch_pose, 4, DXL_2_Goal_Position);
	HAL_Delay(20);

	// HIP(PITCH) 이동 중 AX 바퀴 후진 보조
	Robot_Wheel_Backward_Assist(LEG_PITCH_ASSIST_TIME_MS);

	// HIP(PITCH) 전환 이후 바퀴 정지
	Robot_Wheel_Stop();
	HAL_Delay(100);

	// KNEE 모터 속도/가속도 설정
	dxl_sync_write_4byte(knee_ids, knee_profile_acc, 4, DXL_2_Profile_Acceleration);
	HAL_Delay(20);

	dxl_sync_write_4byte(knee_ids, knee_profile_vel, 4, DXL_2_Profile_Velocity);
	HAL_Delay(20);

	// KNEE 토크 ON 후 목표 위치 전송
	dxl_sync_write_1byte(knee_ids, torque_on, 4, DXL_2_Torque_Enable);
	HAL_Delay(100);

	dxl_sync_write_4byte(knee_ids, knee_pose, 4, DXL_2_Goal_Position);
	HAL_Delay(LEG_KNEE_WAIT_TIME_MS);
}
