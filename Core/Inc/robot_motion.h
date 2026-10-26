/*
 * robot_motion.h
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */

#ifndef INC_ROBOT_MOTION_H_
#define INC_ROBOT_MOTION_H_

#include <stdint.h>

// 로봇 현재 모드
typedef enum {
	MODE_NONE = 0, MODE_WHEEL, MODE_LEG, MODE_TO_WHEEL, MODE_TO_LEG,
	MODE_RAMP, MODE_WALK_DEMO
} RobotMode_t;

// 촬영용 보행 상태
typedef enum {
	WALK_DEMO_IDLE = 0,
	WALK_DEMO_ENTERING,
	WALK_DEMO_PLAYING,
	WALK_DEMO_FINISHED,
	WALK_DEMO_STOPPED
} RobotWalkDemoState_t;

typedef struct {
	uint8_t active;
	RobotWalkDemoState_t state;
	uint32_t sample_index;
	uint32_t elapsed_ms;
	uint32_t tx_fail_count;
} RobotWalkDemoStatus_t;

extern volatile RobotWalkDemoStatus_t robot_walk_demo_status;

// 로봇 동작 초기화
void Robot_Motion_Init(void);

// 전원 기준 잠금·주차 진입 시 로컬 동작 중단, 관절 토크는 유지
void Robot_Power_StopLocalMotion(void);

// AX 바퀴 정지
void Robot_Wheel_Stop(void);

// Wheel Mode 전진
void Robot_Wheel_Forward(void);

// Wheel Mode 후진
void Robot_Wheel_Backward(void);

// Wheel Mode 전진·후진 가속 처리
void Robot_Wheel_Drive_Task(void);

// 경사로 입력·계산 및 자세 전환 처리
void Robot_Ramp_Control_Task(void);

// 촬영용 3스텝 제어
void Robot_Walk_Demo_Task(void);

// 촬영용 3스텝 시작
uint8_t Robot_Start_Walk_Demo(void);

// 촬영용 3스텝 정지
void Robot_Stop_Walk_Demo(void);

// 좌회전 탱크턴
void Robot_Tank_Left(void);

// 우회전 탱크턴
void Robot_Tank_Right(void);

// Wheel Mode 자세 설정
void Robot_Set_Wheel_Pose(void);
void Robot_Get_Wheel_Pose_Targets(int32_t target[12]);

// Leg Mode 자세 설정
void Robot_Set_Leg_Pose(void);

// Wheel Mode 전환
void Robot_Set_Wheel_Mode(void);

// 경사로 보정 모드 진입
void Robot_Set_Ramp_Mode(void);

// Leg Mode 전환
void Robot_Set_Leg_Mode(void);

// 로봇 현재 모드 확인
RobotMode_t Robot_Get_Mode(void);

#endif /* INC_ROBOT_MOTION_H_ */
