/*
 * robot_motion.h
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */

#ifndef INC_ROBOT_MOTION_H_
#define INC_ROBOT_MOTION_H_

// 로봇 현재 모드
typedef enum {
	MODE_NONE = 0, MODE_WHEEL, MODE_LEG, MODE_TO_WHEEL, MODE_TO_LEG
} RobotMode_t;

// 로봇 동작 초기화
void Robot_Motion_Init(void);

// AX 바퀴 정지
void Robot_Wheel_Stop(void);

// Wheel Mode 전진
void Robot_Wheel_Forward(void);

// Wheel Mode 후진
void Robot_Wheel_Backward(void);

// 좌회전 탱크턴
void Robot_Tank_Left(void);

// 우회전 탱크턴
void Robot_Tank_Right(void);

// Wheel Mode 자세 설정
void Robot_Set_Wheel_Pose(void);

// Leg Mode 자세 설정
void Robot_Set_Leg_Pose(void);

// Wheel Mode 전환
void Robot_Set_Wheel_Mode(void);

// Leg Mode 전환
void Robot_Set_Leg_Mode(void);

// 로봇 현재 모드 확인
RobotMode_t Robot_Get_Mode(void);

#endif /* INC_ROBOT_MOTION_H_ */
