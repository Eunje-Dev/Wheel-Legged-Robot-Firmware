/*
 * imu_driver.h
 *
 *  Created on: 2026. 3. 23.
 *      Author: 이은제
 */

#ifndef INC_IMU_DRIVER_H_
#define INC_IMU_DRIVER_H_

#include "main.h"

// IMU 자세와 각속도 데이터 구조체
typedef struct {
	float roll;    // X축 회전 (좌우 기울기)
	float pitch;   // Y축 회전 (앞뒤 기울기)
	float yaw;     // Z축 회전 (방향)

	float quaternion_x; // Quaternion X 성분
	float quaternion_y; // Quaternion Y 성분
	float quaternion_z; // Quaternion Z 성분
	float quaternion_w; // Quaternion W 성분

	float gyro_x_rad_s; // 센서 X축 각속도 (rad/s)
	float gyro_y_rad_s; // 센서 Y축 각속도 (rad/s)
	float gyro_z_rad_s; // 센서 Z축 각속도 (rad/s)

	uint32_t timestamp_ms; // 마지막 정상 데이터 수신 시각
	uint32_t sequence; // 정상 데이터 수신 순번
	uint8_t valid; // 현재 데이터 유효 여부
	uint8_t updated; // 새로운 데이터 갱신 여부
	uint8_t quaternion_valid; // Quaternion 데이터 유효 여부
	uint8_t gyro_valid; // 각속도 데이터 유효 여부
} IMU_Data_t;

// IMU UART 핸들 등록 및 내부 초기화
void IMU_Init(UART_HandleTypeDef *huart);

// UART DMA 수신 시작
HAL_StatusTypeDef IMU_Start_DMA(void);

// 수신된 데이터를 파싱하여 roll/pitch/yaw 업데이트
void IMU_Process_Data(void);

// 최신 IMU 데이터 반환
IMU_Data_t IMU_Get_Data(void);

// 갱신 상태를 변경하지 않고 최신 IMU 데이터 반환
IMU_Data_t IMU_Get_Latest_Data(void);

// IMU 데이터 오류 횟수 반환
uint32_t IMU_Get_Parse_Error_Count(void);
uint32_t IMU_Get_Dropped_Frame_Count(void);

#endif /* INC_IMU_DRIVER_H_ */
