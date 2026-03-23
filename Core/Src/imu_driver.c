/*
 * imu_driver.c
 *
 *  Created on: 2026. 3. 23.
 *      Author: 이은제
 */
#include <stdio.h>
#include "imu_driver.h"
#include <string.h>
#include <stdlib.h>

// UART 핸들 저장
static UART_HandleTypeDef *imu_huart;

// DMA 수신 버퍼
static uint8_t rx_buf[128];
static uint8_t proc_buf[128];

// 현재 IMU 데이터
static IMU_Data_t imu_data = { 0 };

// 수신 완료 플래그
static volatile uint8_t data_ready = 0;

// IMU 문자열 → roll/pitch/yaw 파싱
static uint8_t IMU_Parse_Frame(const char *buf, float *roll, float *pitch,
		float *yaw);

// IMU 초기화 (UART 핸들 등록)
void IMU_Init(UART_HandleTypeDef *huart) {
	imu_huart = huart;
}

// DMA 수신 시작
void IMU_Start_DMA(void) {
	HAL_UART_Receive_DMA(imu_huart, rx_buf, sizeof(rx_buf));
	__HAL_UART_ENABLE_IT(imu_huart, UART_IT_IDLE);
}

// IDLE 인터럽트 콜백 (프레임 수신 완료)
void IMU_IDLE_Callback(void) {
	// DMA 정지
	HAL_UART_DMAStop(imu_huart);

	// 수신 데이터 복사
	memcpy(proc_buf, rx_buf, sizeof(rx_buf));

	// 플래그 설정
	data_ready = 1;

	// DMA 재시작
	HAL_UART_Receive_DMA(imu_huart, rx_buf, sizeof(rx_buf));
}

// 수신 데이터 파싱
void IMU_Process_Data(void) {
	if (!data_ready)
		return;

	// 문자열 끝 보장
	proc_buf[127] = '\0';

	float roll, pitch, yaw;

	// 파싱 실패 시 기존 데이터 유지
	if (!IMU_Parse_Frame((char*) proc_buf, &roll, &pitch, &yaw))
		return;

	// 파싱 성공 시만 데이터 갱신
	imu_data.roll = roll;
	imu_data.pitch = pitch;
	imu_data.yaw = yaw;
	imu_data.updated = 1;

	// 정상 처리 후 플래그 클리어
	data_ready = 0;
}

// 최신 데이터 반환
IMU_Data_t IMU_Get_Data(void) {
	imu_data.updated = 0;
	return imu_data;
}

// IMU 프레임 파싱 함수
// buf: 수신된 문자열
// roll, pitch, yaw: 파싱 결과 저장
// return: 1(성공), 0(실패)
static uint8_t IMU_Parse_Frame(const char *buf, float *roll, float *pitch,
		float *yaw) {
	// '*' 기준으로 시작 위치 찾기
	const char *start = strrchr(buf, '*');
	if (start == NULL)
		return 0;

	const char *ptr = start + 1;
	char *endptr;

	// roll 파싱
	*roll = strtof(ptr, &endptr);
	if (*endptr != ',')
		return 0;

	// pitch 파싱
	ptr = endptr + 1;
	*pitch = strtof(ptr, &endptr);
	if (*endptr != ',')
		return 0;

	// yaw 파싱
	ptr = endptr + 1;
	*yaw = strtof(ptr, &endptr);

	return 1;
}
