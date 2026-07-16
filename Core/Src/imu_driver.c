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

// IMU DMA 수신 버퍼 크기
#define IMU_RX_BUF_SIZE 128U

// IMU 데이터 유효 시간
#define IMU_STALE_TIMEOUT_MS 200U

// UART 핸들 저장
static UART_HandleTypeDef *imu_huart;

// DMA 수신 버퍼
static uint8_t rx_buf[IMU_RX_BUF_SIZE];
static uint8_t proc_buf[IMU_RX_BUF_SIZE];

// 현재 IMU 데이터
static IMU_Data_t imu_data = { 0 };

// 수신 완료 플래그
static volatile uint8_t data_ready = 0;

// IMU 수신 상태
static volatile uint8_t receive_error = 0;
static volatile uint32_t frame_timestamp_ms = 0;
static volatile uint32_t dropped_frame_count = 0;
static uint32_t parse_error_count = 0;

// IMU 문자열 → roll/pitch/yaw 파싱
static uint8_t IMU_Parse_Frame(const char *buf, float *roll, float *pitch,
		float *yaw);

// IMU 초기화 (UART 핸들 등록)
void IMU_Init(UART_HandleTypeDef *huart) {
	imu_huart = huart;
}

// DMA 수신 시작
HAL_StatusTypeDef IMU_Start_DMA(void) {
	HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(imu_huart, rx_buf,
			sizeof(rx_buf));

	if (status == HAL_OK) {
		__HAL_DMA_DISABLE_IT(imu_huart->hdmarx, DMA_IT_HT);
		receive_error = 0;
	} else {
		receive_error = 1;
	}

	return status;
}

// UART IDLE 수신 완료 콜백
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	if (huart != imu_huart)
		return;

	if (size > 0) {
		uint16_t copy_size = size;

		if (copy_size >= sizeof(proc_buf))
			copy_size = sizeof(proc_buf) - 1;

		if (!data_ready) {
			// 수신 데이터 복사
			memcpy(proc_buf, rx_buf, copy_size);
			proc_buf[copy_size] = '\0';

			// 플래그 설정
			frame_timestamp_ms = HAL_GetTick();
			data_ready = 1;
		} else {
			dropped_frame_count++;
		}
	}

	// DMA 재시작
	IMU_Start_DMA();
}

// 수신 데이터 파싱
void IMU_Process_Data(void) {
	if (!data_ready)
		return;

	// 문자열 끝 보장
	proc_buf[127] = '\0';

	float roll, pitch, yaw;

	// 파싱 실패 시 기존 데이터 유지
	if (!IMU_Parse_Frame((char*) proc_buf, &roll, &pitch, &yaw)) {
		parse_error_count++;
		data_ready = 0;
		return;
	}

	// 파싱 성공 시만 데이터 갱신
	imu_data.roll = roll;
	imu_data.pitch = pitch;
	imu_data.yaw = yaw;
	imu_data.timestamp_ms = frame_timestamp_ms;
	imu_data.sequence++;
	imu_data.valid = 1;
	imu_data.updated = 1;

	// 정상 처리 후 플래그 클리어
	data_ready = 0;
}

// 최신 데이터 반환
IMU_Data_t IMU_Get_Data(void) {
	IMU_Data_t data = imu_data;

	imu_data.updated = 0;

	if (receive_error)
		data.valid = 0;

	if (data.valid
			&& (uint32_t) (HAL_GetTick() - data.timestamp_ms)
					> IMU_STALE_TIMEOUT_MS) {
		data.valid = 0;
	}

	return data;
}

// IMU 데이터 오류 횟수 반환
uint32_t IMU_Get_Parse_Error_Count(void) {
	return parse_error_count;
}

uint32_t IMU_Get_Dropped_Frame_Count(void) {
	return dropped_frame_count;
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
	if (endptr == ptr || *endptr != ',')
		return 0;

	// pitch 파싱
	ptr = endptr + 1;
	*pitch = strtof(ptr, &endptr);
	if (endptr == ptr || *endptr != ',')
		return 0;

	// yaw 파싱
	ptr = endptr + 1;
	*yaw = strtof(ptr, &endptr);
	if (endptr == ptr)
		return 0;

	while (*endptr == ' ' || *endptr == '\r' || *endptr == '\n')
		endptr++;

	if (*endptr != '\0')
		return 0;

	return 1;
}
