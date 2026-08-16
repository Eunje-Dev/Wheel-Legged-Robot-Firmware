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

// IMU 출력 형식별 데이터 개수
#define IMU_EULER_VALUE_COUNT 3U
#define IMU_QUATERNION_GYRO_VALUE_COUNT 7U
#define IMU_MAX_FRAME_VALUE_COUNT 7U

// 각속도 단위 변환
#define IMU_DEG_TO_RAD 0.01745329251994329577f

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

// IMU 문자열의 실수 데이터 파싱
static uint8_t IMU_Parse_Frame(const char *buf, float *values,
		uint8_t *value_count);

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
	(void) IMU_Start_DMA();
}

// UART 수신 오류 발생 시 DMA 수신 재시작
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart != imu_huart)
		return;

	receive_error = 1U;

	// USART 오류 플래그와 남아 있는 수신 데이터 제거
	__HAL_UART_CLEAR_FLAG(huart,
			UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF
					| UART_CLEAR_OREF);
	__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

	(void) IMU_Start_DMA();
}

// 수신 데이터 파싱
void IMU_Process_Data(void) {
	float values[IMU_MAX_FRAME_VALUE_COUNT] = { 0 };
	uint8_t value_count = 0U;

	if (!data_ready)
		return;

	// 문자열 끝 보장
	proc_buf[IMU_RX_BUF_SIZE - 1U] = '\0';

	// 파싱 실패 시 기존 데이터 유지
	if (!IMU_Parse_Frame((char*) proc_buf, values, &value_count)) {
		parse_error_count++;
		data_ready = 0;
		return;
	}

	if (value_count == IMU_EULER_VALUE_COUNT) {
		// Euler 출력 순서: roll, pitch, yaw
		imu_data.roll = values[0];
		imu_data.pitch = values[1];
		imu_data.yaw = values[2];

		imu_data.quaternion_valid = 0U;
		imu_data.gyro_valid = 0U;
	} else {
		// 센서 Quaternion 출력 순서: z, y, x, w
		imu_data.quaternion_z = values[0];
		imu_data.quaternion_y = values[1];
		imu_data.quaternion_x = values[2];
		imu_data.quaternion_w = values[3];

		// 센서 Gyro 출력 단위인 dps를 rad/s로 변환
		imu_data.gyro_x_rad_s = values[4] * IMU_DEG_TO_RAD;
		imu_data.gyro_y_rad_s = values[5] * IMU_DEG_TO_RAD;
		imu_data.gyro_z_rad_s = values[6] * IMU_DEG_TO_RAD;

		imu_data.quaternion_valid = 1U;
		imu_data.gyro_valid = 1U;
	}

	// 파싱 성공 시 공통 상태 갱신
	imu_data.timestamp_ms = frame_timestamp_ms;
	imu_data.sequence++;
	imu_data.valid = 1U;
	imu_data.updated = 1U;

	// 정상 처리 후 플래그 클리어
	data_ready = 0;
}

// 갱신 상태를 변경하지 않고 최신 데이터 반환
IMU_Data_t IMU_Get_Latest_Data(void) {
	IMU_Data_t data = imu_data;

	if (receive_error)
		data.valid = 0;

	if (data.valid
			&& (uint32_t) (HAL_GetTick() - data.timestamp_ms)
					> IMU_STALE_TIMEOUT_MS) {
		data.valid = 0;
	}

	return data;
}

// 최신 데이터 반환
IMU_Data_t IMU_Get_Data(void) {
	IMU_Data_t data = IMU_Get_Latest_Data();

	imu_data.updated = 0;

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
// values: 파싱된 실수 데이터 저장
// value_count: 파싱된 데이터 개수 저장
// return: 1(성공), 0(실패)
static uint8_t IMU_Parse_Frame(const char *buf, float *values,
		uint8_t *value_count) {
	const char *start;
	const char *ptr;
	char *endptr;
	uint8_t count = 0U;

	// '*' 기준으로 시작 위치 찾기
	start = strrchr(buf, '*');
	if (start == NULL)
		return 0U;

	ptr = start + 1;

	while (1) {
		if (count >= IMU_MAX_FRAME_VALUE_COUNT)
			return 0U;

		values[count] = strtof(ptr, &endptr);
		if (endptr == ptr)
			return 0U;

		count++;

		while (*endptr == ' ')
			endptr++;

		if (*endptr == ',') {
			ptr = endptr + 1;
			continue;
		}

		while (*endptr == ' ' || *endptr == '\r' || *endptr == '\n')
			endptr++;

		if (*endptr != '\0')
			return 0U;

		break;
	}

	if (count != IMU_EULER_VALUE_COUNT
			&& count != IMU_QUATERNION_GYRO_VALUE_COUNT) {
		return 0U;
	}

	*value_count = count;

	return 1U;
}
