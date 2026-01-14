/*
 * imu_driver.c
 * Description: IMU 센서 데이터 파싱 및 링버퍼 처리 구현체
 * Dependency: MCU Settings > "Use float with scanf from newlib-nano" 활성화 필수
 */
#include "imu_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // atof 사용 시 필요 (옵션)

#define IMU_BUF_SIZE 128

UART_HandleTypeDef *imu_uart;
uint8_t imu_rx_buf[IMU_BUF_SIZE]; // DMA 수신 버퍼
IMU_Data_t current_imu_data = { 0, };

// IMU 초기화 및 DMA Circular 수신 모드 시작
void IMU_Init(UART_HandleTypeDef *huart) {
	imu_uart = huart;

	// UART IDLE 라인 감지 인터럽트 활성화
	__HAL_UART_ENABLE_IT(imu_uart, UART_IT_IDLE);

	// DMA Circular 모드를 통한 연속 데이터 수신 시작
	HAL_UART_Receive_DMA(imu_uart, imu_rx_buf, IMU_BUF_SIZE);
}

// IDLE 인터럽트 발생 시 호출되는 데이터 처리 함수
void IMU_IDLE_Callback(void) {
	// 1. UART IDLE 인터럽트 플래그 클리어 (중복 인터럽트 방지)
	__HAL_UART_CLEAR_IDLEFLAG(imu_uart);

	// 2. 데이터 파싱을 위한 임시 버퍼 초기화
	char temp_buf[IMU_BUF_SIZE] = { 0, };

	// 3. DMA 버퍼 데이터를 임시 버퍼로 복사 (원자성 확보 및 파싱 용이성 증대)
	// memcpy를 사용하여 현재 수신된 데이터 스냅샷을 획득
	memcpy(temp_buf, (char*) imu_rx_buf, IMU_BUF_SIZE);

	// 4. 최신 데이터 패킷 탐색 (Reverse Search)
	// 프로토콜 포맷: *Roll,Pitch,Yaw (예: *10.5,-5.2,120.0)
	// 버퍼의 끝에서부터 가장 최근의 시작 문자('*')를 검색
	char *last_star = strrchr(temp_buf, '*');

	if (last_star != NULL) {
		float r, p, y;

		// 5. 문자열 파싱 및 실수형 변환
		// sscanf 사용 시 "Use float with scanf" 설정이 활성화되어 있어야 함
		if (sscanf(last_star, "*%f,%f,%f", &r, &p, &y) == 3) {
			// 6. 전역 데이터 구조체 갱신 (Critical Section 보호 고려 가능)
			current_imu_data.roll = r;
			current_imu_data.pitch = p;
			current_imu_data.yaw = y;
		}
	}
}

// 외부에서 최신 IMU 데이터를 조회하기 위한 인터페이스
IMU_Data_t IMU_Get_Data(void) {
	return current_imu_data;
}
