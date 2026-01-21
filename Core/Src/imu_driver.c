/*
 * imu_driver.c
 * Description: IMU 센서 데이터 파싱 및 링버퍼 처리 구현체 (수정본)
 * Note: 인터럽트 부하를 줄이기 위해 파싱 로직을 메인 루프로 이동시킴
 */
#include "imu_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define IMU_BUF_SIZE 128

UART_HandleTypeDef *imu_uart;
uint8_t imu_rx_buf[IMU_BUF_SIZE]; // DMA가 직접 채우는 수신 버퍼

// 파싱을 위해 데이터를 잠시 복사해둘 버퍼와 플래그
char imu_parsing_buf[IMU_BUF_SIZE];
volatile uint8_t imu_data_ready = 0; // 1이면 새로운 데이터가 왔다는 뜻

IMU_Data_t current_imu_data = { 0, };

// IMU 초기화 및 DMA Circular 수신 모드 시작
void IMU_Init(UART_HandleTypeDef *huart) {
	imu_uart = huart;

	// UART IDLE 라인 감지 인터럽트 활성화
	__HAL_UART_ENABLE_IT(imu_uart, UART_IT_IDLE);

	// DMA Circular 모드를 통한 연속 데이터 수신 시작
	HAL_UART_Receive_DMA(imu_uart, imu_rx_buf, IMU_BUF_SIZE);
}

// [인터럽트] IDLE 감지 시 호출됨 - 최대한 짧고 빠르게 끝내야 함
void IMU_IDLE_Callback(void) {
	// 1. UART IDLE 인터럽트 플래그 클리어
	__HAL_UART_CLEAR_IDLEFLAG(imu_uart);

	// 2. DMA 버퍼 데이터를 파싱용 버퍼로 '복사'만 수행 (계산 X)
	// 이렇게 하면 인터럽트 처리가 순식간에 끝나서 스택이 터지지 않음
	memcpy(imu_parsing_buf, (char*) imu_rx_buf, IMU_BUF_SIZE);

	// 3. 메인 루프에게 "데이터 도착했으니 처리해라"라고 깃발 들기
	imu_data_ready = 1;
}

// [메인 루프용] 실제 데이터 파싱 및 변환 수행 (sscanf 사용)
void IMU_Process_Data(void) {
    if (imu_data_ready == 1) {
        imu_data_ready = 0; // 플래그 내림

        // 1. 가장 최근 데이터 패킷 찾기 ('*' 문자로 시작)
        char *start_ptr = strrchr(imu_parsing_buf, '*');

        if (start_ptr != NULL) {
            // 예시 데이터: "* -10.5, 5.3, 90.1"
            start_ptr++; // '*' 다음 문자로 이동

            // 2. 콤마(,)를 기준으로 문자열 자르기
            char *token;
            char *context = NULL; // strtok_r용 문맥 포인터

            // Roll 파싱
            token = strtok_r(start_ptr, ",", &context);
            if (token != NULL) current_imu_data.roll = strtof(token, NULL);

            // Pitch 파싱 (우리가 필요한 값)
            token = strtok_r(NULL, ",", &context);
            if (token != NULL) current_imu_data.pitch = strtof(token, NULL);

            // Yaw 파싱
            token = strtok_r(NULL, ",", &context);
            if (token != NULL) current_imu_data.yaw = strtof(token, NULL);
        }
    }
}

// 외부에서 최신 IMU 데이터를 조회하기 위한 인터페이스
IMU_Data_t IMU_Get_Data(void) {
	return current_imu_data;
}
