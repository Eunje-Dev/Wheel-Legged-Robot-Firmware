/*
 * jetson_comm.c
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */
#include "jetson_comm.h"
#include "robot_motion.h"
#include "imu_driver.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *jetson_huart;

static uint8_t rx_byte;
static char rx_line[64];
static uint8_t rx_index = 0;
static volatile uint8_t line_ready = 0;

// Jetson 문자열 송신
static void Jetson_Send_Text(const char *text) {
	(void) HAL_UART_Transmit(jetson_huart, (uint8_t*) text,
			(uint16_t) strlen(text), 20);
}

// IMU 각도를 100배 정수로 변환
static int32_t Jetson_Angle_To_X100(float angle) {
	if (angle >= 0.0f)
		return (int32_t) (angle * 100.0f + 0.5f);

	return (int32_t) (angle * 100.0f - 0.5f);
}

// 최신 IMU 데이터 송신
static void Jetson_Send_IMU(void) {
	IMU_Data_t data = IMU_Get_Latest_Data();
	char tx_line[160];

	int32_t roll_x100 = Jetson_Angle_To_X100(data.roll);
	int32_t pitch_x100 = Jetson_Angle_To_X100(data.pitch);
	int32_t yaw_x100 = Jetson_Angle_To_X100(data.yaw);

	int length = snprintf(tx_line, sizeof(tx_line),
			"IMU:ROLL_X100=%ld,PITCH_X100=%ld,YAW_X100=%ld,"
					"VALID=%u,SEQ=%lu,TIME=%lu\r\n", (long) roll_x100,
			(long) pitch_x100, (long) yaw_x100, (unsigned int) data.valid,
			(unsigned long) data.sequence, (unsigned long) data.timestamp_ms);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		(void) HAL_UART_Transmit(jetson_huart, (uint8_t*) tx_line,
				(uint16_t) length, 20);
	}
}

// Jetson 통신 초기화
void Jetson_Comm_Init(UART_HandleTypeDef *huart) {
	jetson_huart = huart;
	rx_index = 0;
	line_ready = 0;

	HAL_UART_Receive_IT(jetson_huart, &rx_byte, 1);
}

// Jetson UART 수신 완료 콜백
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		RobotMode_t current_mode = Robot_Get_Mode();

		// 모드 전환 중 수신된 Jetson 명령 폐기
		if (current_mode == MODE_TO_WHEEL || current_mode == MODE_TO_LEG) {
			rx_index = 0;
			line_ready = 0;

			HAL_UART_Receive_IT(jetson_huart, &rx_byte, 1);
			return;
		}

		if (!line_ready) {
			if (rx_byte == '\n' || rx_byte == '\r') {
				rx_line[rx_index] = '\0';

				if (rx_index > 0) {
					line_ready = 1;
				}

				rx_index = 0;
			} else {
				if (rx_index < sizeof(rx_line) - 1) {
					rx_line[rx_index++] = rx_byte;
				} else {
					rx_index = 0;
				}
			}
		}

		HAL_UART_Receive_IT(jetson_huart, &rx_byte, 1);
	}
}

// Jetson 명령 처리
void Jetson_Comm_Task(void) {
	char command[sizeof(rx_line)];

	if (!line_ready) {
		return;
	}

	// 수신 명령을 별도 공간에 복사
	strncpy(command, rx_line, sizeof(command) - 1);
	command[sizeof(command) - 1] = '\0';
	line_ready = 0;

	if (strcmp(command, "PING") == 0) {
		Jetson_Send_Text("OK\r\n");
	} else if (strcmp(command, "GET_IMU") == 0) {
		Jetson_Send_IMU();
	} else if (strcmp(command, "STOP") == 0) {
		Robot_Wheel_Stop();
	} else if (strcmp(command, "FWD") == 0) {
		Robot_Wheel_Forward();
	} else if (strcmp(command, "BACK") == 0) {
		Robot_Wheel_Backward();
	} else if (strcmp(command, "TL") == 0) {
		Robot_Tank_Left();
	} else if (strcmp(command, "TR") == 0) {
		Robot_Tank_Right();
	} else if (strcmp(command, "WHEEL") == 0) {
		Robot_Set_Wheel_Mode();
	} else if (strcmp(command, "LEG") == 0) {
		Robot_Set_Leg_Mode();
	}
}

