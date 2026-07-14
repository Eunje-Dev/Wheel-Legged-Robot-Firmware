/*
 * jetson_comm.c
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */
#include "jetson_comm.h"
#include "robot_motion.h"
#include <string.h>

static UART_HandleTypeDef *jetson_huart;

static uint8_t rx_byte;
static char rx_line[64];
static uint8_t rx_index = 0;
static volatile uint8_t line_ready = 0;

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
	if (!line_ready) {
		return;
	}

	line_ready = 0;

	if (strcmp(rx_line, "STOP") == 0) {
		Robot_Wheel_Stop();
	} else if (strcmp(rx_line, "FWD") == 0) {
		Robot_Wheel_Forward();
	} else if (strcmp(rx_line, "BACK") == 0) {
		Robot_Wheel_Backward();
	} else if (strcmp(rx_line, "TL") == 0) {
		Robot_Tank_Left();
	} else if (strcmp(rx_line, "TR") == 0) {
		Robot_Tank_Right();
	} else if (strcmp(rx_line, "WHEEL") == 0) {
		Robot_Set_Wheel_Mode();
	} else if (strcmp(rx_line, "LEG") == 0) {
		Robot_Set_Leg_Mode();
	}
}

