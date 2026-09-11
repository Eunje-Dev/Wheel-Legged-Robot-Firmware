/*
 * jetson_comm.h
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */

#ifndef INC_JETSON_COMM_H_
#define INC_JETSON_COMM_H_

#include "main.h"

// Jetson 통신 초기화
void Jetson_Comm_Init(UART_HandleTypeDef *huart);

// Jetson UART 수신 오류 처리
uint8_t Jetson_Comm_HandleUartError(UART_HandleTypeDef *huart);

// 로컬 동작 전 대기 중인 원격 명령 폐기
void Jetson_Comm_DiscardPendingCommands(void);

// Jetson 명령 처리
void Jetson_Comm_Task(void);

#endif /* INC_JETSON_COMM_H_ */
