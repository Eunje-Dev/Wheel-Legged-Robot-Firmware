/*
 * imu_driver.h
 * Description: UART DMA & IDLE 인터럽트 기반 IMU 센서(EBIMU) 데이터 수신 드라이버
 */

#ifndef INC_IMU_DRIVER_H_
#define INC_IMU_DRIVER_H_

#include "main.h"

// IMU 3축 오일러 각(Euler Angles) 데이터 구조체 정의
typedef struct {
	float roll;
	float pitch;
	float yaw;
} IMU_Data_t;

// --- 함수 프로토타입 선언 ---

// IMU 초기화: UART 및 DMA 수신 설정
void IMU_Init(UART_HandleTypeDef *huart);

// UART IDLE 인터럽트 콜백 함수 (ISR 컨텍스트에서 호출 필요)
// Note: stm32h7xx_it.c의 UART IRQ Handler 내부에서 호출되어야 함
void IMU_IDLE_Callback(void);

// 최신 IMU 데이터 반환 (Getter)
IMU_Data_t IMU_Get_Data(void);

#endif /* INC_IMU_DRIVER_H_ */
