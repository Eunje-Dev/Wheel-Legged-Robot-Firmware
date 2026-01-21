/*
 * imu_driver.h
 * Description: UART DMA & IDLE 인터럽트 기반 IMU 센서(EBIMU) 데이터 수신 드라이버
 * 수정사항: 스택 오버플로우 방지를 위한 데이터 수신과 파싱 로직 분리
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

// UART IDLE 인터럽트 콜백 함수 (ISR 컨텍스트에서 호출 - 가볍게 유지)
void IMU_IDLE_Callback(void);

// [신규] 수신된 데이터를 파싱하여 변환하는 함수 (while(1) 루프에서 호출)
void IMU_Process_Data(void);

// 최신 IMU 데이터 반환 (Getter)
IMU_Data_t IMU_Get_Data(void);

#endif /* INC_IMU_DRIVER_H_ */
