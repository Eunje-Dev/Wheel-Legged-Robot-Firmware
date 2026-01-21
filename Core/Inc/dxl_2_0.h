// 휠-레그 로봇 12축 모터 통합 제어 드라이버
// MX-106(고관절), MX-64(무릎), AX-12(바퀴) 혼합 제어 지원

#ifndef INC_DXL_2_0_H_
#define INC_DXL_2_0_H_

#include "main.h"

// 로봇 시스템 관련 설정값
#define LEG_COUNT 4      // 다리 개수
#define MX_DATA_LEN 4    // MX 시리즈 목표 위치 데이터 길이 (4바이트)
#define AX_DATA_LEN 2    // AX 시리즈 목표 속도 데이터 길이 (2바이트)

// 다이나믹셀 프로토콜 2.0 주소 (MX-106, MX-64 관절용)
enum Dxl_2_0_Addr {
    DXL_2_Torque_Enable = 64,   // 토크 온/오프 (1: 사용, 0: 해제)
    DXL_2_LED           = 65,   // LED 제어
    DXL_2_Goal_Position = 116,  // 목표 위치 제어 (0 ~ 4095)
};

// 다이나믹셀 프로토콜 1.0 주소 (AX-12 바퀴용)
enum Dxl_1_0_Addr {
    DXL_1_Torque_Enable = 24,   // 토크 온/오프
    DXL_1_LED           = 25,   // LED 제어
    DXL_1_Goal_Position = 30,   // 위치 제어 시 사용
    DXL_1_MOVING_SPEED  = 32,   // 바퀴 모드 속도 제어 (0~1023: CCW, 1024~2047: CW)
};

// 다리별 모터 구성을 관리하는 구조체
typedef struct {
    uint8_t hip;   // 고관절 모터 ID
    uint8_t knee;  // 무릎관절 모터 ID
    uint8_t wheel; // 바퀴 모터 ID
} LegMotors;

// 외부에서 참조할 전역 변수
extern LegMotors legs[5];

// 모터 제어 및 통신 관련 함수 선언
void dxl_torque_set(uint8_t on_hip, uint8_t on_knee, uint8_t on_wheel); // 전체 모터 토크 제어
void DXL_Emergency_All_Off(void);                                      // 비상 정지 (모든 토크 해제)
void send_sync_write_1_wheel(int16_t *wheel_speeds);                  // 바퀴 4개 동시 속도 제어
void send_sync_write_2_joints(uint32_t *hip_pos, uint32_t *knee_pos); // 관절 8개 동시 위치 제어
void dxl_write_1_0(uint8_t id, uint8_t addr, uint8_t data_len, uint16_t data); // 개별 AX-12 제어
uint16_t clc_speed_1(int16_t wheel_speed);                            // 바퀴 속도 값 변환 함수

// 통신 프로토콜 무결성 검사 함수
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length);

#endif
