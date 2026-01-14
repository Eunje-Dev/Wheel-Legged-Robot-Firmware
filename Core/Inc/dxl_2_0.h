/*
 * dxl_2_0.h
 * Description: 휠-레그 로봇 제어를 위한 다이나믹셀 프로토콜 1.0 & 2.0 통합 드라이버
 * (Dynamixel Protocol 1.0 & 2.0 Integrated Driver)
 */

#ifndef INC_DXL_2_0_H_
#define INC_DXL_2_0_H_
#include "main.h"

// 비상 정지 함수 선언
void DXL_Emergency_All_Off(void);

// UART3 통신 상태 플래그
extern volatile uint8_t uart3_busy;

#define leg_count 4

// Protocol 2.0 제어 주소 (MX-Series)
enum Dxl_2_0_Addr {
	DXL_2_Torque_Enable = 64, DXL_2_LED = 65, DXL_2_Goal_Position = 116,
};

// Protocol 1.0 제어 주소 (AX-Series)
enum Dxl_1_0_Addr {
	DXL_1_Torque_Enable = 24,
	DXL_1_LED = 25,
	DXL_1_Goal_Position = 30,
	DXL_1_MOVING_SPEED = 32,
};

// Protocol 1.0 패킷 구조체 (AX-Series)
typedef struct {
	uint8_t header[2];
	uint8_t id;
	uint8_t length;
	uint8_t inst;
	uint8_t start_addr;
	uint8_t data_len;
	uint8_t params[64];
	uint8_t checksum;
} dxl_sync_write_1_0;

// Protocol 2.0 패킷 구조체 (MX-Series)
typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;
	uint16_t length;
	uint8_t inst;
	uint16_t start_addr;
	uint16_t data_len;
	uint8_t params[64];
	uint16_t crc;
} dxl_sync_write_2_0;

// 다리별 모터 구성 (Leg Configuration)
typedef struct {
	uint8_t hip;   // Hip Joint (MX)
	uint8_t knee;  // Knee Joint (MX)
	uint8_t wheel; // Wheel Motor (AX)
} LegMotors;

// --- 함수 프로토타입 선언 ---

// 모터 설정 및 제어
void dxl_torque_set(uint8_t on_hip, uint8_t on_knee, uint8_t on_wheel);
uint16_t clc_speed_1(int16_t wheel_speed);
void ax_led_test(uint8_t on);
void dxl_write_1_0(uint8_t id, uint8_t addr, uint8_t data_len, uint16_t data);

// Sync Write 전송 (동기화 제어)
void send_sync_write_1(uint16_t addr, uint16_t data_len, uint16_t *wheel_data);
void send_sync_write_2(uint16_t addr, uint16_t data_len, uint32_t *hip_data,
		uint32_t *knee_data);

// 유틸리티 (CRC/Checksum/Packet 생성)
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
		unsigned short data_blk_size);
uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length);
dxl_sync_write_1_0 creat_sync_packet_1_0(void);
dxl_sync_write_2_0 creat_sync_packet_2_0(void);

#endif /* INC_DXL_2_0_H_ */
