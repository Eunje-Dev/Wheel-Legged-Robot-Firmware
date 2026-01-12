/*
 * dxl_2_0.h
 *
 * Created on: 2026. 1. 12.
 * Author: User
 * Description: 휠-레그 로봇 구동을 위한 다이나믹셀 1.0 & 2.0 통합 드라이버
 */

#ifndef INC_DXL_2_0_H_
#define INC_DXL_2_0_H_

#include "main.h"

// UART 송신 상태 플래그
extern volatile uint8_t uart_tx_busy;

#define leg_count 4

// 모터 개수 정의
#define all_wheel_motor 4          // Protocol 1.0 (AX-12)
#define all_hip_knee_motor 8       // Protocol 2.0 (MX-106, MX-64)

// --- 프로토콜 상수 정의 ---
#define DXL_STD_PARAMS 64
#define DXL_SYNC_PARAMS 64
#define DXL_BULK_PARAMS 128

enum DxlInst{
	DXL_INST_PING = 0x01,
	DXL_INST_READ = 0x02,
	DXL_INST_WRITE = 0x03,
	DXL_INST_REG_WRITE = 0x04,
 	DXL_INST_ACTION = 0x05,
	DXL_INST_FACTORY_RESET = 0x06,
	DXL_INST_REBOOT = 0x08,
	DXL_INST_CLEAR = 0x10,
	DXL_INST_STATUS = 0x55,
	DXL_INST_SYNC_READ = 0x82,
	DXL_INST_SYNC_WRITE = 0x83,
	DXL_INST_BULK_READ = 0x92,
	DXL_INST_BULK_WRITE = 0x93,
};

// MX-64, MX-106 (Protocol 2.0) 컨트롤 테이블 주소
enum Dxl_2_0_Addr{
	DXL_2_Torque_Enable = 64,
	DXL_2_LED = 65,
	DXL_2_Goal_PWM = 100,
	DXL_2_Goal_Velocity = 112,
	DXL_2_Goal_Position = 116,
	DXL_2_Present_Current = 126,
	DXL_2_Present_Position = 132,
	DXL_2_Present_Temperature = 146,
};

// AX-12A (Protocol 1.0) 컨트롤 테이블 주소
enum Dxl_1_0_Addr{
	DXL_1_Torque_Enable = 24,
	DXL_1_LED = 25,
	DXL_1_MOVING_SPEED = 32,
	DXL_1_Goal_Torque_Limit = 34,
	DXL_1_Goal_Position = 30,
	DXL_1_Present_Load = 40,
	DXL_1_Present_Position = 36,
	DXL_1_Present_Temperature = 43,
};

// 로봇 전체 제어 데이터 구조체
typedef struct {
	uint32_t hip_data[4];   // MX-106 (Hip)
	uint32_t knee_data[4];  // MX-64 (Knee)
	uint16_t wheel_data[4]; // AX-12 (Wheel)
} data_set_all;

// --- 통신 패킷 구조체 ---
typedef struct {
	uint8_t header[3]; uint8_t reserved; uint8_t id; uint16_t length; uint8_t inst;
	uint8_t params[DXL_STD_PARAMS]; uint16_t crc;
} dxl_packet_2_0;

typedef struct {
	uint8_t header[2]; uint8_t id; uint8_t length; uint8_t inst;
	uint8_t params[DXL_STD_PARAMS]; uint8_t checksum;
} dxl_packet_1_0;

typedef struct {
	uint8_t header[2]; uint8_t id; uint8_t length; uint8_t inst;
	uint8_t start_addr; uint8_t data_len; uint8_t params[DXL_SYNC_PARAMS]; uint8_t checksum;
} dxl_sync_write_1_0;

typedef struct {
	uint8_t header[3]; uint8_t reserved; uint8_t id; uint16_t length; uint8_t inst;
	uint16_t start_addr; uint16_t data_len; uint8_t params[DXL_SYNC_PARAMS]; uint16_t crc;
} dxl_sync_write_2_0;

// 다리별 모터 ID 매핑 구조체
typedef struct {
	uint8_t hip;    // Protocol 2.0
	uint8_t knee;   // Protocol 2.0
	uint8_t wheel;  // Protocol 1.0
} LegMotors;


// --- 함수 선언 ---
void dxl_torque_set(uint8_t on_hip, uint8_t on_knee, uint8_t on_wheel);
uint16_t clc_speed_1(int16_t wheel_speed);

// Protocol 1.0 제어 함수 (Wheel)
void send_sync_write_1(uint16_t addr, uint16_t data_len, uint16_t *wheel_data);

// Protocol 2.0 제어 함수 (Hip, Knee)
void send_sync_write_2(uint16_t addr, uint16_t data_len, uint32_t *hip_data, uint32_t *knee_data);

// 유틸리티 함수
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length);
dxl_sync_write_1_0 creat_sync_packet_1_0(void);
dxl_sync_write_2_0 creat_sync_packet_2_0(void);
uint16_t serialize_sync_write_2_0(dxl_sync_write_2_0 *packet, uint8_t *id_array, uint8_t id_count, uint32_t *data_array, uint8_t *buffer, uint16_t buffer_size);
uint16_t serialize_sync_write_1_0(dxl_sync_write_1_0 *packet, uint8_t *id_array, uint8_t id_count, uint16_t *data_array, uint8_t *buffer, uint16_t buffer_size);

#endif /* INC_DXL_2_0_H_ */
