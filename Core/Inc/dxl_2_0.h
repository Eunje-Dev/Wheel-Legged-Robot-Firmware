/*
 * dxl_2_0.h
 * 설명: 휠-레그 로봇 구동을 위한 다이나믹셀 1.0(AX) & 2.0(MX) 통합 헤더
 * 수정: 프로토콜 버전별 제어 주소(Control Table) 및 데이터 구조체 명세 정리
 */

#ifndef INC_DXL_2_0_H_
#define INC_DXL_2_0_H_

#include "main.h"

// UART 송신 상태 플래그 (비동기 처리용)
extern volatile uint8_t uart_tx_busy;

#define leg_count 4		// 로봇 다리 개수 (4족 로봇)
#define DXL_SYNC_PARAMS 64

// 로봇 다리 번호
enum RobotLegIndex {
	LEG_FRONT_LEFT = 1,
	LEG_FRONT_RIGHT = 2,
	LEG_REAR_LEFT = 3,
	LEG_REAR_RIGHT = 4,
};

// 다이나믹셀 공통 명령어 (Instruction) 정의
enum DxlInst {
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

// [Protocol 2.0] MX-106, MX-64 컨트롤 테이블 (다리 관절용)
// 통신: UART6 (2Mbps)
enum Dxl_2_0_Addr {
	DXL_2_Torque_Enable = 64,      // 토크 ON/OFF (1: ON, 0: OFF)
	DXL_2_LED = 65,                // LED 제어
	DXL_2_Status_Return_Level = 68, // 응답레벨
	DXL_2_Hardware_Error_Status = 70, // 하드웨어 에러 상태
	DXL_2_Goal_PWM = 100,          // 목표 PWM 값
	DXL_2_Profile_Acceleration = 108, // 프로파일 가속도
	DXL_2_Profile_Velocity = 112,  // 프로파일 속도
	DXL_2_Goal_Position = 116,     // 목표 위치 (Position Control Mode)
	DXL_2_Present_Current = 126,   // 현재 소모 전류
	DXL_2_Present_Velocity = 128,  // 현재 속도
	DXL_2_Present_Position = 132,  // 현재 위치 (Encoder 값)
	DXL_2_Present_Temperature = 146, // 현재 내부 온도
};

// [Protocol 1.0] AX-12+ 컨트롤 테이블 (바퀴 구동용)
// 통신: UART1 (1Mbps)
enum Dxl_1_0_Addr {
	DXL_1_CW_Angle_Limit = 6,      // CW 각도 제한
	DXL_1_CCW_Angle_Limit = 8,     // CCW 각도 제한
	DXL_1_Torque_Enable = 24,      // 토크 ON/OFF
	DXL_1_LED = 25,                // LED 제어
	DXL_1_MOVING_SPEED = 32,       // 목표 속도 (Wheel Mode 시 속도 제어)
	DXL_1_Goal_Torque_Limit = 34,  // 최대 토크 제한
	DXL_1_Goal_Position = 30,      // 목표 위치
	DXL_1_Present_Load = 40,       // 현재 부하율
	DXL_1_Present_Position = 36,   // 현재 위치
	DXL_1_Present_Temperature = 43,   // 현재 온도
};

// 로봇 다리 관절 모터 ID 매핑 구조체 (Protocol 2.0, MX 계열)
// roll : 다리 좌우 기울기 제어 (HIP(ROLL) 축)
// pitch  : 다리 전후 스윙 제어 (HIP(PITCH) 관절)
// knee : 링크 길이 변화 및 보행 동작 제어 (Knee 관절)
typedef struct {
	uint8_t roll;
	uint8_t pitch;
	uint8_t knee;
} LegMotors;

// 바퀴 구동 모터 ID 매핑 구조체 (Protocol 1.0, AX 계열)
// wheel : 이동용 바퀴 모터
typedef struct {
	uint8_t wheel;
} WheelMotors;

// 다리 관절 모터 ID 매핑 테이블 (Protocol 2.0, MX 계열)
extern LegMotors legs[5];

// 바퀴 모터 ID 매핑 테이블 (Protocol 1.0, AX 계열)
extern WheelMotors wheels[5];

// Protocol 1.0 Sync Write 패킷 구조체 (바퀴용)
typedef struct {
	uint8_t header[2];
	uint8_t id;
	uint8_t length;
	uint8_t inst;
	uint8_t start_addr;
	uint8_t data_len;
	uint8_t params[DXL_SYNC_PARAMS];
	uint8_t checksum;
} dxl_sync_write_1_0;

// Protocol 2.0 Sync Write 패킷 구조체 (다리용)
typedef struct {
	uint8_t header[3];
	uint8_t reserved;
	uint8_t id;
	uint16_t length;
	uint8_t inst;
	uint16_t start_addr;
	uint16_t data_len;
	uint8_t params[DXL_SYNC_PARAMS];
	uint16_t crc;
} dxl_sync_write_2_0;

// --- 함수 프로토타입 선언 ---

// 통합 제어 함수
void dxl_torque_set(uint8_t on_roll, uint8_t on_pitch, uint8_t on_knee,
		uint8_t on_wheel);
void DXL_Emergency_All_Off(void); // 비상 정지 (모든 토크 해제)

// 유틸리티 함수
uint16_t clc_speed_1(int16_t wheel_speed); // AX 모터 속도값 변환

// 개별 제어 함수 (Write Instruction)
void dxl_write_1_0(uint8_t id, uint8_t addr, uint8_t data_len, uint16_t data);
void dxl_write_2_0(uint8_t id, uint16_t addr, uint8_t data_len, uint32_t data);

// Protocol 2.0 Read 통신 실패 단계
enum DxlReadFailStage {
	DXL_READ_FAIL_NONE = 0,
	DXL_READ_FAIL_ARGUMENT = 1,
	DXL_READ_FAIL_HEADER_RX = 2,
	DXL_READ_FAIL_HEADER = 3,
	DXL_READ_FAIL_LENGTH = 4,
	DXL_READ_FAIL_BODY_RX = 5,
	DXL_READ_FAIL_CRC = 6,
	DXL_READ_FAIL_STATUS = 7,
	DXL_READ_FAIL_DATA = 8,
	DXL_READ_FAIL_HEADER_RX_TIMEOUT_BASE = 20, // 20~26: 수신 바이트 수 0~6
	DXL_READ_FAIL_HEADER_RX_ERROR = 28,
	DXL_READ_FAIL_HEADER_RX_BUSY = 29,
	DXL_READ_FAIL_DIRECTION = 30,
	DXL_READ_FAIL_RECEIVER = 31,
	DXL_READ_FAIL_OVERRUN = 32,
	DXL_READ_FAIL_FRAMING = 33,
	DXL_READ_FAIL_NOISE = 34,
	DXL_READ_FAIL_RX_PIN_LOW = 35,
	DXL_READ_FAIL_RX_ACTIVITY = 36,
	DXL_READ_FAIL_TX_ERROR = 37,
	DXL_READ_FAIL_TX_BUSY = 38,
	DXL_READ_FAIL_TX_TIMEOUT = 39,
	DXL_READ_FAIL_DIRECT_RX_TIMEOUT_BASE = 40, // 직접 수신 바이트 수 = STAGE - 40
};
// Protocol 2.0 개별 Ping
uint8_t dxl_ping_2_0(uint8_t id, uint8_t *device_error,
		uint8_t *fail_stage);

// Protocol 2.0 개별 Read
uint8_t dxl_read_2_0(uint8_t id, uint16_t addr, uint16_t data_len,
		uint8_t *data, uint8_t *device_error, uint8_t *fail_stage);

// 동기화 제어 함수 (Sync Write)
void send_sync_write_1(uint16_t addr, uint16_t data_len, int16_t *wheel_data);

// 내부 연산 및 패킷 생성 함수
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
		unsigned short data_blk_size);
uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length);

// Protocol 1.0 Sync Write 패킷 생성 함수 (바퀴 모터용)
dxl_sync_write_1_0 creat_sync_packet_1_0(void);

// Protocol 2.0 Sync Write 1바이트 (여러 모터 동시 제어)
void dxl_sync_write_1byte(uint8_t *ids, uint8_t *values, uint8_t count,
		uint16_t addr);

// Protocol 2.0 Sync Write 4바이트 (여러 모터 동시 제어)
void dxl_sync_write_4byte(uint8_t *ids, uint32_t *values, uint8_t count,
		uint16_t addr);

// Dynamixel 패킷 전송 함수 (USART1 바퀴, USART6 다리)
HAL_StatusTypeDef uart_transmit_packet(UART_HandleTypeDef *huart,
		uint8_t *data, uint16_t size);

#endif /* INC_DXL_2_0_H_ */
