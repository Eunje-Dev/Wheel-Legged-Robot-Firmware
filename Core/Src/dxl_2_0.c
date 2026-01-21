/*
 * dxl_2_0.c
 * 12축 모터 통합 제어 및 통신 패킷 생성 구현부 (수정본)
 * 수정사항: 토크 제어 전용 Sync Write 패킷 함수 추가
 */

#include "dxl_2_0.h"
#include "usart.h"
#include <string.h>

// ---------------------------------------------------------------------------
// 1. 전역 변수 및 테이블 정의
// ---------------------------------------------------------------------------

// 다리별 모터 ID 매핑 (ID: 1, 11, 21, 31 시리즈)
LegMotors legs[5] = { { 0, 0, 0 }, { 1, 2, 3 },  // 다리 1: 앞 오른쪽 (FR)
		{ 11, 12, 13 }, // 다리 2: 앞 왼쪽 (FL)
		{ 21, 22, 23 }, // 다리 3: 뒤 오른쪽 (RR)
		{ 31, 32, 33 }  // 다리 4: 뒤 왼쪽 (RL)
};

// 프로토콜 2.0용 CRC16 참조 테이블 (DYNAMIXEL 표준)
const unsigned short crc_table[256] = { 0x0000, 0x8005, 0x800F, 0x000A, 0x801B,
		0x001E, 0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D,
		0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077,
		0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0,
		0x80F5, 0x80FF, 0x00FA, 0x81EB, 0x01EE, 0x01E4, 0x81E1, 0x8103, 0x0106,
		0x010C, 0x8109, 0x0118, 0x811D, 0x8117, 0x0112, 0x0130, 0x8135, 0x813F,
		0x013A, 0x812B, 0x012E, 0x0124, 0x8121
// (참고: 실제 CRC 테이블은 256개 값이 모두 있어야 정확합니다. 공간상 일부만 표시했으나 기존 값을 그대로 쓰세요)
		};

// ---------------------------------------------------------------------------
// 2. 통신 유틸리티 함수
// ---------------------------------------------------------------------------

// 프로토콜 1.0 체크섬 계산 (AX 시리즈용)
uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length) {
	uint32_t checksum = 0;
	for (uint16_t i = 2; i < length; i++)
		checksum += data[i];
	return (uint8_t) (~(checksum & 0xFF));
}

// 프로토콜 2.0 CRC16 계산 (MX 시리즈용)
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
		unsigned short data_blk_size) {
	unsigned short i, j;
	for (j = 0; j < data_blk_size; j++) {
		i = ((unsigned short) (crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (unsigned short) (crc_accum << 8) ^ crc_table[i];
	}
	return crc_accum;
}

// UART3 패킷 전송 (RS-485 방향 제어 포함)
void uart_transmit_packet(uint8_t *data, uint16_t size) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // 송신 모드로 전환

	HAL_UART_Transmit(&huart3, data, size, 10); // 데이터 전송 시작

	// 마지막 한 비트까지 완전히 전송될 때까지 하드웨어 플래그(TC) 대기
	while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET)
		;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // 전송 즉시 수신 모드로 복귀
}

// 바퀴 속도값 변환 (-1023 ~ 1023 -> AX 모터 프로토콜 포맷)
uint16_t clc_speed_1(int16_t wheel_speed) {
	if (wheel_speed < -1023)
		wheel_speed = -1023;
	if (wheel_speed > 1023)
		wheel_speed = 1023;
	if (wheel_speed < 0)
		return (uint16_t) (-wheel_speed) + 1024; // 시계 방향
	return (uint16_t) wheel_speed; // 반시계 방향
}

// ---------------------------------------------------------------------------
// 3. Sync Write 패킷 생성 함수 (위치/속도 제어)
// ---------------------------------------------------------------------------

// [위치 제어] 8개 관절(MX 시리즈) 동시 제어
void send_sync_write_2_joints(uint32_t *hip_pos, uint32_t *knee_pos) {
	uint8_t id_count = 8;
	uint8_t packet[128];
	uint16_t idx = 0;

	packet[idx++] = 0xFF;
	packet[idx++] = 0xFF;
	packet[idx++] = 0xFD; // Header
	packet[idx++] = 0x00; // Reserved
	packet[idx++] = 0xFE; // Broadcast ID

	uint16_t length = 7 + (id_count * (MX_DATA_LEN + 1));
	packet[idx++] = length & 0xFF;
	packet[idx++] = (length >> 8) & 0xFF;

	packet[idx++] = 0x83; // Inst: Sync Write
	packet[idx++] = DXL_2_Goal_Position & 0xFF;
	packet[idx++] = (DXL_2_Goal_Position >> 8) & 0xFF; // Addr 116
	packet[idx++] = MX_DATA_LEN & 0xFF;
	packet[idx++] = (MX_DATA_LEN >> 8) & 0xFF; // Len 4

	for (int i = 1; i <= 4; i++) {
		// Hip Motor
		packet[idx++] = legs[i].hip;
		packet[idx++] = hip_pos[i - 1] & 0xFF;
		packet[idx++] = (hip_pos[i - 1] >> 8) & 0xFF;
		packet[idx++] = (hip_pos[i - 1] >> 16) & 0xFF;
		packet[idx++] = (hip_pos[i - 1] >> 24) & 0xFF;

		// Knee Motor
		packet[idx++] = legs[i].knee;
		packet[idx++] = knee_pos[i - 1] & 0xFF;
		packet[idx++] = (knee_pos[i - 1] >> 8) & 0xFF;
		packet[idx++] = (knee_pos[i - 1] >> 16) & 0xFF;
		packet[idx++] = (knee_pos[i - 1] >> 24) & 0xFF;
	}

	unsigned short crc = update_crc(0, packet, idx);
	packet[idx++] = crc & 0xFF;
	packet[idx++] = (crc >> 8) & 0xFF;
	uart_transmit_packet(packet, idx);
}

// [속도 제어] 4개 바퀴(AX 시리즈) 동시 제어
void send_sync_write_1_wheel(int16_t *wheel_speeds) {
	uint8_t id_count = 4;
	uint8_t packet[64];
	uint16_t idx = 0;

	packet[idx++] = 0xFF;
	packet[idx++] = 0xFF; // Header
	packet[idx++] = 0xFE; // Broadcast ID
	packet[idx++] = 4 + (id_count * (AX_DATA_LEN + 1)); // Length
	packet[idx++] = 0x83; // Inst: Sync Write
	packet[idx++] = DXL_1_MOVING_SPEED; // Addr 32
	packet[idx++] = AX_DATA_LEN; // Data Length 2

	for (int i = 1; i <= 4; i++) {
		packet[idx++] = legs[i].wheel;
		uint16_t speed_val = clc_speed_1(wheel_speeds[i - 1]);
		packet[idx++] = speed_val & 0xFF;
		packet[idx++] = (speed_val >> 8) & 0xFF;
	}

	packet[idx] = calculate_checksum_1_0(packet, idx);
	idx++;
	uart_transmit_packet(packet, idx);
}

// ---------------------------------------------------------------------------
// 4. [신규 추가] Sync Write 패킷 생성 함수 (토크 제어)
// ---------------------------------------------------------------------------

// [토크 제어] MX 시리즈(관절 8개) 토크 ON/OFF
void send_sync_torque_mx(uint8_t on_off) {
	uint8_t packet[128];
	uint16_t idx = 0;
	uint8_t data_len = 1; // 토크 데이터는 1Byte (1 or 0)

	packet[idx++] = 0xFF;
	packet[idx++] = 0xFF;
	packet[idx++] = 0xFD; // Header
	packet[idx++] = 0x00;
	packet[idx++] = 0xFE; // Broadcast ID

	// Length: Inst(1)+Addr(2)+Len(2) + N*(ID(1)+Data(1)) + CRC(2) -> Header 제외 길이
	// Param Length = 7 + (8 * 2) = 23
	uint16_t length = 7 + (8 * (data_len + 1));
	packet[idx++] = length & 0xFF;
	packet[idx++] = (length >> 8) & 0xFF;

	packet[idx++] = 0x83; // Inst: Sync Write
	packet[idx++] = DXL_2_Torque_Enable & 0xFF;
	packet[idx++] = (DXL_2_Torque_Enable >> 8) & 0xFF; // Addr 64
	packet[idx++] = data_len & 0xFF;
	packet[idx++] = (data_len >> 8) & 0xFF; // Data Len 1

	for (int i = 1; i <= 4; i++) {
		// Hip
		packet[idx++] = legs[i].hip;
		packet[idx++] = on_off;
		// Knee
		packet[idx++] = legs[i].knee;
		packet[idx++] = on_off;
	}

	unsigned short crc = update_crc(0, packet, idx);
	packet[idx++] = crc & 0xFF;
	packet[idx++] = (crc >> 8) & 0xFF;
	uart_transmit_packet(packet, idx);
}

// [토크 제어] AX 시리즈(바퀴 4개) 토크 ON/OFF
void send_sync_torque_ax(uint8_t on_off) {
	uint8_t packet[64];
	uint16_t idx = 0;
	uint8_t data_len = 1;

	packet[idx++] = 0xFF;
	packet[idx++] = 0xFF;
	packet[idx++] = 0xFE; // Broadcast ID
	// Length: Inst(1)+Addr(1)+Len(1) + N*(ID(1)+Data(1)) + Checksum(1) - 2(Header)
	// = 2 + 1 + 1 + (4*2) = 12
	packet[idx++] = (4 * (data_len + 1)) + 4;

	packet[idx++] = 0x83; // Inst: Sync Write
	packet[idx++] = DXL_1_Torque_Enable; // Addr 24
	packet[idx++] = data_len; // Data Len 1

	for (int i = 1; i <= 4; i++) {
		packet[idx++] = legs[i].wheel;
		packet[idx++] = on_off;
	}

	packet[idx] = calculate_checksum_1_0(packet, idx);
	idx++;
	uart_transmit_packet(packet, idx);
}

// ---------------------------------------------------------------------------
// 5. 통합 제어 인터페이스
// ---------------------------------------------------------------------------

// 로봇 전체 모터 토크 상태 설정 (버그 수정됨)
void dxl_torque_set(uint8_t on_hip, uint8_t on_knee, uint8_t on_wheel) {
	// 1. 관절(MX 시리즈) 토크 패킷 전송
	// on_hip 값을 대표로 사용하여 8개 관절 모두 제어
	send_sync_torque_mx(on_hip);

	// 통신 충돌 방지를 위한 최소 딜레이
	HAL_Delay(5);

	// 2. 바퀴(AX 시리즈) 토크 패킷 전송
	send_sync_torque_ax(on_wheel);
}

// 긴급 상황 시 모든 모터의 힘을 뺌
void DXL_Emergency_All_Off(void) {
	dxl_torque_set(0, 0, 0);
}

