// dxl_2_0.c
// Dynamixel Protocol 1.0(AX/USART1) + Protocol 2.0(MX/USART6) 제어 드라이버
// USART1 : 바퀴(AX) / USART3 : Jetson 통신 / USART6 : 다리(MX)

#include "main.h"
#include "dxl_2_0.h"
#include "stdint.h"
#include "usart.h"
#include <stdbool.h>

// 송신 상태 플래그
volatile uint8_t uart_tx_busy = 0;

// 송신 버퍼 (CPU 직접 접근용)
static uint8_t dxl_tx_buffer[256];

// 메인 루프에서 정의된 UART 핸들 참조
extern UART_HandleTypeDef huart1; // 바퀴(AX, 1Mbps) 제어용
extern UART_HandleTypeDef huart3; // Jetson VCP 통신용
extern UART_HandleTypeDef huart6; // 다리(MX, 2Mbps) 제어용

// 다리별 관절 모터 ID 매핑 (Protocol 2.0, MX 계열)
// 1: Front Left, 2: Front Right, 3: Rear Left, 4: Rear Right
// 끝자리 1: HIP(ROLL, MX-64T), 끝자리 2: HIP(PITCH, MX-106T), 끝자리 3: KNEE(MX-64T)
LegMotors legs[5] = { { 0, 0, 0 }, { 1, 2, 3 }, { 11, 12, 13 }, { 21, 22, 23 },
		{ 31, 32, 33 } };

// 바퀴 모터 ID 매핑 (Protocol 1.0, AX 계열)
// 1: Front Left, 2: Front Right, 3: Rear Left, 4: Rear Right
// 끝자리 4: WHEEL(AX-12+)
WheelMotors wheels[5] = { { 0 }, { 4 }, { 14 }, { 24 }, { 34 } };

// CRC 테이블 (Protocol 2.0용)
const unsigned short crc_table[256] = { 0x0000, 0x8005, 0x800F, 0x000A, 0x801B,
		0x001E, 0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D,
		0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077,
		0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0,
		0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
		0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C,
		0x8099, 0x0088, 0x808D, 0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189,
		0x0198, 0x819D, 0x8197, 0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB,
		0x01AE, 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE,
		0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7,
		0x01C2, 0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
		0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132, 0x0110, 0x8115,
		0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101, 0x8303, 0x0306, 0x030C,
		0x8309, 0x0318, 0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A,
		0x832B, 0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A, 0x837B,
		0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D,
		0x8347, 0x0342, 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4,
		0x83D1, 0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390,
		0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381, 0x0280, 0x8285,
		0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC,
		0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9,
		0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB,
		0x02CE, 0x02C4, 0x82C1, 0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D,
		0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
		0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202 };

// Protocol 2.0 CRC 계산
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,
		unsigned short data_blk_size) {
	unsigned short i, j;
	for (j = 0; j < data_blk_size; j++) {
		i = ((unsigned short) (crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	return crc_accum;
}

// Protocol 1.0 Checksum 계산
uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length) {
	uint8_t checksum = 0;
	for (uint16_t i = 2; i < length; i++)
		checksum += data[i];
	return ~checksum;
}

// Sync Write 패킷 구조체 초기화 (P1.0 - 바퀴)
dxl_sync_write_1_0 creat_sync_packet_1_0(void) {
	dxl_sync_write_1_0 sw1;
	sw1.header[0] = 0xFF;
	sw1.header[1] = 0XFF;
	sw1.id = 0xFE; // Broadcasting ID
	sw1.inst = 0x83; // Sync Write Instruction
	return sw1;
}

// UART 패킷 전송 및 TX Enable 제어 (USART1/USART6 분기)
void uart_transmit_packet(UART_HandleTypeDef *huart, uint8_t *data,
		uint16_t size) {

	// SN74LS241N 버퍼 Enable 제어 (코드 기준)
	// TX 전송 시 EN=HIGH, 전송 완료 후 EN=LOW

	// 1) TX Enable
	if (huart->Instance == USART1) {
		// 바퀴용 (AX 시리즈)
		HAL_GPIO_WritePin(DXL_TX_EN1_GPIO_Port, DXL_TX_EN1_Pin, GPIO_PIN_SET);
	} else if (huart->Instance == USART6) {
		// 다리용 (MX 시리즈)
		HAL_GPIO_WritePin(DXL_TX_EN6_GPIO_Port, DXL_TX_EN6_Pin, GPIO_PIN_SET);
	}

	// (선택) EN 안정화용 짧은 지연
	// for (volatile int i = 0; i < 50; i++) { __NOP(); }

	// 2) 데이터 전송 (Timeout 20ms)
	(void) HAL_UART_Transmit(huart, data, size, 20);

	// 3) 전송 완료(TC)까지 대기 (마지막 stop bit까지)
	while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
		;
	}

	// 4) TX Disable (버스 반환)
	if (huart->Instance == USART1) {
		HAL_GPIO_WritePin(DXL_TX_EN1_GPIO_Port, DXL_TX_EN1_Pin, GPIO_PIN_RESET);
	} else if (huart->Instance == USART6) {
		HAL_GPIO_WritePin(DXL_TX_EN6_GPIO_Port, DXL_TX_EN6_Pin, GPIO_PIN_RESET);
	}
}

// Sync Write 전송 - Protocol 1.0 (바퀴 제어용, UART1 유지)
void send_sync_write_1(uint16_t addr, uint16_t data_len, int16_t *wheel_data) {
	uint8_t id_count = 4;
	uint8_t id[4];
	uint16_t data_array[4];
	dxl_sync_write_1_0 sw1 = creat_sync_packet_1_0();

	for (int i = 1; i <= 4; i++) {
		id[i - 1] = wheels[i].wheel;
		data_array[i - 1] = clc_speed_1(wheel_data[i - 1]);
	}

	sw1.start_addr = addr;
	sw1.data_len = data_len;
	sw1.length = 4 + (id_count * (sw1.data_len + 1));

	uint16_t index = 0;
	dxl_tx_buffer[index++] = sw1.header[0];
	dxl_tx_buffer[index++] = sw1.header[1];
	dxl_tx_buffer[index++] = sw1.id;
	dxl_tx_buffer[index++] = sw1.length;
	dxl_tx_buffer[index++] = sw1.inst;
	dxl_tx_buffer[index++] = sw1.start_addr;
	dxl_tx_buffer[index++] = sw1.data_len;

	for (int i = 0; i < id_count; i++) {
		dxl_tx_buffer[index++] = id[i];
		for (int j = 0; j < sw1.data_len; j++) {
			dxl_tx_buffer[index++] = (data_array[i] >> (8 * j)) & 0xFF;
		}
	}

	sw1.checksum = calculate_checksum_1_0(dxl_tx_buffer, index);
	dxl_tx_buffer[index++] = sw1.checksum;

	// 바퀴는 UART1으로 전송
	uart_transmit_packet(&huart1, dxl_tx_buffer, index);
}

// 전체 모터 토크 제어 (통합 인터페이스)
void dxl_torque_set(uint8_t on_roll, uint8_t on_pitch, uint8_t on_knee,
		uint8_t on_wheel) {
	uint8_t leg_ids[12];
	uint8_t leg_torque[12];
	int16_t wheel_t[4] = { on_wheel, on_wheel, on_wheel, on_wheel };

	for (int i = 1; i <= 4; i++) {
		leg_ids[3 * (i - 1)] = legs[i].roll;
		leg_torque[3 * (i - 1)] = on_roll;

		leg_ids[3 * (i - 1) + 1] = legs[i].pitch;
		leg_torque[3 * (i - 1) + 1] = on_pitch;

		leg_ids[3 * (i - 1) + 2] = legs[i].knee;
		leg_torque[3 * (i - 1) + 2] = on_knee;
	}

	// 다리 모터 (MX, UART6) 토크 설정
	dxl_sync_write_1byte(leg_ids, leg_torque, 12, DXL_2_Torque_Enable);

	// 통신 충돌 방지용 미세 딜레이
	for (volatile int i = 0; i < 10000; i++) {
		__NOP();
	}

	// 바퀴 모터 (AX, UART1) 토크 설정
	send_sync_write_1(DXL_1_Torque_Enable, 1, wheel_t);
}

// 비상 정지 (전체 토크 해제)
void DXL_Emergency_All_Off(void) {
	dxl_torque_set(0, 0, 0, 0);
}

// 속도값 변환 (Protocol 1.0 양방향 속도 처리)
uint16_t clc_speed_1(int16_t wheel_speed) {
	if (wheel_speed < -1023 || wheel_speed > 1023)
		return 0;
	if (wheel_speed < 0)
		return (uint16_t) (-wheel_speed) + 1024;
	return (uint16_t) wheel_speed;
}

// Protocol 1.0 개별 Write (바퀴용, UART1 유지)
void dxl_write_1_0(uint8_t id, uint8_t addr, uint8_t data_len, uint16_t data) {
	uint8_t idx = 0;
	dxl_tx_buffer[idx++] = 0xFF;
	dxl_tx_buffer[idx++] = 0xFF;
	dxl_tx_buffer[idx++] = id;

	// Length = Inst(1) + Addr(1) + Data(N) + Checksum(1)
	dxl_tx_buffer[idx++] = 3 + data_len;

	dxl_tx_buffer[idx++] = 0x03; // Write
	dxl_tx_buffer[idx++] = addr;
	dxl_tx_buffer[idx++] = data & 0xFF;

	if (data_len == 2) {
		dxl_tx_buffer[idx++] = (data >> 8) & 0xFF;
	}

	dxl_tx_buffer[idx] = calculate_checksum_1_0(dxl_tx_buffer, idx);
	idx++;

	uart_transmit_packet(&huart1, dxl_tx_buffer, idx);
}

// Protocol 2.0 개별 Write (다리용, UART6로 변경)
void dxl_write_2_0(uint8_t id, uint16_t addr, uint8_t data_len, uint32_t data) {
	uint16_t idx = 0;
	dxl_tx_buffer[idx++] = 0xFF;
	dxl_tx_buffer[idx++] = 0xFF;
	dxl_tx_buffer[idx++] = 0xFD;
	dxl_tx_buffer[idx++] = 0x00;
	dxl_tx_buffer[idx++] = id;

	// Length = Inst(1) + Addr(2) + Data(N) + CRC(2)
	uint16_t len = 5 + data_len;
	dxl_tx_buffer[idx++] = len & 0xFF;
	dxl_tx_buffer[idx++] = (len >> 8) & 0xFF;

	dxl_tx_buffer[idx++] = 0x03; // Write
	dxl_tx_buffer[idx++] = addr & 0xFF;
	dxl_tx_buffer[idx++] = (addr >> 8) & 0xFF;

	for (int i = 0; i < data_len; i++) {
		dxl_tx_buffer[idx++] = (data >> (8 * i)) & 0xFF;
	}

	unsigned short crc = update_crc(0, dxl_tx_buffer, idx);
	dxl_tx_buffer[idx++] = crc & 0xFF;
	dxl_tx_buffer[idx++] = (crc >> 8) & 0xFF;

	// 다리 모터는 UART6 사용
	uart_transmit_packet(&huart6, dxl_tx_buffer, idx);
}

// Protocol 2.0 Sync Write 1바이트 (여러 모터 동시 제어)
void dxl_sync_write_1byte(uint8_t *ids, uint8_t *values, uint8_t count,
		uint16_t addr) {
	uint8_t tx_packet[256];
	uint16_t idx = 0;
	uint16_t length;
	uint16_t crc;
	uint8_t i;

	// 입력값 유효성 검사
	if (ids == NULL || values == NULL || count == 0) {
		return;
	}

	// 패킷 길이 계산
	length = 7 + (count * 2);

	// 패킷 버퍼 초과 방지
	if ((length + 7) > sizeof(tx_packet)) {
		return;
	}

	// 패킷 헤더
	tx_packet[idx++] = 0xFF;
	tx_packet[idx++] = 0xFF;
	tx_packet[idx++] = 0xFD;
	tx_packet[idx++] = 0x00;

	// Broadcast ID
	tx_packet[idx++] = 0xFE;

	// 패킷 길이
	tx_packet[idx++] = length & 0xFF;
	tx_packet[idx++] = (length >> 8) & 0xFF;

	// Sync Write 명령
	tx_packet[idx++] = 0x83;

	// 시작 주소
	tx_packet[idx++] = addr & 0xFF;
	tx_packet[idx++] = (addr >> 8) & 0xFF;

	// 데이터 길이 (1바이트)
	tx_packet[idx++] = 0x01;
	tx_packet[idx++] = 0x00;

	// 모터 ID와 데이터
	for (i = 0; i < count; i++) {
		tx_packet[idx++] = ids[i];
		tx_packet[idx++] = values[i];
	}

	// CRC 추가
	crc = update_crc(0, tx_packet, idx);
	tx_packet[idx++] = crc & 0xFF;
	tx_packet[idx++] = (crc >> 8) & 0xFF;

	// UART6로 패킷 전송
	uart_transmit_packet(&huart6, tx_packet, idx);
}

// Protocol 2.0 Sync Write 4바이트 (여러 모터 동시 제어)
void dxl_sync_write_4byte(uint8_t *ids, uint32_t *values, uint8_t count,
		uint16_t addr) {
	uint8_t tx_packet[256];
	uint16_t idx = 0;
	uint16_t length;
	uint16_t crc;
	uint8_t i;

	// 입력값 유효성 검사
	if (ids == NULL || values == NULL || count == 0) {
		return;
	}

	// 패킷 길이 계산
	length = 7 + (count * 5);

	// 패킷 버퍼 초과 방지
	if ((length + 7) > sizeof(tx_packet)) {
		return;
	}

	// 패킷 헤더
	tx_packet[idx++] = 0xFF;
	tx_packet[idx++] = 0xFF;
	tx_packet[idx++] = 0xFD;
	tx_packet[idx++] = 0x00;

	// Broadcast ID
	tx_packet[idx++] = 0xFE;

	// 패킷 길이
	tx_packet[idx++] = length & 0xFF;
	tx_packet[idx++] = (length >> 8) & 0xFF;

	// Sync Write 명령
	tx_packet[idx++] = 0x83;

	// 시작 주소
	tx_packet[idx++] = addr & 0xFF;
	tx_packet[idx++] = (addr >> 8) & 0xFF;

	// 데이터 길이 (4바이트)
	tx_packet[idx++] = 0x04;
	tx_packet[idx++] = 0x00;

	// 모터 ID와 데이터
	for (i = 0; i < count; i++) {
		tx_packet[idx++] = ids[i];
		tx_packet[idx++] = (uint8_t) (values[i] & 0xFF);
		tx_packet[idx++] = (uint8_t) ((values[i] >> 8) & 0xFF);
		tx_packet[idx++] = (uint8_t) ((values[i] >> 16) & 0xFF);
		tx_packet[idx++] = (uint8_t) ((values[i] >> 24) & 0xFF);
	}

	// CRC 추가
	crc = update_crc(0, tx_packet, idx);
	tx_packet[idx++] = crc & 0xFF;
	tx_packet[idx++] = (crc >> 8) & 0xFF;

	// UART6로 패킷 전송
	uart_transmit_packet(&huart6, tx_packet, idx);
}
