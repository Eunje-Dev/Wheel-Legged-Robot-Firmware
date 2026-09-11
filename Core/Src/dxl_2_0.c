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

// 12개 MX Status Packet DMA 수신 버퍼
#define DXL_SYNC_READ_RX_BUFFER_SIZE         320U
#define DXL_SYNC_READ_STATUS_PACKET_MAX_SIZE 32U
#define DXL_SYNC_READ_TIMEOUT_MS             10U

static uint8_t dxl_sync_read_rx_buffer[DXL_SYNC_READ_RX_BUFFER_SIZE];

// 다리 모터 상태 배열 순서
static const uint8_t dxl_leg_motor_ids[DXL_LEG_MOTOR_COUNT] = {
		1, 2, 3,
		11, 12, 13,
		21, 22, 23,
		31, 32, 33
};

static uint32_t dxl_leg_state_sequence = 0U;

// 메인 루프용 최근 모터 조회 결과 저장 (부분 수신·실패 포함)
static DXL_Leg_State_t dxl_recent_leg_state;
static uint8_t dxl_recent_leg_state_available = 0U;

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

// 모터 ID에 해당하는 전체 상태 배열 위치 확인
static int8_t DXL_Sync_Read_Find_Motor_Index(uint8_t id) {
	for (uint8_t i = 0U; i < DXL_LEG_MOTOR_COUNT; i++) {
		if (dxl_leg_motor_ids[i] == id) {
			return (int8_t) i;
		}
	}

	return -1;
}

// 12개 다리 모터 상태 조회 결과 초기화
static void DXL_Sync_Read_Reset_State(DXL_Leg_State_t *state) {
	state->valid_mask = 0U;
	state->timestamp_ms = 0U;
	state->sequence = 0U;
	state->received_count = 0U;

	for (uint8_t i = 0U; i < DXL_LEG_MOTOR_COUNT; i++) {
		state->motors[i].id = dxl_leg_motor_ids[i];
		state->motors[i].current_raw = 0;
		state->motors[i].velocity_raw = 0;
		state->motors[i].position_raw = 0;
		state->motors[i].status_error = 0xFFU;
		state->motors[i].valid = 0U;
	}
}

// Sync Read Status Packet 한 개 검사 및 상태 저장
static uint8_t DXL_Sync_Read_Parse_Status_Packet(const uint8_t *packet,
		uint16_t total_length, DXL_Leg_State_t *state) {
	uint8_t data[DXL_SYNC_STATE_DATA_LENGTH];
	uint16_t packet_length;
	uint16_t received_crc;
	uint16_t calculated_crc;
	uint16_t src_index;
	uint16_t parameter_end;
	uint8_t data_index = 0U;
	int8_t motor_index;
	uint8_t state_index;

	if (packet == NULL || state == NULL || total_length < 11U) {
		return 0U;
	}

	// Status Packet 헤더 확인
	if (packet[0] != 0xFFU || packet[1] != 0xFFU
			|| packet[2] != 0xFDU || packet[3] != 0x00U) {
		return 0U;
	}

	packet_length = (uint16_t) packet[5]
			| ((uint16_t) packet[6] << 8);

	if (total_length != (uint16_t) (7U + packet_length)) {
		return 0U;
	}

	// Status Instruction 확인
	if (packet[7] != DXL_INST_STATUS) {
		return 0U;
	}

	// Status Packet CRC 확인
	received_crc = (uint16_t) packet[total_length - 2U]
			| ((uint16_t) packet[total_length - 1U] << 8);

	calculated_crc = update_crc(0U, (uint8_t*) packet,
			(uint16_t) (total_length - 2U));

	if (received_crc != calculated_crc) {
		return 0U;
	}

	motor_index = DXL_Sync_Read_Find_Motor_Index(packet[4]);

	if (motor_index < 0) {
		return 0U;
	}

	// Byte Stuffing을 제거하면서 10바이트 상태 데이터 복사
	src_index = 9U;
	parameter_end = total_length - 2U;

	while (src_index < parameter_end) {
		if (data_index >= DXL_SYNC_STATE_DATA_LENGTH) {
			return 0U;
		}

		data[data_index++] = packet[src_index++];

		if (data_index >= 3U
				&& data[data_index - 3U] == 0xFFU
				&& data[data_index - 2U] == 0xFFU
				&& data[data_index - 1U] == 0xFDU
				&& src_index < parameter_end
				&& packet[src_index] == 0xFDU) {
			src_index++;
		}
	}

	if (data_index != DXL_SYNC_STATE_DATA_LENGTH) {
		return 0U;
	}

	state_index = (uint8_t) motor_index;

	// 주소 126부터 Current, Velocity, Position 순서로 저장
	state->motors[state_index].current_raw =
			(int16_t) ((uint16_t) data[0]
					| ((uint16_t) data[1] << 8));

	state->motors[state_index].velocity_raw =
			(int32_t) ((uint32_t) data[2]
					| ((uint32_t) data[3] << 8)
					| ((uint32_t) data[4] << 16)
					| ((uint32_t) data[5] << 24));

	state->motors[state_index].position_raw =
			(int32_t) ((uint32_t) data[6]
					| ((uint32_t) data[7] << 8)
					| ((uint32_t) data[8] << 16)
					| ((uint32_t) data[9] << 24));

	state->motors[state_index].status_error = packet[8];

	if (!state->motors[state_index].valid) {
		state->received_count++;
	}

	state->motors[state_index].valid = 1U;
	state->valid_mask |= (uint16_t) (1U << state_index);

	return 1U;
}

// DMA 수신 버퍼에서 여러 Status Packet 검색
static void DXL_Sync_Read_Parse_Stream(uint16_t received_length,
		DXL_Leg_State_t *state) {
	uint16_t buffer_index = 0U;
	uint16_t packet_length;
	uint16_t total_length;

	if (state == NULL || received_length == 0U) {
		return;
	}

	if (received_length > DXL_SYNC_READ_RX_BUFFER_SIZE) {
		received_length = DXL_SYNC_READ_RX_BUFFER_SIZE;
	}

	while ((uint16_t) (buffer_index + 7U) <= received_length
			&& state->received_count < DXL_LEG_MOTOR_COUNT) {
		// Protocol 2.0 헤더 위치 검색
		if (dxl_sync_read_rx_buffer[buffer_index] != 0xFFU
				|| dxl_sync_read_rx_buffer[buffer_index + 1U] != 0xFFU
				|| dxl_sync_read_rx_buffer[buffer_index + 2U] != 0xFDU
				|| dxl_sync_read_rx_buffer[buffer_index + 3U] != 0x00U) {
			buffer_index++;
			continue;
		}

		packet_length =
				(uint16_t) dxl_sync_read_rx_buffer[buffer_index + 5U]
				| ((uint16_t) dxl_sync_read_rx_buffer[buffer_index + 6U] << 8);

		total_length = (uint16_t) (7U + packet_length);

		// 비정상 패킷 길이는 다음 바이트부터 다시 검색
		if (packet_length < 4U
				|| total_length > DXL_SYNC_READ_STATUS_PACKET_MAX_SIZE) {
			buffer_index++;
			continue;
		}

		// 마지막 패킷이 아직 완전히 수신되지 않은 경우 종료
		if ((uint16_t) (buffer_index + total_length) > received_length) {
			break;
		}

		if (DXL_Sync_Read_Parse_Status_Packet(
				&dxl_sync_read_rx_buffer[buffer_index], total_length,
				state)) {
			buffer_index = (uint16_t) (buffer_index + total_length);
		} else {
			// CRC 또는 패킷 검증 실패 시 다음 헤더를 다시 검색
			buffer_index++;
		}
	}
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
HAL_StatusTypeDef uart_transmit_packet(UART_HandleTypeDef *huart,
		uint8_t *data, uint16_t size) {
	HAL_StatusTypeDef tx_status;

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
	tx_status = HAL_UART_Transmit(huart, data, size, 20);

	// 3) 전송 완료(TC)까지 대기 (마지막 stop bit까지)
	if (tx_status == HAL_OK) {
		while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {
			;
		}
	}

	// 4) TX Disable (버스 반환)
	if (huart->Instance == USART1) {
		HAL_GPIO_WritePin(DXL_TX_EN1_GPIO_Port, DXL_TX_EN1_Pin, GPIO_PIN_RESET);
	} else if (huart->Instance == USART6) {
		HAL_GPIO_WritePin(DXL_TX_EN6_GPIO_Port, DXL_TX_EN6_Pin, GPIO_PIN_RESET);
	}

	return tx_status;
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

// Protocol 2.0 개별 Read (다리용, UART6)
uint8_t dxl_read_2_0(uint8_t id, uint16_t addr, uint16_t data_len,
		uint8_t *data, uint8_t *device_error, uint8_t *fail_stage) {
	uint8_t tx_packet[14];
	uint8_t rx_packet[64];
	uint16_t tx_idx = 0;
	uint16_t packet_length;
	uint16_t total_length;
	uint16_t received_crc;
	uint16_t received_count;
	uint16_t src_idx;
	uint16_t dst_idx = 0;
	uint16_t param_end;
	uint16_t calculated_crc;
	uint32_t rx_start_tick;
	uint32_t primask;
	HAL_StatusTypeDef tx_status;

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_ARGUMENT;
	}

	if (data == NULL || data_len == 0 || data_len > 32 || id > 252) {
		return 0;
	}

	if (device_error != NULL) {
		*device_error = 0xFF;
	}

	// Read 명령 패킷 생성
	tx_packet[tx_idx++] = 0xFF;
	tx_packet[tx_idx++] = 0xFF;
	tx_packet[tx_idx++] = 0xFD;
	tx_packet[tx_idx++] = 0x00;
	tx_packet[tx_idx++] = id;

	// Length = Instruction(1) + Parameter(4) + CRC(2)
	tx_packet[tx_idx++] = 0x07;
	tx_packet[tx_idx++] = 0x00;

	tx_packet[tx_idx++] = DXL_INST_READ;
	tx_packet[tx_idx++] = addr & 0xFF;
	tx_packet[tx_idx++] = (addr >> 8) & 0xFF;
	tx_packet[tx_idx++] = data_len & 0xFF;
	tx_packet[tx_idx++] = (data_len >> 8) & 0xFF;

	calculated_crc = update_crc(0, tx_packet, tx_idx);
	tx_packet[tx_idx++] = calculated_crc & 0xFF;
	tx_packet[tx_idx++] = (calculated_crc >> 8) & 0xFF;

	// 이전 UART 수신 데이터 제거
	__HAL_UART_CLEAR_OREFLAG(&huart6);
	__HAL_UART_SEND_REQ(&huart6, UART_RXDATA_FLUSH_REQUEST);

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_HEADER_RX;
	}

	// Read 명령 전송 후 수신 방향으로 전환
	primask = __get_PRIMASK();
	__disable_irq();
	tx_status = uart_transmit_packet(&huart6, tx_packet, tx_idx);
	__set_PRIMASK(primask);

	if (tx_status != HAL_OK) {
		if (fail_stage != NULL) {
			if (tx_status == HAL_BUSY) {
				*fail_stage = DXL_READ_FAIL_TX_BUSY;
			} else if (tx_status == HAL_TIMEOUT) {
				*fail_stage = DXL_READ_FAIL_TX_TIMEOUT;
			} else {
				*fail_stage = DXL_READ_FAIL_TX_ERROR;
			}
		}

		return 0;
	}

	received_count = 0U;
	rx_start_tick = HAL_GetTick();

	// Status Packet 헤더를 USART6 수신 레지스터에서 직접 확인
	while (received_count < 7U
			&& (HAL_GetTick() - rx_start_tick) < 10U) {
		if ((huart6.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
			rx_packet[received_count++] =
					(uint8_t) (huart6.Instance->RDR & 0xFFU);
		}
	}

	if (received_count != 7U) {
		if (fail_stage != NULL) {
			*fail_stage = DXL_READ_FAIL_DIRECT_RX_TIMEOUT_BASE
					+ (uint8_t) received_count;
		}

		return 0;
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_HEADER;
	}

	// Status Packet 헤더와 모터 ID 확인
	if (rx_packet[0] != 0xFF || rx_packet[1] != 0xFF || rx_packet[2] != 0xFD
			|| rx_packet[3] != 0x00 || rx_packet[4] != id) {
		return 0;
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_LENGTH;
	}

	packet_length = (uint16_t) rx_packet[5] | ((uint16_t) rx_packet[6] << 8);

	if (packet_length < 4 || packet_length > (sizeof(rx_packet) - 7)) {
		return 0;
	}

	total_length = 7U + packet_length;

	// Status Packet 본문을 USART6 수신 레지스터에서 직접 확인
	while (received_count < total_length
			&& (HAL_GetTick() - rx_start_tick) < 10U) {
		if ((huart6.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
			rx_packet[received_count++] =
					(uint8_t) (huart6.Instance->RDR & 0xFFU);
		}
	}

	if (received_count != total_length) {
		if (fail_stage != NULL) {
			*fail_stage = DXL_READ_FAIL_DIRECT_RX_TIMEOUT_BASE
					+ (uint8_t) received_count;
		}

		return 0;
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_CRC;
	}

	// Status Packet CRC 확인
	received_crc = (uint16_t) rx_packet[total_length - 2]
			| ((uint16_t) rx_packet[total_length - 1] << 8);

	calculated_crc = update_crc(0, rx_packet, total_length - 2);

	if (received_crc != calculated_crc) {
		return 0;
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_STATUS;
	}

	if (rx_packet[7] != DXL_INST_STATUS) {
		return 0;
	}

	if (device_error != NULL) {
		*device_error = rx_packet[8];
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_DATA;
	}

	// Byte Stuffing을 제거하면서 요청한 데이터 복사
	src_idx = 9;
	param_end = total_length - 2;

	while (src_idx < param_end) {
		if (dst_idx >= data_len) {
			return 0;
		}

		data[dst_idx++] = rx_packet[src_idx++];

		if (dst_idx >= 3 && data[dst_idx - 3] == 0xFF
				&& data[dst_idx - 2] == 0xFF && data[dst_idx - 1] == 0xFD
				&& src_idx < param_end && rx_packet[src_idx] == 0xFD) {
			src_idx++;
		}
	}

	if (dst_idx != data_len) {
		return 0;
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_NONE;
	}

	return 1;
}

// 12개 다리 모터 Current, Velocity, Position Sync Read
static uint16_t DXL_Sync_Read_Leg_State_Uncached(DXL_Leg_State_t *state) {
	uint8_t tx_packet[32];
	uint16_t tx_index = 0U;
	uint16_t packet_length;
	uint16_t crc;
	uint16_t received_length = 0U;
	uint16_t previous_received_length = 0U;
	uint32_t start_tick;
	uint32_t primask;
	HAL_StatusTypeDef dma_status;
	HAL_StatusTypeDef tx_status;

	if (state == NULL) {
		return 0U;
	}

	DXL_Sync_Read_Reset_State(state);
	state->sequence = ++dxl_leg_state_sequence;

	if (huart6.hdmarx == NULL) {
		state->timestamp_ms = HAL_GetTick();
		return 0U;
	}

	// Length = Instruction(1) + Address(2) + Data Length(2) + IDs(12) + CRC(2)
	packet_length = (uint16_t) (7U + DXL_LEG_MOTOR_COUNT);

	// Protocol 2.0 Sync Read 패킷 생성
	tx_packet[tx_index++] = 0xFFU;
	tx_packet[tx_index++] = 0xFFU;
	tx_packet[tx_index++] = 0xFDU;
	tx_packet[tx_index++] = 0x00U;
	tx_packet[tx_index++] = 0xFEU;

	tx_packet[tx_index++] = (uint8_t) (packet_length & 0xFFU);
	tx_packet[tx_index++] = (uint8_t) ((packet_length >> 8) & 0xFFU);

	tx_packet[tx_index++] = DXL_INST_SYNC_READ;

	// Present Current 주소 126부터 10바이트 조회
	tx_packet[tx_index++] =
			(uint8_t) (DXL_2_Present_Current & 0xFFU);
	tx_packet[tx_index++] =
			(uint8_t) ((DXL_2_Present_Current >> 8) & 0xFFU);

	tx_packet[tx_index++] =
			(uint8_t) (DXL_SYNC_STATE_DATA_LENGTH & 0xFFU);
	tx_packet[tx_index++] =
			(uint8_t) ((DXL_SYNC_STATE_DATA_LENGTH >> 8) & 0xFFU);

	// 12개 다리 모터 ID 추가
	for (uint8_t i = 0U; i < DXL_LEG_MOTOR_COUNT; i++) {
		tx_packet[tx_index++] = dxl_leg_motor_ids[i];
	}

	crc = update_crc(0U, tx_packet, tx_index);
	tx_packet[tx_index++] = (uint8_t) (crc & 0xFFU);
	tx_packet[tx_index++] = (uint8_t) ((crc >> 8) & 0xFFU);

	// 이전 USART6 수신 데이터와 오류 플래그 제거
	__HAL_UART_CLEAR_FLAG(&huart6,
			UART_CLEAR_OREF | UART_CLEAR_NEF
					| UART_CLEAR_PEF | UART_CLEAR_FEF);
	__HAL_UART_SEND_REQ(&huart6, UART_RXDATA_FLUSH_REQUEST);

	// 수신 오류가 발생해도 USART6 RX DMA가 계속 동작하도록 설정
	CLEAR_BIT(huart6.Instance->CR3, USART_CR3_DDRE);

	// 첫 번째 모터 응답을 놓치지 않도록 송신 전에 RX DMA 시작
	dma_status = HAL_UART_Receive_DMA(&huart6,
			dxl_sync_read_rx_buffer, DXL_SYNC_READ_RX_BUFFER_SIZE);

	if (dma_status != HAL_OK) {
		state->timestamp_ms = HAL_GetTick();
		return 0U;
	}

	// 일시적인 USART6 오류로 HAL이 RX DMA 전체를 중단하지 않도록 설정
	__HAL_UART_DISABLE_IT(&huart6, UART_IT_ERR);

	// 절반 수신 인터럽트는 사용하지 않음
	__HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);

	// 송신 완료와 RX 방향 전환 사이의 인터럽트 지연 방지
	primask = __get_PRIMASK();
	__disable_irq();
	tx_status = uart_transmit_packet(&huart6, tx_packet, tx_index);
	__set_PRIMASK(primask);

	if (tx_status != HAL_OK) {
		(void) HAL_UART_AbortReceive(&huart6);
		state->timestamp_ms = HAL_GetTick();
		return 0U;
	}

	start_tick = HAL_GetTick();

	// 12개 Status Packet 검증 완료 또는 전체 Timeout까지 대기
	while ((HAL_GetTick() - start_tick) < DXL_SYNC_READ_TIMEOUT_MS) {
		received_length = (uint16_t) (DXL_SYNC_READ_RX_BUFFER_SIZE
				- __HAL_DMA_GET_COUNTER(huart6.hdmarx));

		if (received_length != previous_received_length) {
			previous_received_length = received_length;

			// 현재까지 완전히 수신된 Status Packet 검사
			__DMB();
			DXL_Sync_Read_Parse_Stream(received_length, state);

			if (state->received_count >= DXL_LEG_MOTOR_COUNT) {
				break;
			}
		}

		if (received_length >= DXL_SYNC_READ_RX_BUFFER_SIZE) {
			break;
		}
	}

	// DMA를 중지하기 전에 최종 수신 바이트 수 저장
	received_length = (uint16_t) (DXL_SYNC_READ_RX_BUFFER_SIZE
			- __HAL_DMA_GET_COUNTER(huart6.hdmarx));

	(void) HAL_UART_AbortReceive(&huart6);
	__DMB();

	DXL_Sync_Read_Parse_Stream(received_length, state);

	state->timestamp_ms = HAL_GetTick();

	return state->valid_mask;
}

// 12개 다리 모터 Current, Velocity, Position Sync Read
uint16_t dxl_sync_read_leg_state(DXL_Leg_State_t *state) {
	uint16_t valid_mask;

	if (state == NULL) {
		return 0U;
	}

	valid_mask = DXL_Sync_Read_Leg_State_Uncached(state);
	// 조회 실패 시에도 최근 결과 갱신
	dxl_recent_leg_state = *state;
	dxl_recent_leg_state_available = 1U;
	return valid_mask;
}

// 메인 루프에서 최근 조회 결과 재사용, 유효 시간 0이면 새로 조회
uint16_t dxl_get_recent_leg_state(DXL_Leg_State_t *state, uint32_t max_age_ms) {
	if (state == NULL) {
		return 0U;
	}

	if (dxl_recent_leg_state_available != 0U
			&& (uint32_t) (HAL_GetTick() - dxl_recent_leg_state.timestamp_ms)
					< max_age_ms) {
		// 조회 시각·순번·유효성 유지
		*state = dxl_recent_leg_state;
		return state->valid_mask;
	}

	return dxl_sync_read_leg_state(state);
}

// Protocol 2.0 개별 Ping (다리용, UART6)
uint8_t dxl_ping_2_0(uint8_t id, uint8_t *device_error,
		uint8_t *fail_stage) {
	uint8_t tx_packet[10];
	uint8_t rx_packet[32];
	uint16_t tx_idx = 0;
	uint16_t packet_length;
	uint16_t total_length;
	uint16_t received_crc;
	uint16_t calculated_crc;
	uint16_t received_count;
	uint32_t rx_start_tick;
	uint32_t primask;
	const uint16_t expected_total_length = 14;
	HAL_StatusTypeDef tx_status;

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_ARGUMENT;
	}

	if (id > 252U) {
		return 0;
	}

	if (device_error != NULL) {
		*device_error = 0xFF;
	}

	// Ping 명령 패킷 생성
	tx_packet[tx_idx++] = 0xFF;
	tx_packet[tx_idx++] = 0xFF;
	tx_packet[tx_idx++] = 0xFD;
	tx_packet[tx_idx++] = 0x00;
	tx_packet[tx_idx++] = id;
	tx_packet[tx_idx++] = 0x03;
	tx_packet[tx_idx++] = 0x00;
	tx_packet[tx_idx++] = DXL_INST_PING;

	calculated_crc = update_crc(0, tx_packet, tx_idx);
	tx_packet[tx_idx++] = calculated_crc & 0xFF;
	tx_packet[tx_idx++] = (calculated_crc >> 8) & 0xFF;

	// 이전 UART 수신 데이터 제거
	__HAL_UART_CLEAR_OREFLAG(&huart6);
	__HAL_UART_SEND_REQ(&huart6, UART_RXDATA_FLUSH_REQUEST);

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_HEADER_RX;
	}

	// HAL 인터럽트 수신을 사용하지 않고 USART6 레지스터를 직접 확인

	// Ping 명령 전송 후 Status Packet 수신 완료 대기
	// 송신 완료와 수신 방향 전환 사이의 인터럽트 지연 방지
	primask = __get_PRIMASK();
	__disable_irq();
	tx_status = uart_transmit_packet(&huart6, tx_packet, tx_idx);
	__set_PRIMASK(primask);

	if (tx_status != HAL_OK) {
		(void) HAL_UART_AbortReceive(&huart6);

		if (fail_stage != NULL) {
			if (tx_status == HAL_BUSY) {
				*fail_stage = DXL_READ_FAIL_TX_BUSY;
			} else if (tx_status == HAL_TIMEOUT) {
				*fail_stage = DXL_READ_FAIL_TX_TIMEOUT;
			} else {
				*fail_stage = DXL_READ_FAIL_TX_ERROR;
			}
		}

		return 0;
	}

	received_count = 0U;
	rx_start_tick = HAL_GetTick();

	// USART6 수신 레지스터 직접 확인
	while (received_count < expected_total_length
			&& (HAL_GetTick() - rx_start_tick) < 10U) {
		if ((huart6.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
			rx_packet[received_count++] =
					(uint8_t) (huart6.Instance->RDR & 0xFFU);
		}
	}

	if (received_count != expected_total_length) {
		if (fail_stage != NULL) {
			*fail_stage = DXL_READ_FAIL_DIRECT_RX_TIMEOUT_BASE
					+ (uint8_t) received_count;
		}

		return 0;
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_HEADER;
	}

	// Status Packet 헤더와 모터 ID 확인
	if (rx_packet[0] != 0xFF || rx_packet[1] != 0xFF || rx_packet[2] != 0xFD
			|| rx_packet[3] != 0x00
			|| (id != 0xFE && rx_packet[4] != id)) {
		return 0;
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_LENGTH;
	}

	packet_length = (uint16_t) rx_packet[5] | ((uint16_t) rx_packet[6] << 8);

	if (packet_length != 7) {
		return 0;
	}

	total_length = 7 + packet_length;

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_CRC;
	}

	// Status Packet CRC 확인
	received_crc = (uint16_t) rx_packet[total_length - 2]
			| ((uint16_t) rx_packet[total_length - 1] << 8);

	calculated_crc = update_crc(0, rx_packet, total_length - 2);

	if (received_crc != calculated_crc) {
		return 0;
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_STATUS;
	}

	if (rx_packet[7] != DXL_INST_STATUS) {
		return 0;
	}

	if (device_error != NULL) {
		*device_error = rx_packet[8];
	}

	if (fail_stage != NULL) {
		*fail_stage = DXL_READ_FAIL_NONE;
	}

	return 1;
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
static uint8_t DXL_Sync_Write_4byte_Checked(const uint8_t *ids,
        const uint32_t *values, uint8_t count, uint16_t addr) {
	uint8_t tx_packet[256];
	uint16_t idx = 0;
	uint16_t length;
	uint16_t crc;
	uint8_t i;

	// 입력값 유효성 검사
	if (ids == NULL || values == NULL || count == 0) {
		return 0U;
	}

	// 패킷 길이 계산
	length = 7 + (count * 5);

	// 패킷 버퍼 초과 방지
	if (((uint32_t) length + 7U) > sizeof(tx_packet)) {
		return 0U;
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
	return uart_transmit_packet(&huart6, tx_packet, idx) == HAL_OK ? 1U : 0U;
}

void dxl_sync_write_4byte(uint8_t *ids, uint32_t *values, uint8_t count,
        uint16_t addr)
{
    // Protocol 2.0 Sync Write 4바이트 (여러 모터 동시 제어)
    (void) DXL_Sync_Write_4byte_Checked(ids, values, count, addr);
}

uint8_t dxl_sync_write_goal_positions_checked(const uint8_t *ids,
        const uint32_t *values, uint8_t count)
{
    uint8_t i, j;
    if (ids == NULL || values == NULL || count == 0U || count > DXL_LEG_MOTOR_COUNT)
        return 0U;
    for (i = 0U; i < count; ++i) {
        if (ids[i] == 0U || ids[i] >= 0xFDU || values[i] > 4095U) return 0U;
        for (j = 0U; j < i; ++j) if (ids[i] == ids[j]) return 0U;
    }
    // 단일 회전 목표 범위로 바이트 삽입 불필요
    // 반환값은 송신 결과, 모터 응답·동작 확인과 별도
    return DXL_Sync_Write_4byte_Checked(ids, values, count, DXL_2_Goal_Position);
}
