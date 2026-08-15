/*
 * jetson_comm.c
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */
#include "jetson_comm.h"
#include "robot_motion.h"
#include "imu_driver.h"
#include "dxl_2_0.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *jetson_huart;

static uint8_t rx_byte;
static char rx_line[64];
static uint8_t rx_index = 0;
static volatile uint8_t line_ready = 0;

// 12개 MX 모터 상태 바이너리 패킷 규격
#define JETSON_MOTOR_STATE_BIN_HEADER_0		0x57U
#define JETSON_MOTOR_STATE_BIN_HEADER_1		0x4BU
#define JETSON_MOTOR_STATE_BIN_VERSION		0x01U
#define JETSON_MOTOR_STATE_BIN_TYPE		0x01U
#define JETSON_MOTOR_STATE_BIN_HEADER_SIZE	16U
#define JETSON_MOTOR_STATE_BIN_DATA_SIZE	10U
#define JETSON_MOTOR_STATE_BIN_CRC_SIZE		2U
#define JETSON_MOTOR_STATE_BIN_PACKET_SIZE	\
		(JETSON_MOTOR_STATE_BIN_HEADER_SIZE \
		+ (DXL_LEG_MOTOR_COUNT * JETSON_MOTOR_STATE_BIN_DATA_SIZE) \
		+ JETSON_MOTOR_STATE_BIN_CRC_SIZE)

// 모터 상태 바이너리 DMA 송신 버퍼와 상태
static uint8_t jetson_motor_state_tx_packet[
		JETSON_MOTOR_STATE_BIN_PACKET_SIZE];
static volatile uint8_t jetson_motor_state_tx_busy = 0U;
static volatile uint32_t jetson_motor_state_total_start_cycle = 0U;
static volatile uint32_t jetson_motor_state_uart_tx_start_cycle = 0U;

// 모터 상태 바이너리 처리시간 측정값
static volatile uint32_t jetson_motor_state_sync_read_us = 0U;
static volatile uint32_t jetson_motor_state_packet_build_us = 0U;
static volatile uint32_t jetson_motor_state_dma_start_us = 0U;
static volatile uint32_t jetson_motor_state_cpu_active_us = 0U;
static volatile uint32_t jetson_motor_state_uart_tx_us = 0U;
static volatile uint32_t jetson_motor_state_total_us = 0U;
static volatile HAL_StatusTypeDef jetson_motor_state_tx_status = HAL_ERROR;

// Jetson 문자열 송신
static void Jetson_Send_Text(const char *text) {
	(void) HAL_UART_Transmit(jetson_huart, (uint8_t*) text,
			(uint16_t) strlen(text), 20);
}

// 16비트 값을 Little Endian 순서로 저장
static void Jetson_Write_U16_LE(uint8_t *buffer, uint16_t *index,
		uint16_t value) {
	buffer[(*index)++] = (uint8_t) (value & 0xFFU);
	buffer[(*index)++] = (uint8_t) ((value >> 8) & 0xFFU);
}

// 32비트 값을 Little Endian 순서로 저장
static void Jetson_Write_U32_LE(uint8_t *buffer, uint16_t *index,
		uint32_t value) {
	buffer[(*index)++] = (uint8_t) (value & 0xFFU);
	buffer[(*index)++] = (uint8_t) ((value >> 8) & 0xFFU);
	buffer[(*index)++] = (uint8_t) ((value >> 16) & 0xFFU);
	buffer[(*index)++] = (uint8_t) ((value >> 24) & 0xFFU);
}

// 바이너리 패킷 CRC-16/CCITT-FALSE 계산
static uint16_t Jetson_Calculate_CRC16_CCITT(const uint8_t *data,
		uint16_t length) {
	uint16_t crc = 0xFFFFU;

	for (uint16_t i = 0U; i < length; i++) {
		crc ^= (uint16_t) data[i] << 8;

		for (uint8_t bit = 0U; bit < 8U; bit++) {
			if ((crc & 0x8000U) != 0U) {
				crc = (uint16_t) ((crc << 1) ^ 0x1021U);
			} else {
				crc = (uint16_t) (crc << 1);
			}
		}
	}

	return crc;
}

// CPU Cycle을 마이크로초로 변환
static uint32_t Jetson_Cycles_To_Us(uint32_t cycles) {
	if (SystemCoreClock == 0U) {
		return 0U;
	}

	return (uint32_t) (((uint64_t) cycles * 1000000ULL)
			/ (uint64_t) SystemCoreClock);
}

// IMU 각도를 100배 정수로 변환
static int32_t Jetson_Angle_To_X100(float angle) {
	if (angle >= 0.0f)
		return (int32_t) (angle * 100.0f + 0.5f);

	return (int32_t) (angle * 100.0f - 0.5f);
}

// 최신 IMU 데이터 송신
static void Jetson_Send_IMU(void) {
	IMU_Data_t data = IMU_Get_Latest_Data();
	char tx_line[160];

	int32_t roll_x100 = Jetson_Angle_To_X100(data.roll);
	int32_t pitch_x100 = Jetson_Angle_To_X100(data.pitch);
	int32_t yaw_x100 = Jetson_Angle_To_X100(data.yaw);

	int length = snprintf(tx_line, sizeof(tx_line),
			"IMU:ROLL_X100=%ld,PITCH_X100=%ld,YAW_X100=%ld,"
					"VALID=%u,SEQ=%lu,TIME=%lu\r\n", (long) roll_x100,
			(long) pitch_x100, (long) yaw_x100, (unsigned int) data.valid,
			(unsigned long) data.sequence, (unsigned long) data.timestamp_ms);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// 다리 관절 모터 ID 확인 함수 선언
static uint8_t Jetson_Is_Leg_Motor_ID(uint8_t id);

// 12개 다리 모터 전체 상태 송신
static void Jetson_Send_All_Motor_State(void) {
	DXL_Leg_State_t state;
	char tx_part[128];
	int length;

	(void) dxl_sync_read_leg_state(&state);

	length = snprintf(tx_part, sizeof(tx_part),
			"MOTOR_STATE_ALL:SEQ=%lu,TIME=%lu,COUNT=%u,"
			"VALID_MASK=0x%03X,ALL_VALID=%u",
			(unsigned long) state.sequence,
			(unsigned long) state.timestamp_ms,
			(unsigned int) state.received_count,
			(unsigned int) state.valid_mask,
			(state.valid_mask == 0x0FFFU) ? 1U : 0U);

	if (length <= 0 || length >= (int) sizeof(tx_part)) {
		Jetson_Send_Text(
				"MOTOR_STATE_ALL:VALID=0,REASON=FORMAT\r\n");
		return;
	}

	Jetson_Send_Text(tx_part);

	for (uint8_t i = 0U; i < DXL_LEG_MOTOR_COUNT; i++) {
		const DXL_Leg_Motor_State_t *motor = &state.motors[i];

		length = snprintf(tx_part, sizeof(tx_part),
				"|ID=%u,POS=%ld,VEL=%ld,CURRENT_RAW=%d,"
				"STATUS_ERR=%u,VALID=%u",
				(unsigned int) motor->id,
				(long) motor->position_raw,
				(long) motor->velocity_raw,
				(int) motor->current_raw,
				(unsigned int) motor->status_error,
				(unsigned int) motor->valid);

		if (length <= 0 || length >= (int) sizeof(tx_part)) {
			Jetson_Send_Text("|FORMAT_ERROR=1\r\n");
			return;
		}

		Jetson_Send_Text(tx_part);
	}

	Jetson_Send_Text("\r\n");
}

// 12개 다리 모터 전체 상태 바이너리 송신
static void Jetson_Send_All_Motor_State_Binary(void) {
	DXL_Leg_State_t state;
	uint16_t tx_index = 0U;
	uint16_t crc;
	uint32_t total_start_cycle;
	uint32_t sync_read_end_cycle;
	uint32_t packet_build_end_cycle;
	uint32_t dma_start_end_cycle;
	HAL_StatusTypeDef tx_status;

	jetson_motor_state_sync_read_us = 0U;
	jetson_motor_state_packet_build_us = 0U;
	jetson_motor_state_dma_start_us = 0U;
	jetson_motor_state_cpu_active_us = 0U;
	jetson_motor_state_uart_tx_us = 0U;
	jetson_motor_state_total_us = 0U;
	jetson_motor_state_tx_status = HAL_ERROR;

	total_start_cycle = DWT->CYCCNT;

	(void) dxl_sync_read_leg_state(&state);

	sync_read_end_cycle = DWT->CYCCNT;
	jetson_motor_state_sync_read_us = Jetson_Cycles_To_Us(
			sync_read_end_cycle - total_start_cycle);

	// 이전 바이너리 패킷 DMA 송신 중에는 새 패킷을 덮어쓰지 않음
	if (jetson_motor_state_tx_busy != 0U) {
		dma_start_end_cycle = DWT->CYCCNT;
		jetson_motor_state_cpu_active_us = Jetson_Cycles_To_Us(
				dma_start_end_cycle - total_start_cycle);
		jetson_motor_state_total_us = jetson_motor_state_cpu_active_us;
		jetson_motor_state_tx_status = HAL_BUSY;
		return;
	}

	// 패킷 Header와 기본 정보 저장
	jetson_motor_state_tx_packet[tx_index++] =
			JETSON_MOTOR_STATE_BIN_HEADER_0;
	jetson_motor_state_tx_packet[tx_index++] =
			JETSON_MOTOR_STATE_BIN_HEADER_1;
	jetson_motor_state_tx_packet[tx_index++] =
			JETSON_MOTOR_STATE_BIN_VERSION;
	jetson_motor_state_tx_packet[tx_index++] =
			JETSON_MOTOR_STATE_BIN_TYPE;

	Jetson_Write_U16_LE(jetson_motor_state_tx_packet, &tx_index,
			(uint16_t) JETSON_MOTOR_STATE_BIN_PACKET_SIZE);
	Jetson_Write_U32_LE(jetson_motor_state_tx_packet, &tx_index,
			state.sequence);
	Jetson_Write_U32_LE(jetson_motor_state_tx_packet, &tx_index,
			state.timestamp_ms);
	Jetson_Write_U16_LE(jetson_motor_state_tx_packet, &tx_index,
			state.valid_mask);

	// 모터 ID 순서: 1, 2, 3, 11, 12, 13, 21, 22, 23, 31, 32, 33
	for (uint8_t i = 0U; i < DXL_LEG_MOTOR_COUNT; i++) {
		const DXL_Leg_Motor_State_t *motor = &state.motors[i];

		Jetson_Write_U32_LE(jetson_motor_state_tx_packet, &tx_index,
				(uint32_t) motor->position_raw);
		Jetson_Write_U32_LE(jetson_motor_state_tx_packet, &tx_index,
				(uint32_t) motor->velocity_raw);
		Jetson_Write_U16_LE(jetson_motor_state_tx_packet, &tx_index,
				(uint16_t) motor->current_raw);
	}

	// CRC를 제외한 길이가 정확한지 확인
	if (tx_index
			!= (JETSON_MOTOR_STATE_BIN_PACKET_SIZE
					- JETSON_MOTOR_STATE_BIN_CRC_SIZE)) {
		jetson_motor_state_tx_status = HAL_ERROR;
		return;
	}

	crc = Jetson_Calculate_CRC16_CCITT(
			jetson_motor_state_tx_packet, tx_index);

	Jetson_Write_U16_LE(jetson_motor_state_tx_packet, &tx_index, crc);

	// 최종 패킷 길이가 정확한 경우에만 송신
	if (tx_index != JETSON_MOTOR_STATE_BIN_PACKET_SIZE) {
		jetson_motor_state_tx_status = HAL_ERROR;
		return;
	}

	packet_build_end_cycle = DWT->CYCCNT;
	jetson_motor_state_packet_build_us = Jetson_Cycles_To_Us(
			packet_build_end_cycle - sync_read_end_cycle);

	// DMA 송신 완료 콜백에서 전체 처리시간을 계산할 수 있도록 시작값 저장
	jetson_motor_state_total_start_cycle = total_start_cycle;
	jetson_motor_state_uart_tx_start_cycle = DWT->CYCCNT;
	jetson_motor_state_tx_busy = 1U;

	// DMA가 송신 버퍼의 최신 데이터를 읽도록 메모리 접근 완료
	__DMB();

	tx_status = HAL_UART_Transmit_DMA(jetson_huart,
			jetson_motor_state_tx_packet, tx_index);

	dma_start_end_cycle = DWT->CYCCNT;

	jetson_motor_state_dma_start_us = Jetson_Cycles_To_Us(
			dma_start_end_cycle - packet_build_end_cycle);
	jetson_motor_state_cpu_active_us = Jetson_Cycles_To_Us(
			dma_start_end_cycle - total_start_cycle);
	jetson_motor_state_total_us = jetson_motor_state_cpu_active_us;
	jetson_motor_state_tx_status = tx_status;

	// DMA 송신 시작에 실패한 경우 버퍼를 다시 사용할 수 있도록 해제
	if (tx_status != HAL_OK) {
		jetson_motor_state_tx_busy = 0U;
	}
}

// 모터 상태 바이너리 처리시간 송신
static void Jetson_Send_Motor_State_Timing(void) {
	char tx_line[224];

	int length = snprintf(tx_line, sizeof(tx_line),
			"MOTOR_STATE_TIMING:CORE_CLOCK_HZ=%lu,"
			"SYNC_READ_US=%lu,PACKET_BUILD_US=%lu,"
			"DMA_START_US=%lu,CPU_ACTIVE_US=%lu,"
			"UART_TX_US=%lu,TOTAL_US=%lu,TX_STATUS=%u\r\n",
			(unsigned long) SystemCoreClock,
			(unsigned long) jetson_motor_state_sync_read_us,
			(unsigned long) jetson_motor_state_packet_build_us,
			(unsigned long) jetson_motor_state_dma_start_us,
			(unsigned long) jetson_motor_state_cpu_active_us,
			(unsigned long) jetson_motor_state_uart_tx_us,
			(unsigned long) jetson_motor_state_total_us,
			(unsigned int) jetson_motor_state_tx_status);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// 현재 모터 속도 송신
static void Jetson_Send_Motor_Velocity(const char *id_text) {
	char *end_ptr;
	unsigned long id_value;
	uint8_t velocity_data[4];
	uint8_t device_error = 0xFF;
	uint8_t fail_stage = DXL_READ_FAIL_NONE;
	char tx_line[96];

	id_value = strtoul(id_text, &end_ptr, 10);

	if (id_text == end_ptr || *end_ptr != '\0' || id_value > 252
			|| !Jetson_Is_Leg_Motor_ID((uint8_t) id_value)) {
		Jetson_Send_Text("MOTOR_VEL:VALID=0,REASON=ID\r\n");
		return;
	}

	uint8_t id = (uint8_t) id_value;

	if (!dxl_read_2_0(id, DXL_2_Present_Velocity, 4, velocity_data,
			&device_error, &fail_stage)) {
		int length = snprintf(tx_line, sizeof(tx_line),
				"MOTOR_VEL:ID=%u,VALID=0,REASON=COMM,STAGE=%u\r\n",
				(unsigned int) id, (unsigned int) fail_stage);

		if (length > 0 && length < (int) sizeof(tx_line)) {
			Jetson_Send_Text(tx_line);
		}
		return;
	}

	int32_t velocity = (int32_t) ((uint32_t) velocity_data[0]
			| ((uint32_t) velocity_data[1] << 8)
			| ((uint32_t) velocity_data[2] << 16)
			| ((uint32_t) velocity_data[3] << 24));

	int length = snprintf(tx_line, sizeof(tx_line),
			"MOTOR_VEL:ID=%u,VEL=%ld,STATUS_ERR=%u,VALID=1\r\n",
			(unsigned int) id, (long) velocity, (unsigned int) device_error);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// 현재 모터 전류 송신
static void Jetson_Send_Motor_Current(const char *id_text) {
	char *end_ptr;
	unsigned long id_value;
	uint8_t current_data[2];
	uint8_t device_error = 0xFF;
	uint8_t fail_stage = DXL_READ_FAIL_NONE;
	char tx_line[128];

	id_value = strtoul(id_text, &end_ptr, 10);

	if (id_text == end_ptr || *end_ptr != '\0' || id_value > 252
			|| !Jetson_Is_Leg_Motor_ID((uint8_t) id_value)) {
		Jetson_Send_Text("MOTOR_CURRENT:VALID=0,REASON=ID\r\n");
		return;
	}

	uint8_t id = (uint8_t) id_value;

	if (!dxl_read_2_0(id, DXL_2_Present_Current, 2, current_data, &device_error,
			&fail_stage)) {
		int length = snprintf(tx_line, sizeof(tx_line),
				"MOTOR_CURRENT:ID=%u,VALID=0,REASON=COMM,STAGE=%u\r\n",
				(unsigned int) id, (unsigned int) fail_stage);

		if (length > 0 && length < (int) sizeof(tx_line)) {
			Jetson_Send_Text(tx_line);
		}
		return;
	}

	int16_t current_raw = (int16_t) ((uint16_t) current_data[0]
			| ((uint16_t) current_data[1] << 8));
	int32_t current_ma_x100 = (int32_t) current_raw * 336;

	int length = snprintf(tx_line, sizeof(tx_line),
			"MOTOR_CURRENT:ID=%u,CURRENT_RAW=%d,CURRENT_MA_X100=%ld,"
					"STATUS_ERR=%u,VALID=1\r\n", (unsigned int) id,
			(int) current_raw, (long) current_ma_x100,
			(unsigned int) device_error);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// 모터 하드웨어 에러 상태 송신
static void Jetson_Send_Motor_Hardware_Error(const char *id_text) {
	char *end_ptr;
	unsigned long id_value;
	uint8_t hardware_error = 0U;
	uint8_t device_error = 0xFFU;
	uint8_t fail_stage = DXL_READ_FAIL_NONE;
	char tx_line[96];

	id_value = strtoul(id_text, &end_ptr, 10);

	if (id_text == end_ptr || *end_ptr != '\0' || id_value > 252
			|| !Jetson_Is_Leg_Motor_ID((uint8_t) id_value)) {
		Jetson_Send_Text("MOTOR_ERROR:VALID=0,REASON=ID\r\n");
		return;
	}

	uint8_t id = (uint8_t) id_value;

	if (!dxl_read_2_0(id, DXL_2_Hardware_Error_Status, 1, &hardware_error,
			&device_error, &fail_stage)) {
		int length = snprintf(tx_line, sizeof(tx_line),
				"MOTOR_ERROR:ID=%u,VALID=0,REASON=COMM,STAGE=%u\r\n",
				(unsigned int) id, (unsigned int) fail_stage);

		if (length > 0 && length < (int) sizeof(tx_line)) {
			Jetson_Send_Text(tx_line);
		}
		return;
	}

	int length = snprintf(tx_line, sizeof(tx_line),
			"MOTOR_ERROR:ID=%u,HW_ERROR=%u,STATUS_ERR=%u,VALID=1\r\n",
			(unsigned int) id, (unsigned int) hardware_error,
			(unsigned int) device_error);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// 다리 관절 모터 ID 확인
static uint8_t Jetson_Is_Leg_Motor_ID(uint8_t id) {
	for (int i = 1; i <= leg_count; i++) {
		if (id == legs[i].roll || id == legs[i].pitch || id == legs[i].knee) {
			return 1;
		}
	}

	return 0;
}

// 모터 Ping 결과 송신
static void Jetson_Send_Motor_Ping(const char *id_text) {
	char *end_ptr;
	unsigned long id_value;
	uint8_t device_error = 0xFF;
	uint8_t fail_stage = DXL_READ_FAIL_NONE;
	char tx_line[96];

	id_value = strtoul(id_text, &end_ptr, 10);

	if (id_text == end_ptr || *end_ptr != '\0'
			|| (id_value > 252 && id_value != 254)
			|| (id_value != 254 && !Jetson_Is_Leg_Motor_ID((uint8_t) id_value))) {
		Jetson_Send_Text("MOTOR_PING:VALID=0,REASON=ID\r\n");
		return;
	}

	uint8_t id = (uint8_t) id_value;

	if (!dxl_ping_2_0(id, &device_error, &fail_stage)) {
		int length = snprintf(tx_line, sizeof(tx_line),
				"MOTOR_PING:ID=%u,VALID=0,REASON=COMM,STAGE=%u\r\n",
				(unsigned int) id, (unsigned int) fail_stage);

		if (length > 0 && length < (int) sizeof(tx_line)) {
			Jetson_Send_Text(tx_line);
		}
		return;
	}

	int length = snprintf(tx_line, sizeof(tx_line),
			"MOTOR_PING:ID=%u,STATUS_ERR=%u,VALID=1\r\n", (unsigned int) id,
			(unsigned int) device_error);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// 현재 모터 위치 송신
static void Jetson_Send_Motor_Position(const char *id_text) {
	char *end_ptr;
	unsigned long id_value;
	uint8_t position_data[4];
	uint8_t device_error = 0xFF;
	uint8_t fail_stage = DXL_READ_FAIL_NONE;
	char tx_line[96];

	id_value = strtoul(id_text, &end_ptr, 10);

	if (id_text == end_ptr || *end_ptr != '\0' || id_value > 252
			|| !Jetson_Is_Leg_Motor_ID((uint8_t) id_value)) {
		Jetson_Send_Text("MOTOR:VALID=0,REASON=ID\r\n");
		return;
	}

	uint8_t id = (uint8_t) id_value;

	if (!dxl_read_2_0(id, DXL_2_Present_Position, 4, position_data,
			&device_error, &fail_stage)) {
		int length = snprintf(tx_line, sizeof(tx_line),
				"MOTOR:ID=%u,VALID=0,REASON=COMM,STAGE=%u\r\n",
				(unsigned int) id, (unsigned int) fail_stage);

		if (length > 0 && length < (int) sizeof(tx_line)) {
			Jetson_Send_Text(tx_line);
		}
		return;
	}

	int32_t position = (int32_t) ((uint32_t) position_data[0]
			| ((uint32_t) position_data[1] << 8)
			| ((uint32_t) position_data[2] << 16)
			| ((uint32_t) position_data[3] << 24));

	int length = snprintf(tx_line, sizeof(tx_line),
			"MOTOR:ID=%u,POS=%ld,STATUS_ERR=%u,VALID=1\r\n", (unsigned int) id,
			(long) position, (unsigned int) device_error);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// Jetson 통신 초기화
void Jetson_Comm_Init(UART_HandleTypeDef *huart) {
	jetson_huart = huart;
	rx_index = 0;
	line_ready = 0;

	// 통신 소요시간 측정을 위한 CPU Cycle Counter 활성화
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55UL;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	HAL_UART_Receive_IT(jetson_huart, &rx_byte, 1);
}

// Jetson UART DMA 송신 완료 콜백
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	uint32_t uart_tx_end_cycle;

	if (huart->Instance == USART3
			&& jetson_motor_state_tx_busy != 0U) {
		uart_tx_end_cycle = DWT->CYCCNT;

		jetson_motor_state_uart_tx_us = Jetson_Cycles_To_Us(
				uart_tx_end_cycle
						- jetson_motor_state_uart_tx_start_cycle);
		jetson_motor_state_total_us = Jetson_Cycles_To_Us(
				uart_tx_end_cycle
						- jetson_motor_state_total_start_cycle);

		jetson_motor_state_tx_busy = 0U;
	}
}

// Jetson UART 수신 완료 콜백
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		RobotMode_t current_mode = Robot_Get_Mode();

		// 모드 전환 중 수신된 Jetson 명령 폐기
		if (current_mode == MODE_TO_WHEEL || current_mode == MODE_TO_LEG) {
			rx_index = 0;
			line_ready = 0;

			HAL_UART_Receive_IT(jetson_huart, &rx_byte, 1);
			return;
		}

		if (!line_ready) {
			if (rx_byte == '\n' || rx_byte == '\r') {
				rx_line[rx_index] = '\0';

				if (rx_index > 0) {
					line_ready = 1;
				}

				rx_index = 0;
			} else {
				if (rx_index < sizeof(rx_line) - 1) {
					rx_line[rx_index++] = rx_byte;
				} else {
					rx_index = 0;
				}
			}
		}

		HAL_UART_Receive_IT(jetson_huart, &rx_byte, 1);
	}
}

// Jetson 명령 처리
void Jetson_Comm_Task(void) {
	char command[sizeof(rx_line)];

	if (!line_ready) {
		return;
	}

	// 수신 명령을 별도 공간에 복사
	strncpy(command, rx_line, sizeof(command) - 1);
	command[sizeof(command) - 1] = '\0';
	line_ready = 0;

	if (strcmp(command, "PING") == 0) {
		Jetson_Send_Text("OK\r\n");
	} else if (strcmp(command, "GET_IMU") == 0) {
		Jetson_Send_IMU();
	} else if (strcmp(command, "GET_MOTOR_STATE_BIN") == 0) {
		Jetson_Send_All_Motor_State_Binary();
	} else if (strcmp(command, "GET_MOTOR_STATE_TIMING") == 0) {
		Jetson_Send_Motor_State_Timing();
	} else if (strcmp(command, "GET_MOTOR_STATE_ALL") == 0) {
		Jetson_Send_All_Motor_State();
	} else if (strncmp(command, "PING_MOTOR:", 11) == 0) {
		Jetson_Send_Motor_Ping(command + 11);
	} else if (strncmp(command, "GET_MOTOR_CURRENT:", 18) == 0) {
		Jetson_Send_Motor_Current(command + 18);
	} else if (strncmp(command, "GET_MOTOR_ERROR:", 16) == 0) {
		Jetson_Send_Motor_Hardware_Error(command + 16);
	} else if (strncmp(command, "GET_MOTOR_VEL:", 14) == 0) {
		Jetson_Send_Motor_Velocity(command + 14);
	} else if (strncmp(command, "GET_MOTOR:", 10) == 0) {
		Jetson_Send_Motor_Position(command + 10);
	} else if (strcmp(command, "STOP") == 0) {
		Robot_Wheel_Stop();
	} else if (strcmp(command, "FWD") == 0) {
		Robot_Wheel_Forward();
	} else if (strcmp(command, "BACK") == 0) {
		Robot_Wheel_Backward();
	} else if (strcmp(command, "TL") == 0) {
		Robot_Tank_Left();
	} else if (strcmp(command, "TR") == 0) {
		Robot_Tank_Right();
	} else if (strcmp(command, "WHEEL") == 0) {
		Robot_Set_Wheel_Mode();
	} else if (strcmp(command, "LEG") == 0) {
		Robot_Set_Leg_Mode();
	}
}

