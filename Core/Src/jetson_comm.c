/*
 * jetson_comm.c
 *
 *  Created on: 2026. 5. 16.
 *      Author: 이은제
 */
#include "jetson_comm.h"
#include "robot_motion.h"
#include "woki_ramp_control.h"
#include "imu_driver.h"
#include "dxl_2_0.h"
#include "woki_power_guard.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *jetson_huart;

static uint8_t rx_byte;
static char rx_line[64];
static uint8_t rx_index = 0;
// 수신 명령 대기열 크기
#define JETSON_COMMAND_QUEUE_CAPACITY 16U
// 우선 처리할 정지 명령
#define JETSON_STOP_NONE 0U
#define JETSON_STOP_WHEEL 1U
#define JETSON_STOP_LEG 2U
#define JETSON_STOP_SAFETY 3U
typedef struct {
    char text[sizeof(rx_line)];
    uint32_t motion_epoch;
} Jetson_Queued_Command_t;
static Jetson_Queued_Command_t jetson_command_queue[JETSON_COMMAND_QUEUE_CAPACITY];
static volatile uint8_t jetson_queue_head, jetson_queue_tail, jetson_queue_count;
static volatile uint8_t jetson_stop_pending;
static volatile uint32_t jetson_motion_epoch;
static volatile uint32_t jetson_rx_queue_overflow_count;
static volatile uint32_t jetson_rx_invalid_line_count;
static volatile uint32_t jetson_rx_stale_motion_count;
static RobotMode_t jetson_observed_mode;

// 수신 오류 복구 재시도 간격
// 송신 유지, 1바이트 인터럽트 수신만 복구
#define JETSON_RX_RECOVERY_RETRY_MS 10U
static volatile uint8_t jetson_rx_recovery_pending;
static volatile uint8_t jetson_rx_discard_to_eol;
static volatile uint32_t jetson_rx_error_count;
static volatile uint32_t jetson_rx_rearm_failure_count;
static uint32_t jetson_rx_last_retry_ms;

// 수신 인터럽트 또는 인터럽트 보호 구간에서 대기열 초기화
static void Jetson_ClearQueue_Unlocked(void)
{
    jetson_queue_head = jetson_queue_tail = jetson_queue_count = 0U;
    ++jetson_motion_epoch;
}

static void Jetson_DiscardPending_Unlocked(void)
{
    if (rx_index != 0U) jetson_rx_discard_to_eol = 1U;
    rx_index = 0U;
    Jetson_ClearQueue_Unlocked();
    jetson_stop_pending = 0U;
}

void Jetson_Comm_DiscardPendingCommands(void)
{
    const uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    Jetson_DiscardPending_Unlocked();
    __set_PRIMASK(irq_mask);
}

// Wheel·Leg 전환 완료 후 이전 명령 폐기
// 경사로 진입 후 전진·후진 명령 유지
static void Jetson_SyncMode_Unlocked(RobotMode_t mode)
{
    if (mode != jetson_observed_mode
            && (mode == MODE_WHEEL || mode == MODE_LEG)) {
        Jetson_DiscardPending_Unlocked();
    }
    jetson_observed_mode = mode;
}

static uint8_t Jetson_Is_Motion_Command(const char *command)
{
    return strcmp(command, "WHEEL") == 0 || strcmp(command, "LEG") == 0
        || strcmp(command, "RAMP") == 0
        || strcmp(command, "WHEEL_FWD") == 0
        || strcmp(command, "WHEEL_BACK") == 0
        || strcmp(command, "WHEEL_TL") == 0
        || strcmp(command, "WHEEL_TR") == 0
        || strcmp(command, "WHEEL_STOP") == 0
        || strcmp(command, "LEG_FWD") == 0
        || strcmp(command, "LEG_BACK") == 0
        || strcmp(command, "LEG_LEFT") == 0
        || strcmp(command, "LEG_RIGHT") == 0
        || strcmp(command, "LEG_STOP") == 0;
}

// 수신 완료 명령 저장
// 정지 명령 별도 보관 및 이전 주행 명령 무효화
static void Jetson_QueueLine_FromISR(void)
{
    const RobotMode_t mode = Robot_Get_Mode();
    uint8_t stop_request = JETSON_STOP_NONE;

    // 전원 절차 명령 수신 시 앞서 대기 중인 동작 명령을 무효화
    if (strcmp(rx_line, "PREPARE_POWER_OFF") == 0
            || strcmp(rx_line, "CANCEL_POWER_OFF") == 0
            || strcmp(rx_line, "CONFIRM_START_SUPPORTED") == 0
            || strcmp(rx_line, "CONFIRM_POWER_OFF_SUPPORTED") == 0)
        Jetson_ClearQueue_Unlocked();

    if (strcmp(rx_line, "WHEEL_STOP") == 0
            && (mode == MODE_WHEEL || mode == MODE_RAMP)) {
        stop_request = JETSON_STOP_WHEEL;
    } else if (strcmp(rx_line, "LEG_STOP") == 0
            && (mode == MODE_LEG || mode == MODE_WALK_DEMO)) {
        stop_request = JETSON_STOP_LEG;
    }
    if (stop_request != JETSON_STOP_NONE) {
        ++jetson_motion_epoch;
        jetson_stop_pending = stop_request;
        return;
    }
    if (jetson_queue_count == JETSON_COMMAND_QUEUE_CAPACITY) {
        ++jetson_rx_queue_overflow_count;
        ++jetson_motion_epoch;
        jetson_stop_pending = JETSON_STOP_SAFETY;
        return; // 대기열 초과 시 주행 정지
    }
    Jetson_Queued_Command_t *entry = &jetson_command_queue[jetson_queue_tail];
    memcpy(entry->text, rx_line, (size_t)rx_index + 1U);
    entry->motion_epoch = jetson_motion_epoch;
    jetson_queue_tail = (uint8_t)((jetson_queue_tail + 1U)
            % JETSON_COMMAND_QUEUE_CAPACITY);
    ++jetson_queue_count;
}

// 실행할 명령 1개 조회, 정지 명령 우선 처리
static uint8_t Jetson_TakeCommand(char *command, uint32_t *epoch,
        uint8_t *safety_stop)
{
    const RobotMode_t mode = Robot_Get_Mode();
    const uint32_t irq_mask = __get_PRIMASK();
    uint8_t available = 0U;
    *safety_stop = 0U;
    __disable_irq();
    Jetson_SyncMode_Unlocked(mode);
    if (mode == MODE_TO_WHEEL || mode == MODE_TO_LEG) {
        Jetson_DiscardPending_Unlocked();
    } else if (jetson_stop_pending) {
        if (jetson_stop_pending == JETSON_STOP_WHEEL) {
            memcpy(command, "WHEEL_STOP", sizeof("WHEEL_STOP"));
        } else if (jetson_stop_pending == JETSON_STOP_LEG) {
            memcpy(command, "LEG_STOP", sizeof("LEG_STOP"));
        } else {
            command[0] = '\0';
            *safety_stop = 1U;
        }
        jetson_stop_pending = JETSON_STOP_NONE;
        *epoch = jetson_motion_epoch;
        available = 1U;
    } else if (!jetson_rx_recovery_pending && jetson_queue_count != 0U) {
        const Jetson_Queued_Command_t *entry = &jetson_command_queue[jetson_queue_head];
        // 수신 명령을 별도 공간에 복사
        memcpy(command, entry->text, sizeof(rx_line));
        *epoch = entry->motion_epoch;
        jetson_queue_head = (uint8_t)((jetson_queue_head + 1U)
                % JETSON_COMMAND_QUEUE_CAPACITY);
        --jetson_queue_count;
        available = 1U;
    }
    __set_PRIMASK(irq_mask);
    return available;
}

uint8_t Jetson_Comm_HandleUartError(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart != jetson_huart) return 0U;
    ++jetson_rx_error_count;
    jetson_rx_recovery_pending = 1U;
    jetson_rx_discard_to_eol = 1U;
    rx_index = 0U;
    Jetson_ClearQueue_Unlocked(); // 기존 정지 요청 유지
    return 1U;
}

static void Jetson_ArmReceive(void)
{
    if (HAL_UART_Receive_IT(jetson_huart, &rx_byte, 1U) != HAL_OK) {
        ++jetson_rx_rearm_failure_count;
        (void)Jetson_Comm_HandleUartError(jetson_huart);
    }
}

static void Jetson_RecoverReceive_Task(void)
{
    const uint32_t now = HAL_GetTick();
    if (jetson_huart == NULL || !jetson_rx_recovery_pending
            || (uint32_t)(now - jetson_rx_last_retry_ms)
                    < JETSON_RX_RECOVERY_RETRY_MS) return;
    jetson_rx_last_retry_ms = now;

    // 송신·모터 통신 유지, 수신만 중단 후 재시작
    if (HAL_UART_AbortReceive(jetson_huart) != HAL_OK) return;
    rx_index = 0U;
    jetson_rx_discard_to_eol = 1U;
    // 수신 재시작 전 복구 플래그 초기화
    jetson_rx_recovery_pending = 0U;
    Jetson_ArmReceive();
}

// 12개 MX 모터와 IMU 상태 바이너리 패킷 규격
#define JETSON_MOTOR_STATE_BIN_HEADER_0		0x57U
#define JETSON_MOTOR_STATE_BIN_HEADER_1		0x4BU
#define JETSON_MOTOR_STATE_BIN_VERSION		0x02U
#define JETSON_MOTOR_STATE_BIN_TYPE		0x01U
#define JETSON_MOTOR_STATE_BIN_HEADER_SIZE	16U
#define JETSON_MOTOR_STATE_BIN_DATA_SIZE	10U
#define JETSON_MOTOR_STATE_BIN_IMU_META_SIZE	9U
#define JETSON_MOTOR_STATE_BIN_QUATERNION_SIZE	16U
#define JETSON_MOTOR_STATE_BIN_GYRO_SIZE	12U
#define JETSON_MOTOR_STATE_BIN_IMU_SIZE		\
		(JETSON_MOTOR_STATE_BIN_IMU_META_SIZE \
		+ JETSON_MOTOR_STATE_BIN_QUATERNION_SIZE \
		+ JETSON_MOTOR_STATE_BIN_GYRO_SIZE)
#define JETSON_MOTOR_STATE_BIN_CRC_SIZE		2U

// IMU 데이터 유효성 비트
#define JETSON_IMU_VALID_DATA			0x01U
#define JETSON_IMU_VALID_QUATERNION		0x02U
#define JETSON_IMU_VALID_GYRO			0x04U

#define JETSON_MOTOR_STATE_BIN_PACKET_SIZE	\
		(JETSON_MOTOR_STATE_BIN_HEADER_SIZE \
		+ (DXL_LEG_MOTOR_COUNT * JETSON_MOTOR_STATE_BIN_DATA_SIZE) \
		+ JETSON_MOTOR_STATE_BIN_IMU_SIZE \
		+ JETSON_MOTOR_STATE_BIN_CRC_SIZE)

// 모터 상태 스트리밍 주기
#define JETSON_MOTOR_STATE_STREAM_PERIOD_MS	10U
// 12개 다리 모터 설정 조회 규격
#define JETSON_ACTUATOR_CONFIG_MOTOR_COUNT	12U
#define JETSON_ACTUATOR_CONFIG_REGISTER_COUNT	15U
#define JETSON_ACTUATOR_CONFIG_EXPECTED_READS	\
	(JETSON_ACTUATOR_CONFIG_MOTOR_COUNT * JETSON_ACTUATOR_CONFIG_REGISTER_COUNT)

typedef struct {
	uint8_t id;
	const char *joint_role;
} Jetson_Actuator_Config_Motor_t;

typedef struct {
	const char *name;
	uint16_t address;
	uint8_t byte_width;
	uint8_t is_signed;
	const char *raw_type;
} Jetson_Actuator_Config_Register_t;

static const Jetson_Actuator_Config_Motor_t jetson_actuator_config_motors[
		JETSON_ACTUATOR_CONFIG_MOTOR_COUNT] = {
	{ 1U, "FL_HIP_ROLL" }, { 2U, "FL_HIP_PITCH" }, { 3U, "FL_KNEE" },
	{ 11U, "FR_HIP_ROLL" }, { 12U, "FR_HIP_PITCH" }, { 13U, "FR_KNEE" },
	{ 21U, "RL_HIP_ROLL" }, { 22U, "RL_HIP_PITCH" }, { 23U, "RL_KNEE" },
	{ 31U, "RR_HIP_ROLL" }, { 32U, "RR_HIP_PITCH" }, { 33U, "RR_KNEE" }
};

// MX 모터 설정 주소·바이트 수·부호 정보
static const Jetson_Actuator_Config_Register_t jetson_actuator_config_registers[
		JETSON_ACTUATOR_CONFIG_REGISTER_COUNT] = {
	{ "OPERATING_MODE", DXL_2_Operating_Mode, 1U, 0U, "U8" },
	{ "TEMPERATURE_LIMIT", DXL_2_Temperature_Limit, 1U, 0U, "U8" },
	{ "PWM_LIMIT", DXL_2_PWM_Limit, 2U, 0U, "U16" },
	{ "CURRENT_LIMIT", DXL_2_Current_Limit, 2U, 0U, "U16" },
	{ "VELOCITY_LIMIT", DXL_2_Velocity_Limit, 4U, 0U, "U32" },
	{ "MAX_POSITION_LIMIT", DXL_2_Max_Position_Limit, 4U, 0U, "U32" },
	{ "MIN_POSITION_LIMIT", DXL_2_Min_Position_Limit, 4U, 0U, "U32" },
	{ "SHUTDOWN", DXL_2_Shutdown, 1U, 0U, "U8" },
	{ "POSITION_D_GAIN", DXL_2_Position_D_Gain, 2U, 0U, "U16" },
	{ "POSITION_I_GAIN", DXL_2_Position_I_Gain, 2U, 0U, "U16" },
	{ "POSITION_P_GAIN", DXL_2_Position_P_Gain, 2U, 0U, "U16" },
	{ "FEEDFORWARD_2ND_GAIN", DXL_2_Feedforward_2nd_Gain, 2U, 0U, "U16" },
	{ "FEEDFORWARD_1ST_GAIN", DXL_2_Feedforward_1st_Gain, 2U, 0U, "U16" },
	{ "BUS_WATCHDOG", DXL_2_Bus_Watchdog, 1U, 1U, "I8" },
	{ "GOAL_PWM", DXL_2_Goal_PWM, 2U, 1U, "I16" }
};


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

// 모터 상태 스트리밍 상태와 진단값
static volatile uint8_t jetson_motor_state_stream_enabled = 0U;
static uint32_t jetson_motor_state_stream_next_tick = 0U;
static volatile uint32_t jetson_motor_state_stream_packet_count = 0U;
static volatile uint32_t jetson_motor_state_stream_busy_count = 0U;
static volatile uint32_t jetson_motor_state_stream_missed_count = 0U;

// 문자열 송신 대기 및 재시도 설정
// 송신 중 상태만 재시도, 오류·시간 초과 시 재전송 제외
#define JETSON_TEXT_TX_TIMEOUT_MS 20U
static volatile uint32_t jetson_text_tx_busy_retry_count;
static volatile uint32_t jetson_text_tx_failure_count;
static volatile HAL_StatusTypeDef jetson_text_tx_last_status = HAL_OK;

// Jetson 문자열 송신
static uint8_t Jetson_Send_Text(const char *text)
{
    const uint32_t start = HAL_GetTick();
    HAL_StatusTypeDef status;
    do {
        status = HAL_UART_Transmit(jetson_huart, (uint8_t *)text,
                (uint16_t)strlen(text), JETSON_TEXT_TX_TIMEOUT_MS);
        jetson_text_tx_last_status = status;
        if (status == HAL_OK) return 1U;
        if (status != HAL_BUSY
                || (uint32_t)(HAL_GetTick() - start) >= JETSON_TEXT_TX_TIMEOUT_MS)
            break;
        ++jetson_text_tx_busy_retry_count;
        HAL_Delay(1U); // 수신·송신 완료 인터럽트 유지
    } while (1);
    ++jetson_text_tx_failure_count;
    return 0U;
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

// 32비트 float 값을 IEEE-754 Little Endian 순서로 저장
static void Jetson_Write_F32_LE(uint8_t *buffer, uint16_t *index,
		float value) {
	uint32_t raw_value = 0U;

	memcpy(&raw_value, &value, sizeof(raw_value));
	Jetson_Write_U32_LE(buffer, index, raw_value);
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

	if (!Jetson_Send_Text(tx_part)) return;

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

		if (!Jetson_Send_Text(tx_part)) return;
	}

	Jetson_Send_Text("\r\n");
}

// 12개 다리 모터와 IMU 전체 상태 바이너리 송신
static void Jetson_Send_All_Motor_State_Binary(void) {
	DXL_Leg_State_t state;
	IMU_Data_t imu_data;
	uint16_t tx_index = 0U;
	uint16_t crc;
	uint8_t imu_valid_flags;
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

	// 모터 상태와 함께 전송할 최신 IMU 데이터 저장
	imu_data = IMU_Get_Latest_Data();
	imu_valid_flags = 0U;

	if (imu_data.valid != 0U) {
		imu_valid_flags |= JETSON_IMU_VALID_DATA;
	}

	if (imu_data.quaternion_valid != 0U) {
		imu_valid_flags |= JETSON_IMU_VALID_QUATERNION;
	}

	if (imu_data.gyro_valid != 0U) {
		imu_valid_flags |= JETSON_IMU_VALID_GYRO;
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

	// IMU 수신 순번, 수신 시각과 유효성 저장
	Jetson_Write_U32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.sequence);
	Jetson_Write_U32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.timestamp_ms);
	jetson_motor_state_tx_packet[tx_index++] = imu_valid_flags;

	// Quaternion 순서: X, Y, Z, W
	Jetson_Write_F32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.quaternion_x);
	Jetson_Write_F32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.quaternion_y);
	Jetson_Write_F32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.quaternion_z);
	Jetson_Write_F32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.quaternion_w);

	// 센서 좌표계 각속도 순서: X, Y, Z (rad/s)
	Jetson_Write_F32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.gyro_x_rad_s);
	Jetson_Write_F32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.gyro_y_rad_s);
	Jetson_Write_F32_LE(jetson_motor_state_tx_packet, &tx_index,
			imu_data.gyro_z_rad_s);

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

// 모터 상태 100Hz 스트리밍 실행
static void Jetson_Motor_State_Stream_Task(void) {
	uint32_t current_tick;
	uint32_t elapsed_ms;
	uint32_t missed_periods;

	if (jetson_motor_state_stream_enabled == 0U) {
		return;
	}

	current_tick = HAL_GetTick();

	// 다음 송신 시각에 도달하지 않은 경우 대기
	if ((int32_t) (current_tick
			- jetson_motor_state_stream_next_tick) < 0) {
		return;
	}

	elapsed_ms = current_tick
			- jetson_motor_state_stream_next_tick;
	missed_periods = elapsed_ms
			/ JETSON_MOTOR_STATE_STREAM_PERIOD_MS;

	// 처리 지연으로 지나간 주기는 다시 몰아서 실행하지 않음
	if (missed_periods > 0U) {
		jetson_motor_state_stream_missed_count += missed_periods;
	}

	jetson_motor_state_stream_next_tick +=
			(missed_periods + 1U)
					* JETSON_MOTOR_STATE_STREAM_PERIOD_MS;

	Jetson_Send_All_Motor_State_Binary();

	if (jetson_motor_state_tx_status == HAL_OK) {
		jetson_motor_state_stream_packet_count++;
	} else if (jetson_motor_state_tx_status == HAL_BUSY) {
		jetson_motor_state_stream_busy_count++;
	}
}

// 진행 중인 모터 상태 DMA 송신 완료 대기
static uint8_t Jetson_Wait_Motor_State_Tx_Complete(
		uint32_t timeout_ms) {
	uint32_t start_tick = HAL_GetTick();

	while (jetson_motor_state_tx_busy != 0U) {
		if ((HAL_GetTick() - start_tick) >= timeout_ms) {
			return 0U;
		}
        HAL_Delay(1U);
	}

	return 1U;
}

// 모터 상태 스트리밍 시작
static void Jetson_Start_Motor_State_Stream(void) {
	if (!Jetson_Wait_Motor_State_Tx_Complete(10U)) {
		Jetson_Send_Text(
				"MOTOR_STATE_STREAM:START_FAILED,REASON=TX_BUSY\r\n");
		return;
	}

	jetson_motor_state_stream_packet_count = 0U;
	jetson_motor_state_stream_busy_count = 0U;
	jetson_motor_state_stream_missed_count = 0U;

	if (!Jetson_Send_Text(
			"MOTOR_STATE_STREAM:STARTED,PERIOD_MS=10\r\n")) return;

	jetson_motor_state_stream_next_tick =
			HAL_GetTick()
					+ JETSON_MOTOR_STATE_STREAM_PERIOD_MS;
	jetson_motor_state_stream_enabled = 1U;
}

// 모터 상태 스트리밍 정지
static void Jetson_Stop_Motor_State_Stream(void) {
	char tx_line[128];
	int length;

	jetson_motor_state_stream_enabled = 0U;

	if (!Jetson_Wait_Motor_State_Tx_Complete(10U)) {
		Jetson_Send_Text(
				"MOTOR_STATE_STREAM:STOP_FAILED,REASON=TX_BUSY\r\n");
		return;
	}

	length = snprintf(tx_line, sizeof(tx_line),
			"MOTOR_STATE_STREAM:STOPPED,"
			"PACKETS=%lu,BUSY=%lu,MISSED=%lu\r\n",
			(unsigned long) jetson_motor_state_stream_packet_count,
			(unsigned long) jetson_motor_state_stream_busy_count,
			(unsigned long) jetson_motor_state_stream_missed_count);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// 모터 조회 명령의 ID 변환 및 검사
static uint8_t Jetson_Parse_Motor_ID(const char *id_text,
		const char *response_name, uint8_t allow_broadcast, uint8_t *id) {
	char *end_ptr;
	unsigned long id_value = strtoul(id_text, &end_ptr, 10);
	uint8_t broadcast = allow_broadcast && id_value == 254U;

	if (id_text == end_ptr || *end_ptr != '\0'
			|| (!broadcast && (id_value > 252U
					|| !Jetson_Is_Leg_Motor_ID((uint8_t) id_value)))) {
		char tx_line[64];
		int length = snprintf(tx_line, sizeof(tx_line),
				"%s:VALID=0,REASON=ID\r\n", response_name);
		if (length > 0 && length < (int) sizeof(tx_line)) {
			Jetson_Send_Text(tx_line);
		}
		return 0U;
	}

	*id = (uint8_t) id_value;
	return 1U;
}

// 모터 조회 통신 실패 응답, 기존 명령별 형식 유지
static void Jetson_Send_Motor_Comm_Error(const char *response_name,
		uint8_t id, uint8_t fail_stage) {
	char tx_line[96];
	int length = snprintf(tx_line, sizeof(tx_line),
			"%s:ID=%u,VALID=0,REASON=COMM,STAGE=%u\r\n",
			response_name, (unsigned int) id, (unsigned int) fail_stage);

	if (length > 0 && length < (int) sizeof(tx_line)) {
		Jetson_Send_Text(tx_line);
	}
}

// 개별 모터 레지스터 조회 및 실패 처리
static uint8_t Jetson_Read_Motor_Register(uint8_t id, uint16_t address,
		uint16_t size, uint8_t *data, uint8_t *device_error,
		const char *response_name) {
	uint8_t fail_stage = DXL_READ_FAIL_NONE;

	if (!dxl_read_2_0(id, address, size, data, device_error, &fail_stage)) {
		Jetson_Send_Motor_Comm_Error(response_name, id, fail_stage);
		return 0U;
	}
	return 1U;
}

// 현재 모터 속도 송신
static void Jetson_Send_Motor_Velocity(const char *id_text) {
	uint8_t id;
	uint8_t velocity_data[4];
	uint8_t device_error = 0xFF;
	char tx_line[96];

	if (!Jetson_Parse_Motor_ID(id_text, "MOTOR_VEL", 0U, &id)) {
		return;
	}

	if (!Jetson_Read_Motor_Register(id, DXL_2_Present_Velocity, 4U,
			velocity_data, &device_error, "MOTOR_VEL")) {
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
	uint8_t id;
	uint8_t current_data[2];
	uint8_t device_error = 0xFF;
	char tx_line[128];

	if (!Jetson_Parse_Motor_ID(id_text, "MOTOR_CURRENT", 0U, &id)) {
		return;
	}

	if (!Jetson_Read_Motor_Register(id, DXL_2_Present_Current, 2U,
			current_data, &device_error, "MOTOR_CURRENT")) {
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
	uint8_t id;
	uint8_t hardware_error = 0U;
	uint8_t device_error = 0xFFU;
	char tx_line[96];

	if (!Jetson_Parse_Motor_ID(id_text, "MOTOR_ERROR", 0U, &id)) {
		return;
	}

	if (!Jetson_Read_Motor_Register(id, DXL_2_Hardware_Error_Status, 1U,
			&hardware_error, &device_error, "MOTOR_ERROR")) {
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
// 하위 바이트부터 32비트 정수로 결합
static uint32_t Jetson_Decode_Config_Raw(const uint8_t *data,
		uint8_t byte_width) {
	uint32_t raw_value = 0U;

	for (uint8_t i = 0U; i < byte_width; i++) {
		raw_value |= ((uint32_t) data[i]) << (8U * i);
	}

	return raw_value;
}

// 원본 값 유지 및 부호 있는 정수로 변환
static int32_t Jetson_Decode_Config_Signed(uint32_t raw_value,
		uint8_t byte_width) {
	if (byte_width == 1U) {
		return (int32_t) (int8_t) (raw_value & 0xFFU);
	}

	if (byte_width == 2U) {
		return (int32_t) (int16_t) (raw_value & 0xFFFFU);
	}

	return (int32_t) raw_value;
}

// 정지한 Wheel 자세에서 모터 설정 1회 조회
static void Jetson_Dump_Actuator_Config(void) {
	char tx_line[320];
	uint16_t total_valid_count = 0U;
	uint16_t total_error_count = 0U;

	// 사용자 정지 확인 필요, 코드에서는 모드만 확인
	// 정지 명령·토크 설정 변경 제외
	if (Robot_Get_Mode() != MODE_WHEEL) {
		Jetson_Send_Text(
				"ACTUATOR_CONFIG_REJECTED,reason=ROBOT_MODE_NOT_WHEEL,"
				"required_operator_state=WHEEL_STATIONARY\r\n");
		return;
	}

	// 상태 스트리밍 중 설정 조회 거부
	if (jetson_motor_state_stream_enabled != 0U) {
		Jetson_Send_Text(
				"ACTUATOR_CONFIG_REJECTED,reason=MOTOR_STATE_STREAM_ACTIVE,"
				"action=STOP_MOTOR_STATE_STREAM_FIRST\r\n");
		return;
	}

	(void) snprintf(tx_line, sizeof(tx_line),
			"ACTUATOR_CONFIG_BEGIN,schema_version=1,motor_count=%u,"
			"register_count=%u,total_expected_reads=%u,"
			"operator_state=WHEEL_STATIONARY_REQUIRED\r\n",
			(unsigned int) JETSON_ACTUATOR_CONFIG_MOTOR_COUNT,
			(unsigned int) JETSON_ACTUATOR_CONFIG_REGISTER_COUNT,
			(unsigned int) JETSON_ACTUATOR_CONFIG_EXPECTED_READS);
	if (!Jetson_Send_Text(tx_line)) return;

	for (uint8_t motor_index = 0U;
			motor_index < JETSON_ACTUATOR_CONFIG_MOTOR_COUNT; motor_index++) {
		const Jetson_Actuator_Config_Motor_t *motor =
				&jetson_actuator_config_motors[motor_index];
		uint16_t valid_mask = 0U;
		uint8_t valid_count = 0U;
		uint8_t error_count = 0U;

		for (uint8_t register_index = 0U;
				register_index < JETSON_ACTUATOR_CONFIG_REGISTER_COUNT;
				register_index++) {
			const Jetson_Actuator_Config_Register_t *reg =
					&jetson_actuator_config_registers[register_index];
			uint8_t raw_bytes[4] = { 0U, 0U, 0U, 0U };
			uint8_t device_error = 0xFFU;
			uint8_t fail_stage = DXL_READ_FAIL_NONE;
			uint32_t timestamp_ms = HAL_GetTick();
			uint8_t read_ok = dxl_read_2_0(motor->id, reg->address,
					reg->byte_width, raw_bytes, &device_error, &fail_stage);

			if (read_ok != 0U && device_error == 0U) {
				uint32_t raw_value = Jetson_Decode_Config_Raw(raw_bytes,
						reg->byte_width);
				int32_t decoded_value = reg->is_signed != 0U ?
						Jetson_Decode_Config_Signed(raw_value,
								reg->byte_width) : (int32_t) raw_value;

				valid_mask |= (uint16_t) (1U << register_index);
				valid_count++;
				total_valid_count++;

				(void) snprintf(tx_line, sizeof(tx_line),
						"ACTUATOR_CONFIG_ROW,timestamp_ms=%lu,motor_id=%u,"
						"joint_role=%s,register_index=%u,register_name=%s,"
						"address=%u,byte_width=%u,raw_type=%s,valid=1,"
						"raw_u32=%lu,decoded_s32=%ld,status_error=0,"
						"fail_stage=0\r\n",
						(unsigned long) timestamp_ms, (unsigned int) motor->id,
						motor->joint_role, (unsigned int) register_index,
						reg->name, (unsigned int) reg->address,
						(unsigned int) reg->byte_width, reg->raw_type,
						(unsigned long) raw_value, (long) decoded_value);
			} else {
				error_count++;
				total_error_count++;

				(void) snprintf(tx_line, sizeof(tx_line),
						"ACTUATOR_CONFIG_ROW,timestamp_ms=%lu,motor_id=%u,"
						"joint_role=%s,register_index=%u,register_name=%s,"
						"address=%u,byte_width=%u,raw_type=%s,valid=0,"
						"raw_u32=UNKNOWN,decoded_s32=UNKNOWN,status_error=%u,"
						"fail_stage=%u\r\n",
						(unsigned long) timestamp_ms, (unsigned int) motor->id,
						motor->joint_role, (unsigned int) register_index,
						reg->name, (unsigned int) reg->address,
						(unsigned int) reg->byte_width, reg->raw_type,
						(unsigned int) device_error,
						(unsigned int) fail_stage);
			}

			if (!Jetson_Send_Text(tx_line)) return;
		}

		(void) snprintf(tx_line, sizeof(tx_line),
				"ACTUATOR_CONFIG_MOTOR_SUMMARY,timestamp_ms=%lu,motor_id=%u,"
				"joint_role=%s,expected_count=%u,valid_count=%u,"
				"valid_mask=0x%04X,error_count=%u\r\n",
				(unsigned long) HAL_GetTick(), (unsigned int) motor->id,
				motor->joint_role,
				(unsigned int) JETSON_ACTUATOR_CONFIG_REGISTER_COUNT,
				(unsigned int) valid_count, (unsigned int) valid_mask,
				(unsigned int) error_count);
		if (!Jetson_Send_Text(tx_line)) return;
	}

	(void) snprintf(tx_line, sizeof(tx_line),
			"ACTUATOR_CONFIG_END,total_expected_reads=%u,total_valid_count=%u,"
			"total_error_count=%u,complete=%u\r\n",
			(unsigned int) JETSON_ACTUATOR_CONFIG_EXPECTED_READS,
			(unsigned int) total_valid_count,
			(unsigned int) total_error_count,
			(unsigned int) (total_valid_count
					== JETSON_ACTUATOR_CONFIG_EXPECTED_READS));
	if (!Jetson_Send_Text(tx_line)) return;
}


// 모터 Ping 결과 송신
static void Jetson_Send_Motor_Ping(const char *id_text) {
	uint8_t id;
	uint8_t device_error = 0xFF;
	uint8_t fail_stage = DXL_READ_FAIL_NONE;
	char tx_line[96];

	if (!Jetson_Parse_Motor_ID(id_text, "MOTOR_PING", 1U, &id)) {
		return;
	}

	if (!dxl_ping_2_0(id, &device_error, &fail_stage)) {
		Jetson_Send_Motor_Comm_Error("MOTOR_PING", id, fail_stage);
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
	uint8_t id;
	uint8_t position_data[4];
	uint8_t device_error = 0xFF;
	char tx_line[96];

	if (!Jetson_Parse_Motor_ID(id_text, "MOTOR", 0U, &id)) {
		return;
	}

	if (!Jetson_Read_Motor_Register(id, DXL_2_Present_Position, 4U,
			position_data, &device_error, "MOTOR")) {
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

// 외부 지지 확인을 뜻하는 명시적 설정 명령, 부팅·일반 WHEEL/LEG 명령과 별도
static void Jetson_Configure_Knee_Extended(void)
{
    DXL_Knee_Mode_Config_Result_t result;
    char response[256];
    if (Robot_Get_Mode() != MODE_NONE) {
        Jetson_Send_Text("KNEE_MODE_CONFIG:VALID=0,REASON=MODE_NOT_NONE\r\n");
        return;
    }
    if (jetson_motor_state_stream_enabled || jetson_motor_state_tx_busy) {
        Jetson_Send_Text("KNEE_MODE_CONFIG:VALID=0,REASON=STOP_STREAM_FIRST\r\n");
        return;
    }
    // 설정 중 들어온 다른 동작 명령은 종료 후 폐기
    Jetson_Comm_DiscardPendingCommands();
    const uint8_t valid = dxl_configure_knee_extended_mode(&result);
    Jetson_Comm_DiscardPendingCommands();
    const int length = snprintf(response, sizeof(response),
            "KNEE_MODE_CONFIG:VALID=%u,MODE=4,MASK=0x%X,TORQUE_OFF_MASK=0x%X,"
            "RESTORED_MASK=0x%X,FAILED_ID=%u,STAGE=%u,STATUS_ERR=%u,READ_STAGE=%u,STOPPED=%u\r\n",
            (unsigned int)valid, (unsigned int)result.mode_mask,
            (unsigned int)result.torque_off_mask, (unsigned int)result.restored_mask,
            (unsigned int)result.failed_id, (unsigned int)result.stage,
            (unsigned int)result.device_error, (unsigned int)result.read_fail_stage,
            (unsigned int)result.stopped);
    if (length > 0 && length < (int)sizeof(response)) Jetson_Send_Text(response);
}

// Jetson 통신 초기화
void Jetson_Comm_Init(UART_HandleTypeDef *huart) {
	jetson_huart = huart;
	rx_index = 0;
    jetson_queue_head = jetson_queue_tail = jetson_queue_count = 0U;
    jetson_stop_pending = 0U;
    jetson_motion_epoch = 0U;
    jetson_rx_queue_overflow_count = jetson_rx_invalid_line_count = 0U;
    jetson_rx_stale_motion_count = 0U;
    jetson_text_tx_busy_retry_count = jetson_text_tx_failure_count = 0U;
    jetson_text_tx_last_status = HAL_OK;
    jetson_observed_mode = Robot_Get_Mode();
	jetson_rx_recovery_pending = jetson_rx_discard_to_eol = 0U;
	jetson_rx_error_count = jetson_rx_rearm_failure_count = 0U;
	jetson_rx_last_retry_ms = HAL_GetTick() - JETSON_RX_RECOVERY_RETRY_MS;

	jetson_motor_state_stream_enabled = 0U;
	jetson_motor_state_stream_next_tick = 0U;
	jetson_motor_state_stream_packet_count = 0U;
	jetson_motor_state_stream_busy_count = 0U;
	jetson_motor_state_stream_missed_count = 0U;

	// 통신 소요시간 측정을 위한 CPU Cycle Counter 활성화
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55UL;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	Jetson_ArmReceive();
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

		// 수신 오류가 있는 데이터 처리 제외
		if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
			(void)Jetson_Comm_HandleUartError(huart);
			return;
		}
		if (jetson_rx_recovery_pending) return;

        Jetson_SyncMode_Unlocked(current_mode);
		// 모드 전환 중 수신된 Jetson 명령 폐기
		if (current_mode == MODE_TO_WHEEL || current_mode == MODE_TO_LEG) {
            Jetson_DiscardPending_Unlocked();
			jetson_rx_discard_to_eol = (rx_byte != '\r' && rx_byte != '\n');

			Jetson_ArmReceive();
			return;
		}

		// 손상되거나 전환에 걸친 명령을 줄 끝까지 폐기
		if (jetson_rx_discard_to_eol) {
			if (rx_byte == '\r' || rx_byte == '\n') jetson_rx_discard_to_eol = 0U;
			Jetson_ArmReceive();
			return;
		}

        if (rx_byte == '\n' || rx_byte == '\r') {
            rx_line[rx_index] = '\0';
            if (rx_index != 0U) Jetson_QueueLine_FromISR();
            rx_index = 0U;
        } else if (rx_byte < 0x20U || rx_byte > 0x7EU
                || rx_index >= sizeof(rx_line) - 1U) {
            ++jetson_rx_invalid_line_count;
            rx_index = 0U;
            jetson_rx_discard_to_eol = 1U; // 비정상 명령의 나머지 데이터 폐기
        } else {
            rx_line[rx_index++] = (char)rx_byte;
        }

		Jetson_ArmReceive();
	}
}

// Jetson 명령 처리
static void Jetson_Send_Power_State(uint8_t accepted)
{
    static const char *const names[] = {
        "LOCKED", "CHECKING", "READY", "PARKING", "PARKED", "SAFE_OFF", "FAULT"
    };
    char response[240];
    const uint32_t phase = (uint32_t)robot_power_status.state;
    const int length = snprintf(response, sizeof(response),
            "POWER:STATE=%s,ACCEPTED=%u,ERROR=%u,ID=%u,POSE_CONFIG=%u,PATH_CONFIG=%u,"
            "SAFE_TO_POWER_OFF=%u,HOLD_SENT=%u\r\n",
            phase < 7U ? names[phase] : "INVALID", (unsigned)accepted,
            (unsigned)robot_power_status.error, (unsigned)robot_power_status.failed_id,
            (unsigned)WOKI_POWER_PARK_POSE_VERIFIED,
            (unsigned)WOKI_POWER_PARK_PATH_VERIFIED,
            (unsigned)RobotPower_SafeToPowerOff(), (unsigned)robot_power_status.hold_sent);
    if (length > 0 && length < (int)sizeof(response)) Jetson_Send_Text(response);
}

void Jetson_Comm_Task(void) {
    char command[sizeof(rx_line)];
    uint32_t epoch = 0U;
    uint8_t safety_stop = 0U;

    Jetson_RecoverReceive_Task();
    if (!Jetson_TakeCommand(command, &epoch, &safety_stop)) {
        if (!jetson_rx_recovery_pending) Jetson_Motor_State_Stream_Task();
        return;
    }
    if (safety_stop) {
        // 기존 주행 대기열 오류의 정지 동작 유지, 주차 중에만 전원 절차도 취소
        if (RobotPower_IsParking() || robot_power_status.state == ROBOT_POWER_PARKED
                || robot_power_status.state == ROBOT_POWER_SAFE_OFF) RobotPower_Cancel();
        Robot_Stop_Walk_Demo();
        Robot_Wheel_Stop();
        return;
    }
    if (jetson_rx_recovery_pending) return;
    if (Jetson_Is_Motion_Command(command) && epoch != jetson_motion_epoch) {
        ++jetson_rx_stale_motion_count;
        return; // 정지·대기열 초과 이전의 주행 명령 폐기
    }
    const RobotMode_t current_mode = Robot_Get_Mode();

    // 기준 확인 전·주차 중에는 주행 명령 차단, 조회·정지 명령은 유지
    const uint8_t stop_command = strcmp(command, "WHEEL_STOP") == 0
            || strcmp(command, "LEG_STOP") == 0;
    if (Jetson_Is_Motion_Command(command) && !stop_command
            && !RobotPower_MotionAllowed()) {
        Jetson_Send_Power_State(0U);
        return;
    }
    if (stop_command && !RobotPower_MotionAllowed()) {
        RobotPower_Cancel();
        Jetson_Comm_DiscardPendingCommands();
        Jetson_Send_Power_State(1U);
        return;
    }

	if (strcmp(command, "PING") == 0) {
		Jetson_Send_Text("OK\r\n");
	} else if (strcmp(command, "GET_POWER_STATE") == 0) {
		Jetson_Send_Power_State(1U);
	} else if (strcmp(command, "CONFIRM_START_SUPPORTED") == 0
			|| strcmp(command, "PREPARE_POWER_OFF") == 0
			|| strcmp(command, "CONFIRM_POWER_OFF_SUPPORTED") == 0
			|| strcmp(command, "CANCEL_POWER_OFF") == 0) {
		uint8_t accepted = 0U;
		Jetson_Comm_DiscardPendingCommands();
		if (strcmp(command, "CONFIRM_START_SUPPORTED") == 0)
			accepted = RobotPower_ConfirmStartSupported();
		else if (strcmp(command, "PREPARE_POWER_OFF") == 0)
			accepted = RobotPower_PreparePowerOff();
		else if (strcmp(command, "CONFIRM_POWER_OFF_SUPPORTED") == 0)
			accepted = RobotPower_ConfirmPowerOffSupported();
		else { RobotPower_Cancel(); accepted = 1U; }
		Jetson_Comm_DiscardPendingCommands();
		Jetson_Send_Power_State(accepted);
	} else if (strcmp(command, "GET_IMU") == 0) {
		Jetson_Send_IMU();
	} else if (strcmp(command,
			"START_MOTOR_STATE_STREAM") == 0) {
		Jetson_Start_Motor_State_Stream();
	} else if (strcmp(command,
			"STOP_MOTOR_STATE_STREAM") == 0) {
		Jetson_Stop_Motor_State_Stream();
	} else if (strcmp(command, "GET_MOTOR_STATE_BIN") == 0) {
        if (Jetson_Wait_Motor_State_Tx_Complete(10U)) {
            Jetson_Send_All_Motor_State_Binary();
        } else {
            Jetson_Send_Text("MOTOR_STATE_BIN:VALID=0,REASON=TX_BUSY\r\n");
        }
	} else if (strcmp(command, "GET_MOTOR_STATE_TIMING") == 0) {
		Jetson_Send_Motor_State_Timing();
	} else if (strcmp(command, "GET_MOTOR_STATE_ALL") == 0) {
		Jetson_Send_All_Motor_State();
	} else if (strcmp(command, "DUMP_ACTUATOR_CONFIG") == 0) {
		Jetson_Dump_Actuator_Config();
	} else if (strcmp(command, "SET_KNEE_EXTENDED_SUPPORTED") == 0) {
		if (current_mode != MODE_NONE || RobotPower_IsParking()
				|| robot_power_status.state == ROBOT_POWER_PARKED
				|| robot_power_status.state == ROBOT_POWER_SAFE_OFF) {
			Jetson_Send_Power_State(0U);
		} else {
			RobotPower_InvalidateReference();
			Jetson_Comm_DiscardPendingCommands();
			Jetson_Configure_Knee_Extended();
		}
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

	} else if (strcmp(command, "WHEEL") == 0) {
		Robot_Set_Wheel_Mode();
	} else if (strcmp(command, "RAMP") == 0) {
        if (current_mode == MODE_WHEEL) Robot_Set_Ramp_Mode();
	} else if (strcmp(command, "LEG") == 0) {
		if (current_mode == MODE_NONE)
			Jetson_Send_Text("POWER:ACCEPTED=0,REASON=ENTER_WHEEL_FIRST\r\n");
		else Robot_Set_Leg_Mode();
	} else if (strcmp(command, "WHEEL_FWD") == 0) {
        if (current_mode == MODE_WHEEL || current_mode == MODE_RAMP) {
            Robot_Wheel_Forward();
        }
	} else if (strcmp(command, "WHEEL_BACK") == 0) {
        if (current_mode == MODE_WHEEL || current_mode == MODE_RAMP) {
            Robot_Wheel_Backward();
        }
	} else if (strcmp(command, "WHEEL_TL") == 0) {
        if (current_mode == MODE_WHEEL) Robot_Tank_Left();
        else if (current_mode == MODE_RAMP) Robot_Wheel_Stop();
	} else if (strcmp(command, "WHEEL_TR") == 0) {
        if (current_mode == MODE_WHEEL) Robot_Tank_Right();
        else if (current_mode == MODE_RAMP) Robot_Wheel_Stop();
	} else if (strcmp(command, "WHEEL_STOP") == 0) {
        if (current_mode == MODE_WHEEL || current_mode == MODE_RAMP) {
            Robot_Wheel_Stop();
        }
	} else if (strcmp(command, "LEG_FWD") == 0) {
        if (current_mode == MODE_LEG) (void)Robot_Start_Walk_Demo();
	} else if (strcmp(command, "LEG_BACK") == 0
            || strcmp(command, "LEG_LEFT") == 0
            || strcmp(command, "LEG_RIGHT") == 0) {
        // 보행 함수 구현 후 연결
	} else if (strcmp(command, "LEG_STOP") == 0) {
        if (current_mode == MODE_LEG || current_mode == MODE_WALK_DEMO) {
            Robot_Stop_Walk_Demo();
        }
	}

    // 일반 명령이 계속 들어와도 예정된 100Hz 상태 송신을 함께 처리
    if (!jetson_rx_recovery_pending) Jetson_Motor_State_Stream_Task();
}
