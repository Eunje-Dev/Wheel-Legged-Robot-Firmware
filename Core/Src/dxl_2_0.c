/*
 * dxl_2_0.c
 *
 * Created on: 2026. 1. 12.
 * Author: User
 * Description: 다이나믹셀 통신 프로토콜 구현 및 모터 제어 함수
 */
#include "main.h"
#include "dxl_2_0.h"
#include "stdint.h"
#include "usart.h"
#include <stdbool.h>

volatile uint8_t uart_tx_busy = 0;

/*
 * [모터 ID 배치 설정]
 *
 * 1. FR (앞-오른쪽)
 * - Hip(1):   MX-106
 * - Knee(2):  MX-64
 * - Wheel(3): AX-12
 *
 * 2. FL (앞-왼쪽)
 * - Hip(11), Knee(12), Wheel(13)
 *
 * 3. RR (뒤-오른쪽)
 * - Hip(21), Knee(22), Wheel(23)
 *
 * 4. RL (뒤-왼쪽)
 * - Hip(31), Knee(32), Wheel(33)
 */

LegMotors legs[5] = {
		{0, 0, 0},         // 인덱스 0 미사용
		{1, 2, 3},         // FR
		{11, 12, 13},      // FL
		{21, 22, 23},      // RR
		{31, 32, 33}       // RL
};


// --- CRC & Checksum 계산 함수 ---
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size) {
    unsigned short i, j;
    unsigned short crc_table[256] = {
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++) {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

uint8_t calculate_checksum_1_0(uint8_t *data, uint16_t length) {
	uint8_t checksum = 0;
	for (uint16_t i = 2; i < length + 3; i++) checksum += data[i];
	return ~checksum;
}

// --- 패킷 생성 및 구조체 초기화 ---
dxl_sync_write_1_0 creat_sync_packet_1_0(void) {
	dxl_sync_write_1_0 sw1; sw1.header[0] = 0xFF; sw1.header[1] = 0XFF; sw1.id = 0xFE; sw1.inst = 0x83; return sw1;
}
dxl_sync_write_2_0 creat_sync_packet_2_0(void) {
	dxl_sync_write_2_0 sw2; sw2.header[0] = 0xFF; sw2.header[1] = 0xFF; sw2.header[2] = 0xFD; sw2.reserved = 0; sw2.id = 0xFE; sw2.inst = 0x83; return sw2;
}

void uart_transmit_packet(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
	if(uart_tx_busy == 1) return;
	if(huart->Instance == USART1) { HAL_UART_Transmit_DMA(huart, data, size); uart_tx_busy = 1; }
	else if(huart->Instance == USART2) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // UART2 (2.0)
		HAL_UART_Transmit_DMA(huart, data, size);
		uart_tx_busy = 1;
	}
	else if(huart->Instance == USART3) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // UART3 (1.0)
		HAL_UART_Transmit_DMA(huart, data, size);
		uart_tx_busy = 1;
	}
}

// --- 패킷 직렬화 (Serialization) ---
uint16_t serialize_sync_write_1_0(dxl_sync_write_1_0 *packet, uint8_t *id_array, uint8_t id_count, uint16_t *data_array, uint8_t *buffer, uint16_t buffer_size) {
    uint16_t index = 0;
    buffer[index++] = packet->header[0]; buffer[index++] = packet->header[1];
    buffer[index++] = packet->id; buffer[index++] = packet->length;
    buffer[index++] = packet->inst; buffer[index++] = packet->start_addr;
    buffer[index++] = packet->data_len;
    for (int i = 0; i < id_count; i++) {
        buffer[index++] = id_array[i];
        for (int j = 0; j < packet->data_len; j++) {
            buffer[index++] = (data_array[i] >> (8 * j)) & 0xFF;
        }
    }
    packet->checksum = calculate_checksum_1_0(buffer, index);
    buffer[index++] = packet->checksum;
    return index;
}

uint16_t serialize_sync_write_2_0(dxl_sync_write_2_0 *packet, uint8_t *id_array, uint8_t id_count, uint32_t *data_array, uint8_t *buffer, uint16_t buffer_size) {
    uint16_t index = 0;
    buffer[index++] = packet->header[0]; buffer[index++] = packet->header[1];
    buffer[index++] = packet->header[2]; buffer[index++] = packet->reserved;
    buffer[index++] = packet->id; buffer[index++] = packet->length & 0xFF;
    buffer[index++] = (packet->length >> 8) & 0xFF;
    buffer[index++] = packet->inst; buffer[index++] = packet->start_addr & 0xFF;
    buffer[index++] = (packet->start_addr >> 8) & 0xFF;
    buffer[index++] = packet->data_len & 0xFF; buffer[index++] = (packet->data_len >> 8) & 0xFF;
    for (int i = 0; i < id_count; i++) {
        buffer[index++] = id_array[i];
        for (int j = 0; j < packet->data_len; j++) {
            buffer[index++] = (data_array[i] >> (8 * j)) & 0xFF;
        }
    }
    unsigned short crc = update_crc(0, buffer, index);
    buffer[index++] = crc & 0x00FF; buffer[index++] = (crc >> 8) & 0x00FF;
    return index;
}

// Protocol 1.0 Sync Write 전송 (Wheel)
void send_sync_write_1(uint16_t addr, uint16_t data_len, uint16_t *wheel_data)
{
	uint8_t id_count = 4;
	uint8_t id[4];
	uint16_t data_array[4];
	dxl_sync_write_1_0 sw1 = creat_sync_packet_1_0();

	for(int i = 1; i <= 4; i++) {
		id[i-1] = legs[i].wheel;
		data_array[i-1] = clc_speed_1(wheel_data[i-1]);
	}

	sw1.start_addr = addr;
	sw1.data_len = data_len;
	sw1.length = 4 + (id_count * (sw1.data_len + 1));

	uint8_t packet_buffer[256] = {0,};
	if((8 + id_count * (sw1.data_len + 1)) <= sizeof(packet_buffer)) {
		uint16_t packet_size = serialize_sync_write_1_0(&sw1, id, id_count, data_array, packet_buffer, sizeof(packet_buffer));

		uint32_t timeout = HAL_GetTick();
		while(uart_tx_busy != 0) {
			if((HAL_GetTick() - timeout) > 10) { uart_tx_busy = 0; break; }
		}
		if(packet_size > 0) uart_transmit_packet(&huart3, packet_buffer, packet_size);
	}
}

// Protocol 2.0 Sync Write 전송 (Hip, Knee)
void send_sync_write_2(uint16_t addr, uint16_t data_len, uint32_t *hip_data, uint32_t *knee_data)
{
	uint8_t id_count = 8;
	uint8_t id[8];
	uint32_t data_array[8];
	dxl_sync_write_2_0 sw2 = creat_sync_packet_2_0();

	for(int i = 1; i <= 4; i++) {
		// Hip (MX-106)
		id[2*(i-1)] = legs[i].hip;
		data_array[2*(i-1)] = hip_data[i-1];

		// Knee/Link (MX-64)
		id[2*(i-1)+1] = legs[i].knee;
		data_array[2*(i-1)+1] = knee_data[i-1];
	}

	sw2.start_addr = addr;
	sw2.data_len = data_len;
	sw2.length = 7 + (id_count * (sw2.data_len + 1));

	uint8_t packet_buffer[256] = {0,};
	if((12 + id_count * (sw2.data_len + 1) + 2) <= sizeof(packet_buffer)) {
		uint16_t packet_size = serialize_sync_write_2_0(&sw2, id, id_count, data_array, packet_buffer, sizeof(packet_buffer));

		uint32_t timeout = HAL_GetTick();
		while(uart_tx_busy != 0) {
			if((HAL_GetTick() - timeout) > 10) { uart_tx_busy = 0; break; }
		}
		if(packet_size > 0) uart_transmit_packet(&huart2, packet_buffer, packet_size);
	}
}

// 토크 활성화 설정
void dxl_torque_set(uint8_t on_hip, uint8_t on_knee, uint8_t on_wheel)
{
	uint32_t hip_t[4] = {on_hip, on_hip, on_hip, on_hip};
	uint32_t knee_t[4] = {on_knee, on_knee, on_knee, on_knee};
	uint16_t wheel_t[4] = {on_wheel, on_wheel, on_wheel, on_wheel};

	// 2.0 Group (Hip, Knee)
	send_sync_write_2(DXL_2_Torque_Enable, 1, hip_t, knee_t);
	HAL_Delay(1);

	// 1.0 Group (Wheel)
	// 바퀴 모터(AX-12)는 Speed 모드에서도 Torque Enable 필요
	send_sync_write_1(DXL_1_Torque_Enable, 1, wheel_t);
	HAL_Delay(1);
}

// AX-12 속도 값 변환 (CCW/CW 처리)
uint16_t clc_speed_1(int16_t wheel_speed){
	if (wheel_speed < -1023 || wheel_speed > 1023) return 0;
	if (wheel_speed < 0) return (uint16_t)(-wheel_speed) + 1024;
	return (uint16_t)wheel_speed;
}

// UART 송신 완료 인터럽트 콜백
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) uart_tx_busy = 0;
    else if(huart->Instance == USART2) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); uart_tx_busy = 0; }
    else if(huart->Instance == USART3) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); uart_tx_busy = 0; }
}
