/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>       // sin, cos, acos 등 삼각함수 연산용
#include "dxl_2_0.h"    // 다이나믹셀 모터 통합 제어 드라이버
#include "imu_driver.h" // IMU 센서 데이터 수신 드라이버
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 다른 파일(usart.c)에 선언된 UART 장치 변수를 가져와서 사용함
extern UART_HandleTypeDef huart2; // IMU 센서용 UART
extern UART_HandleTypeDef huart3; // 모터 제어용 UART

// 다리 구조 실측 치수 (단위: mm)
float L1 = 170.0f; // 허벅지 길이 (고관절 중심부터 무릎 중심까지)
float L2 = 150.0f; // 종아리 길이 (무릎 중심부터 바퀴 축 중심까지)

// 모터 제어값 저장용 배열
uint32_t hip_goals[4] = { 2048, 2048, 2048, 2048 }; // 고관절 목표 위치
uint32_t knee_goals[4] = { 2048, 2048, 2048, 2048 }; // 무릎관절 목표 위치
int16_t wheel_speeds[4] = { 0, 0, 0, 0 };           // 바퀴 회전 속도

int toggle_state = 0; // 0: 일어서기 동작 수행, 1: 앉기(스쿼트) 동작 수행
// 디버깅 모니터링을 위해 전역 변수로 선언
IMU_Data_t imu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
// 역기구학 연산: 목표 높이(H)를 넣으면 모터가 움직여야 할 각도를 계산해줌
void calculate_leg_ik(float H, uint32_t *hip_out, uint32_t *knee_out) {
	// 코사인 법칙을 활용하여 관절 각도 도출
	float cos_knee = (H * H - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

	// 하드웨어 한계를 벗어나는 높이가 입력될 경우 값 고정
	if (cos_knee > 1.0f)
		cos_knee = 1.0f;
	if (cos_knee < -1.0f)
		cos_knee = -1.0f;

	float angle_knee_rad = acosf(cos_knee);
	float angle_hip_rad = asinf((L2 * sinf(angle_knee_rad)) / H);

	// 계산된 라디안 각도를 모터 제어 단위(0~4095)로 변환
	*hip_out = 2048 + (uint32_t) (angle_hip_rad * (4096.0f / (2.0f * M_PI)));
	*knee_out = 2048 - (uint32_t) (angle_knee_rad * (4096.0f / (2.0f * M_PI)));
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(1000);

	IMU_Init(&huart2);

	dxl_torque_set(1, 1, 1);
	HAL_Delay(1000);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		// 인터럽트 대신 여기서 파싱 수행
		IMU_Process_Data(); // sscanf를 안전하게 수행함

		// 1. 최신 IMU 데이터 읽기 (전역 변수에 저장)
		imu = IMU_Get_Data();

		float base_H = 250.0f; // 기준 높이 (mm)
		float compensation = imu.pitch * 2.0f; // 기울기에 따른 높이 보정값 (P제어 예시)

		// 2. 기울기에 맞춰 앞/뒤 다리 높이 차등 계산
		// 몸체가 앞으로 쏠리면 앞다리를 늘리고 뒷다리를 줄여 수평 유지
		float front_H = base_H + compensation;
		float rear_H = base_H - compensation;

		// 3. 역기구학 적용 (앞다리: 0, 1번 / 뒷다리: 2, 3번)
		for (int i = 0; i < 2; i++) {
			calculate_leg_ik(front_H, &hip_goals[i], &knee_goals[i]); // 앞다리 계산
		}
		for (int i = 2; i < 4; i++) {
			calculate_leg_ik(rear_H, &hip_goals[i], &knee_goals[i]);  // 뒷다리 계산
		}

		// 4. 계산된 각도와 휠 속도를 모터로 전송
		send_sync_write_2_joints(hip_goals, knee_goals);
		send_sync_write_1_wheel(wheel_speeds);

		HAL_Delay(20); // 50Hz 주기로 제어 루프 반복
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
