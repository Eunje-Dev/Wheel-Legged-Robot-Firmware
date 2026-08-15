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
#include "dxl_2_0.h"    // 다이나믹셀 모터 통합 제어 드라이버
#include "imu_driver.h"
#include "robot_motion.h"
#include "jetson_comm.h"
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
IMU_Data_t g_imu_data;               // 최신 IMU 데이터
volatile uint8_t user_stop_flag = 0;   // USER 버튼 눌림 여부 (1이면 모터 정지)
volatile uint8_t mode_button_request = 0;
static uint8_t user_button_step = 0;	// USER 버튼 시험 동작 순서
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void IMU_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	IMU_Init(&huart2);      // IMU UART 핸들 등록
	if (IMU_Start_DMA() != HAL_OK) { // DMA 수신 시작
		Error_Handler();
	}

	Robot_Motion_Init(); // 로봇 동작 상태 초기화
	Jetson_Comm_Init(&huart3); // Jetson VCP 통신 초기화

	uint8_t leg_ids[12];     // 12개 다리 모터 ID 배열
	uint8_t srl_values[12];  // 각 모터의 SRL 설정값 배열

	for (int i = 1; i <= 4; i++) {
		leg_ids[3 * (i - 1)] = legs[i].roll;
		srl_values[3 * (i - 1)] = 1;

		leg_ids[3 * (i - 1) + 1] = legs[i].pitch;
		srl_values[3 * (i - 1) + 1] = 1;

		leg_ids[3 * (i - 1) + 2] = legs[i].knee;
		srl_values[3 * (i - 1) + 2] = 1;
	}

	// 다리 모터 응답 레벨 설정 (SRL = 1)
	dxl_sync_write_1byte(leg_ids, srl_values, 12, DXL_2_Status_Return_Level);
	HAL_Delay(50);

	// 다리 모터 위치 제어 PID 및 Goal PWM 설정값
	const uint16_t position_d_gain = 0;
	const uint16_t position_i_gain = 0;
	const uint16_t position_p_gain = 850;
	const uint16_t goal_pwm = 885;

	// 12개 다리 모터의 위치 제어 PID 및 Goal PWM 설정
	for (int i = 0; i < 12; i++) {
		dxl_write_2_0(leg_ids[i], DXL_2_Position_D_Gain, 2,
				position_d_gain);

		dxl_write_2_0(leg_ids[i], DXL_2_Position_I_Gain, 2,
				position_i_gain);

		dxl_write_2_0(leg_ids[i], DXL_2_Position_P_Gain, 2,
				position_p_gain);

		dxl_write_2_0(leg_ids[i], DXL_2_Goal_PWM, 2, goal_pwm);
	}
	HAL_Delay(50);

	// 시작 시 Wheel Mode 자세 설정
	Robot_Set_Wheel_Mode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		IMU_Task();
		Jetson_Comm_Task();

		if (mode_button_request) {
			mode_button_request = 0;

			switch (user_button_step) {

			case 0:
				// 첫 번째 입력: Wheel → Leg
				Robot_Set_Leg_Mode();
				user_button_step = 1;
				break;

			case 1:
				// 두 번째 입력: Leg → Wheel
				Robot_Set_Wheel_Mode();
				user_button_step = 2;
				break;

			case 2:
				// 세 번째 입력: Wheel Mode 전진
				if (Robot_Get_Mode() == MODE_WHEEL) {
					Robot_Wheel_Forward();
					user_button_step = 3;
				}
				break;

			case 3:
				// 네 번째 입력: 바퀴 정지 후 처음 순서로 복귀
				Robot_Wheel_Stop();
				user_button_step = 0;
				break;

			default:
				Robot_Wheel_Stop();
				user_button_step = 0;
				break;
			}

			// 동작 중 발생한 추가 버튼 입력 제거
			mode_button_request = 0;
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 56;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 2048;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void IMU_Task(void) {
	IMU_Process_Data();

	IMU_Data_t data = IMU_Get_Data();
	if (data.updated) {
		g_imu_data = data;
	} else {
		g_imu_data.updated = 0;
	}

	// IMU 데이터 수신 중단 시 유효 상태 해제
	if (!data.valid) {
		g_imu_data.valid = 0;
	}
}

// USER 버튼 인터럽트 콜백
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static uint32_t last_button_tick = 0;

	if (GPIO_Pin == GPIO_PIN_13) {
		uint32_t current_tick = HAL_GetTick();

		// 버튼 채터링 방지
		if ((current_tick - last_button_tick) >= 300U) {
			mode_button_request = 1;
			last_button_tick = current_tick;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
