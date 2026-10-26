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
#include "woki_power_guard.h"
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
	// 바퀴 정지 후 시작 기준 검사, 통과했을 때만 초기 Wheel 자세로 이동
	Robot_Wheel_Stop();
	// 전원 ON 전 Knee 네 개의 실제 누적 위치를 0~4095 구간으로 맞춘 상태에서 시작
	// 검사 통과 후 현재 위치에서 저속으로 Wheel에 직접 이동, 실제 도착 후 모드 전환
	if (RobotPower_CheckBootStart()) Robot_Set_Wheel_Mode();
	Jetson_Comm_DiscardPendingCommands(); // 초기 이동 중 대기한 동작 명령은 재사용하지 않음
	mode_button_request = 0U; // 초기 Wheel 이동 중 입력은 다음 전환으로 넘기지 않음
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		IMU_Task();
		RobotPower_Task(); // 기준 잠금·종료 준비 확인, ISR에서는 모터 통신 없음
		Jetson_Comm_Task();
		Robot_Ramp_Control_Task();
		Robot_Walk_Demo_Task();
		Robot_Wheel_Drive_Task();

		if (mode_button_request) {
			mode_button_request = 0;
			Jetson_Comm_DiscardPendingCommands(); // 버튼 동작 전 대기 중인 명령 폐기
			if (!RobotPower_MotionAllowed()) {
				// 잠금 원인은 보존, 주차 중에만 중단 요청으로 처리
				if (RobotPower_IsParking()) RobotPower_Cancel();
				continue;
			}

			if (Robot_Get_Mode() == MODE_WHEEL) {
				// 첫 번째 입력: Wheel → Leg
				Robot_Set_Leg_Mode();
			} else if (Robot_Get_Mode() == MODE_LEG) {
				// 두 번째 입력: Leg → Wheel, 이후 같은 순서로 반복
				Robot_Set_Wheel_Mode();
			} else if (Robot_Get_Mode() == MODE_WALK_DEMO) {
				// 보행 중 입력: 현재 자세 유지
				Robot_Stop_Walk_Demo();
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


		// 주차 중 버튼 입력은 정지 요청만 저장, 위치 유지 시도는 메인에서 수행
		if (RobotPower_IsParking()) {
			if ((current_tick - last_button_tick) >= 50U) {
				user_stop_flag = 1U;
				last_button_tick = current_tick;
			}
			return;
		}

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
