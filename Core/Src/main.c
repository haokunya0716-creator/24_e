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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_QD4310_PID.h"
#include "app_usart.h"
#include "button_test.h"
#include "QD4310.h"
#include "task.h"
#include "App_roboarm/app_roboarm.h"
#include "App_Task/app_task.h"
#include "Emm_V5/Emm_V5.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  MX_TIM3_Init();
  MX_CAN2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  App_Task_Init();
  //Delay_Test();GetUs也能正常用
  //Button_Test();按键也能正常用（没测试长按）
  //Buzzer_Test();蜂鸣器是可以响的！
  //机械臂的角度，和电机旋转角度的比是4.5:1
  //Robo_Set_YawDeg(-3);
  //Emm_V5_SetPositionRad(1, -3.1415926f, 200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {


    PERIODIC_START(MAIN,10)

      //Emm_V5_RequestPosition(1);
      //App_USART6_Printf("%f\n",Emm_V5_Position.angle_deg);
      Set_Big_Rad(0);
      Set_Small_Rad(0.6);
      App_USART6_Printf("%f,%f\n",SmallMotor.angle,BigMotor.angle);
    PERIODIC_END
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//初始化CAN配置
void CAN_InterfaceInit() {
    //CAN滤波器配置
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    //这里已经开启了滤波器配置
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();
    //这里已经开启了CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
    //这里已经开启了CAN的接收中断
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();

    //CAN滤波器配置
    CAN_FilterTypeDef sFilterConfig2;
    sFilterConfig2.FilterBank = 14;
    sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig2.FilterIdLow = 0x0000;
    sFilterConfig2.FilterIdHigh = 0x0000;
    sFilterConfig2.FilterMaskIdLow = 0x0000;
    sFilterConfig2.FilterMaskIdHigh = 0x0000;
    sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig2.FilterActivation = ENABLE;
    sFilterConfig2.SlaveStartFilterBank = 14;
    //这里已经开启了滤波器配置
    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2) != HAL_OK) Error_Handler();
    //这里已经开启了CAN
    if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();
    //这里已经开启了CAN的接收中断
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
}
//CAN接收回调函数
//CAN接收回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  // 统一在最上面定义，数组大小必须是 8！
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
    return;
  }

  // CAN1 处理 QD4310
  if (hcan == &hcan1) {
    // 确认是正确的ID
    if (rx_header.IDE == CAN_ID_STD) {
      if (rx_header.StdId >= 0x500 && rx_header.StdId <= 0x508) {
        if (rx_header.StdId == 0x500) {
          QD4310_Update(&SmallMotor, rx_data); // 更新云台yaw轴的状态
        } else if (rx_header.StdId == 0x501) {
          QD4310_Update(&BigMotor, rx_data); // 更新云台pitch轴的状态
        }
      }
    }
  }

  else if (hcan == &hcan2) {
    if (rx_header.IDE == CAN_ID_EXT) {
      Emm_V5_UpdatePosition(1, rx_header.ExtId, rx_data, rx_header.DLC);
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
  while (1)
  {
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
