
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "blance_common.h"
#include "string.h"
#include "math.h"

UART_HandleTypeDef huart1;

/* UART	printf */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

TIM_HandleTypeDef htim2;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void extreme_data_process(hal_double *asix_array, hal_double *data_result);


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
#define STEPMOTOR_X_DIR_MOVE_DOWN()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define STEPMOTOR_X_DIR_MOVE_UP()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

#define STEPMOTOR_X_ENABLE()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define STEPMOTOR_X_DISABLE()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)

#define STEPMOTOR_Y_DIR_MOVE_DOWN()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define STEPMOTOR_Y_DIR_MOVE_UP()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)

#define STEPMOTOR_Y_ENABLE()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define STEPMOTOR_Y_DISABLE()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)

/**
* @brief  The application entry point.
*/
int main(void)
{
	 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	 HAL_Init();

	 /* Configure the system clock */
	 SystemClock_Config();

	 /* Initialize all configured peripherals */
	 MX_GPIO_Init();
	 MX_TIM2_Init();
	 MX_USART1_UART_Init();

	 STEPMOTOR_X_DISABLE();
	 STEPMOTOR_Y_DISABLE();

	 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

	 adc_init();					// Init ADC peripheral

	 hal_double X_asix_temp = 0;			
	 hal_double Y_asix_temp = 0;
	 
	 hal_double X_Angle = 0;
	 hal_double	Y_Angle = 0;

	 hal_double x_angle_array[MAX_CAL_NUM] = {0};
	 hal_double y_angle_array[MAX_CAL_NUM] = {0};

	 hal_uint32 count = 0; 						// excute time counter
	 hal_uint32 i = 0;
	 
	 /* Infinite loop */
	 while (1)
	 {
	 	X_Angle = 0;
		Y_Angle = 0;

		for (i = 0;i < MAX_CAL_NUM;i++)
		{
			X_asix_temp = 0;
			Y_asix_temp = 0;
			angle_calculate(&X_asix_temp,&Y_asix_temp);		//	get x-asix,y-asix degree
			
#ifdef ANGLE_DEBUG
			printf("X-asis = [%.2lf]  Y-asis = [%.2lf]\r\n",X_asix_temp,Y_asix_temp);
#endif
			x_angle_array[i] = X_asix_temp;
			y_angle_array[i] = Y_asix_temp;
		}
		
	 	extreme_data_process(x_angle_array,&X_Angle);
	 	extreme_data_process(y_angle_array,&Y_Angle);
	 
	 	printf("Extreme data calculated down:");
	 	printf("X=[%.2lf]	 Y=[%.2lf]\r\n",X_Angle,Y_Angle);

		if (0 == count)
		{
			if (X_Angle > 0)							// 角度大于0，说明高于基准面
				STEPMOTOR_X_DIR_MOVE_DOWN();			// DIR控制位为1，滑块向下运动
			else 
				STEPMOTOR_X_DIR_MOVE_UP();
				

			if (Y_Angle > 0)
				STEPMOTOR_Y_DIR_MOVE_DOWN();			// 同上
			else				
				STEPMOTOR_Y_DIR_MOVE_UP();
			

			STEPMOTOR_X_ENABLE();						// EN=0 使能电机控制位
			STEPMOTOR_Y_ENABLE();
		}
			
		if ( X_Angle <= 0.25)							// 角度的绝对值小于0.25即认为平台处于水平位置
		{	
			STEPMOTOR_X_DISABLE();						// 系统的误差来源于： 1.平衡台的组装（电机柱不稳，泡沫底座未能水平）        
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		}												//					   2.倾角传感器的供电电压不稳定 
		if ( Y_Angle <= 0.25)							//					   3.电机的参数不明，只能根据经验调节速度和转动角度，A4988
		{
			STEPMOTOR_Y_DISABLE();						//驱动器采用32分频，尽量让电机最小角度转动
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);	
		}												//					   4.闭环控制 平台倾斜角度的输出和电机的反应存在时间差，导致有误差
	 	++count;
	 }

}

/* 去掉最高和最低的角度 计算平均值 数据尽量平稳 */
void extreme_data_process(hal_double *asix_array, hal_double *data_result)
{
	hal_double	angle_data[10] = {0};

	hal_double	max = 0;
	hal_double	min = 0;

	hal_uint32 	i = 0;

	hal_double	sum = 0;
	hal_double	result = 0;

	memcpy(angle_data,asix_array,sizeof(angle_data));
	
	/* max&min data calculate */
	max = angle_data[0];
	min = angle_data[0];
	for (i = 0;i < MAX_CAL_NUM;i++)
	{
		if (max < angle_data[i]) {
			max = angle_data[i];
		}

		if (min > angle_data[i]) {
			min = angle_data[i];
		}
	}

	/* average data calculate */
	for (i = 0;i<MAX_CAL_NUM;i++)
	{
		sum += angle_data[i];
	}

	result = (sum - max - min ) / DIVISOR;

	*data_result = result;

	return ;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (2000 - 1) >> 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_X_Pin|DIR_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_X_Pin|EN_Y_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DIR_X_Pin EN_X_Pin DIR_Y_Pin EN_Y_Pin */
  GPIO_InitStruct.Pin = DIR_X_Pin|EN_X_Pin|DIR_Y_Pin|EN_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
