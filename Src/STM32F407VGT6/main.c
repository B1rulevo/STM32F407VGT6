/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint32_t Data;
uint16_t ADC_Data[2];
volatile uint8_t regime;
volatile uint16_t counter;
volatile uint8_t channel;
uint8_t fl_adc = 0;
uint8_t TIM_flag = 0;
volatile uint8_t Scale;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void PHASOMETER(void);
void CALIBRATION(void);
void Timer_Interrupt(void);
void ADC_DMA_TransferComplete_Callback(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  LL_DAC_Enable(DAC, LL_DAC_CHANNEL_2); // Enabling DAC
  LL_mDelay(10); // Delay to prevent bugs

  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    regime = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4); // Check for the regime of the operation

    if (LL_GPIO_IsInputPinSet(GPIOE, LL_GPIO_PIN_7) == 0) // Check for the scale of measured phase shifts
  	  Scale = 0;
    else if (LL_GPIO_IsInputPinSet(GPIOE, LL_GPIO_PIN_8) == 0)
  	  Scale = 1;
    else if (LL_GPIO_IsInputPinSet(GPIOE, LL_GPIO_PIN_9) == 0)
  	  Scale = 2;
    else if (LL_GPIO_IsInputPinSet(GPIOE, LL_GPIO_PIN_10) == 0)
  	  Scale = 3;
    else if (LL_GPIO_IsInputPinSet(GPIOE, LL_GPIO_PIN_11) == 0)
  	  Scale = 4;
    else
  	  Scale = 5;

    if (regime == 1)
  	  PHASOMETER();
    else
  	  CALIBRATION();
  /* USER CODE END 3 */
}

void ADC_DMA_TransferComplete_Callback() // DMA transfer interrupt callback
{
	ADC_Data[0] = (Data & 0x0fff0000) >> 16; // Getting from one 32-bit register two measured values from AD8302 outputs
	ADC_Data[1] = (Data & 0x00000fff);
	fl_adc = 1; // Raise the flag
}

void CALIBRATION() //Testing regime
{

	const uint16_t mult[6] = {16384, 8192, 4096, 2048, 1024, 512};
	uint16_t signal = 0;

	LL_TIM_EnableIT_UPDATE(TIM6);
	LL_TIM_EnableCounter(TIM6);

	while (1)
	{
		if (TIM_flag == 1)
		{
			signal = counter * mult[Scale];
			LL_DAC_ConvertData12RightAligned(DAC, LL_DAC_CHANNEL_2, (uint16_t)(signal/16));
			TIM_flag = 0;
		}
	}
}

void Timer_Interrupt(void) //Timer_INT for the testing regime
{
	TIM_flag = 1;
	counter++;
}

void PHASOMETER(void) //Phasemeter regime of operation
{
	const uint16_t Multiplier[6][4] = {{16384, 4096, 67, 20},
			{8192, 2048, 67, 40},
			{4096, 1024, 67, 80},
			{2048, 512, 67, 160},
			{1024, 256, 67, 320},
			{512, 128, 67, 640}}; // Define scale values
	const uint16_t upV = 2000; // Define threshold values
	const uint16_t dwV = 550; // Define threshold values


	uint16_t Uout = 0;
	int8_t zone;
	int16_t fi_ref, fi_curr;
	int8_t n;

	  /* Initialization of DMA because some driver bugs are possible otherwise */
	  LL_DMA_ConfigTransfer(DMA2,
	                          LL_DMA_CHANNEL_0,
	                          LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
	                          LL_DMA_MODE_CIRCULAR              |
	                          LL_DMA_PERIPH_NOINCREMENT         |
	                          LL_DMA_MEMORY_INCREMENT           |
	                          LL_DMA_PDATAALIGN_WORD        |
	                          LL_DMA_MDATAALIGN_WORD        |
	                          LL_DMA_PRIORITY_HIGH               );

	    LL_DMA_ConfigAddresses(DMA2,
	                           LL_DMA_CHANNEL_0,
	                           LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA_MULTI),
	                           (uint32_t)&Data,
	                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_0, 1);

	    LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_0);

	    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
	    /* Initialization of DMA because some driver bugs are possible otherwise */

	    LL_ADC_Enable(ADC1); // Enabling the main ADC
	    LL_ADC_Enable(ADC2); // Enabling the second ADC
	    LL_ADC_REG_StartConversionSWStart(ADC1); // ADC conversion start


	/* Determination the initial working zone */
	if (ADC_Data[0] > ADC_Data[1])
	{
	  if (ADC_Data[1] < dwV)
		  zone = 0;
	  else
		  zone = 1;
	}
	else
	{
	  if (ADC_Data[1] > upV)
		  zone = 2;
	  else
		  zone = 3;
	}
	/* Determination the initial working zone */

	/* Sign determination of the slope angle of the characteristic   */
	if ((zone / 2) == 0)
	  n = 1;
	else
	  n = -1;
	/* Sign determination of the slope angle of the characteristic   */

	channel = (zone % 2); // Determination of an AD8302 detector which is in the linear region

	fi_ref = (zone * Multiplier[Scale][1]) + n * (ADC_Data[channel] - 1229) * Multiplier[Scale][2] / Multiplier[Scale][3]; // Initial phase shift

	while (1) // Unconditional loop
	{
	  if (fl_adc != 0) // Wait for the conversion flag
	  {
		  /* Check for the working zone */
		  if (ADC_Data[channel] > upV)
		  {
			  zone = zone + n;
			  if (zone == 4)
			  {
				  zone = 0;
				  fi_ref = fi_ref - Multiplier[Scale][0];
			  }
			  else if (zone == -1)
			  {
				  zone = 3;
				  fi_ref = fi_ref + Multiplier[Scale][0];
			  }
			  if ((zone / 2) == 0)
				  n = 1;
			  else
				  n = -1;
			  channel = (zone % 2);
		  }
		  else if (ADC_Data[channel] < dwV)
		  {
			  zone = zone - n;
			  if (zone == 4)
			  {
				  zone = 0;
				  fi_ref = fi_ref - Multiplier[Scale][0];
			  }
			  else if (zone == -1)
			  {
				  zone = 3;
				  fi_ref = fi_ref + Multiplier[Scale][0];
			  }
			  if ((zone / 2) == 0)
				  n = 1;
			  else
				  n = -1;
			  channel = (zone % 2);
		  }
		  /* Check for the working zone */

		  fi_curr = (zone * Multiplier[Scale][1]) + n * (ADC_Data[channel] - 1229) * Multiplier[Scale][2] / Multiplier[Scale][3]; // Define the current phase shift compared to the initial time

		  if (Uout + (fi_curr - fi_ref) > 0) // Prevent overflow due to phase noise at the initial time
			  Uout = Uout + (fi_curr - fi_ref);

		  fi_ref = fi_curr; // Saving the current value

		  fl_adc = 0; // Clear the flag

		  LL_DAC_ConvertData12RightAligned(DAC, LL_DAC_CHANNEL_2, (uint16_t)(Uout/16)); // Transfer the calculated value to the DAC output
	  }
	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(168000000);
  LL_SetSystemCoreClock(168000000);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0-WKUP   ------> ADC1_IN0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_SEQUENCE_CONV);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV6;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;
  ADC_CommonInitStruct.MultiDMATransfer = LL_ADC_MULTI_REG_DMA_UNLMT_2;
  ADC_CommonInitStruct.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_20CYCLES;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_3CYCLES);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC2 GPIO Configuration
  PA2   ------> ADC2_IN2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC2, LL_ADC_REG_FLAG_EOC_SEQUENCE_CONV);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_3CYCLES);
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**DAC GPIO Configuration
  PA5   ------> DAC_OUT2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* DAC interrupt Init */
  NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC channel OUT2 config
  */
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_2, &DAC_InitStruct);
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* TIM6 interrupt Init */
  NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 20999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 499;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10
                          |LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
