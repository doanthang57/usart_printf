/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
__IO int16_t * buff_T; //  buff save counter 1cnt = 10^-4
__IO int16_t count_tmp = 0;
__IO int    save_index  = 0;
__IO int    send_index  = 0;
__IO int    buff_length = 0;
__IO int16_t flash_index = 0;
//__IO uint8_t first_measure_flag = 1;

__IO uint8_t run_flag         = 0;
__IO uint8_t error_flag         = 0;
__IO uint8_t ready_send_flag    = 0;
__IO uint8_t sending_flag       = 0;
__IO uint8_t measure_flag       = 0; // flag is 1 when system is measuring

float   time_log        = 0.0;  // Time tracking to save the Time of each Section, reset to 0 when buffer 's reseted
float   volume_ex       = 0.0;  // Volume logging of expiration
float   volume_ins      = 0.0;  // Volume logging of inspiration
float 	T_debug 		= 0.0;
float 	RPS_debug	=0.0;
float 	RPS_de	=0.0;

uint16_t RPS_send=0,flow_str,T_str;
uint8_t  buff_rec = 0;
uint8_t  check = 1;
uint8_t  check_of = 1;
volatile uint8_t R_data;

const float a1 = 0.2932;
const float a2 = 0.0844;
const float a3 = 0.0306;
const float a4 = 0.3542;

uint8_t cnt_out=0, cnt_in=0;
uint16_t buf_in[5]={0x14,0,0,0,0x13},buf_out[5]={0x15,0,0,0,0x13}, buf_Tin[5]={0x16,0,0,0,0x13}, buf_Tout[5]={0x17,0,0,0,0x13},buf_Start[3]={0x18,0xA5,0x13}, run=0x13;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
#ifndef INC_PRINTF_RETARGET_H_

int __io_putchar(int ch);
int _write(int file,char *ptr, int len);

#define INC_PRINTF_RETARGET_H_

#endif /* INC_PRINTF_RETARGET_H_ */

/* (Re)Define stdio functionality, so printf would output to USART1 */
int __io_putchar(int ch) {
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit(&huart2, &c[0], 1, 10);
 return ch;
}

int _write(int file,char *ptr, int len) {
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++) {
  __io_putchar(*ptr++);
 }
 return len;
}
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Receive_IT(&huart2, &R_data, 1);
//  while(!(R_data=='A'));
  LL_TIM_EnableCounter(TIM2);
  SET_BIT(TIM2->CCER,TIM_CCER_CC1E);
  SET_BIT(TIM2->DIER,TIM_DIER_CC1IE);
  //Start_Measure_Func();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  printf("Hello world");
	  LL_mDelay(10);
//	  while(check)
//	  	  	  {
//	  	  			while(!(R_data=='A'));
//	  	  				run_flag=1;
//	  	  				check = 0;
//	  	  				check_of = 1;
//	  	  				//USART_Write(USART2,&buf_Start[0],6);
//	  	  				HAL_UART_Transmit(&huart2, (uint8_t*)buf_Start, 6, 5);
//
//	  	  	  }
//	  	  	  	send_data();
//	  	  	  	if((TIM2->CNT>20000)&&(sending_flag==0))Reset_Buffer();
//	  	  	  	if((sending_flag==0)&&(R_data==0x69))
//	  	  	  	{
//	  	  	  			LL_TIM_DisableCounter(TIM2);
//	  	  	  			CLEAR_BIT(TIM2->CCER,TIM_CCER_CC1E);
//	  	  	  			CLEAR_BIT(TIM2->DIER,TIM_DIER_CC1IE);
//	  	  	  			check_of = 0;
//	  	  	  	}
//	  	  	  	else;
//	  	  	  	if((sending_flag==0)&&(R_data=='A'))
//	  	  	  	{
//	  	  	 		//USART_Write(USART2,&buf_Start[0],6);
//	  	  	  	  LL_TIM_EnableCounter(TIM2);
//	  	  	  	  SET_BIT(TIM2->CCER,TIM_CCER_CC1E);
//	  	  	  	  SET_BIT(TIM2->DIER,TIM_DIER_CC1IE);
//	  	  	  	}
//	  	  	  	if((check_of == 0)&&(R_data=='A'))
//	  	  	  		  	{
//	  	  	  				check = 1;
//	  	  	  		  	}
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();  
  };
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_SYSCLK);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(TIM2_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration  
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_4BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration  
  PA0   ------> TIM2_CH1 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 6400;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xffff;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14|LL_GPIO_PIN_15);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==huart2.Instance){
		HAL_UART_Receive_IT(&huart2, &R_data, 1);
	}

}

float Caculate_Flow_Func(float t, float t_prev)
{
  float flow = 0.0, flow_tmp = 0.0;
  float b2=0.0, b3=0.0, b4=0.0, b4_2=0.0, b4_3=0.0, b5=0.0, b6=0.0;
  // Calculate the Flow
  if ((t_prev != 0) && (t != 0))
  {
    b2 = a2/t;
    b3 = (float)a3/sqrt(t);
    flow_tmp = a1 + b2 + b3;
    b4 = t - t_prev;
    if (b4<0) b4 = 0 - b4;
    b4_2 = t  *t_prev;
    b4_3 = b4/b4_2;
    b5 = a4*b4_3;
    b6 = b5 + 1;
    flow = flow_tmp * b6;
  }
  else flow = 0;
  return flow;
}

//*** Caculate the Volume of Spirometer ***//
float Caculate_Volume_Func(float t, float flow)
{
  return (t*flow);
}

/**
 * @brief  Reset the Buffer store Data
 * @note
 * @retval None
 */
void Reset_Buffer(void)
{
  // Serial.println("[INF] Sent ALL Data, Clear Buffer");
  Clear_Buffer();
  // Allocate the new Memory to Buffer
  buff_length = Length_buffer;
  //first_measure_flag = 1;
  buff_T      = (int16_t *) malloc(buff_length * sizeof(int16_t));

  if (buff_T == 0)
  {
    error_flag = 1;
  }
  // Reset the Time Period Log
  time_log = 0.0;
//  round_total_out = 0.0;
//  round_total_in  = 0.0;

}

/**
 * @brief
 * @note
 * @retval None
 */
void Clear_Buffer(void)
{
  // Reset the Index
  send_index = 0;
  save_index = 0;
  // Release the Memory which is allocated before
  free(buff_T);

}

/**
 * @brief
 * @note
 * @retval None
 */
void Reset_All(void)
{
  // Reset all Parameter
 // first_measure_flag  = 1;
  error_flag          = 0;
  ready_send_flag     = 0;
  sending_flag        = 0;
  measure_flag        = 0;
  time_log            = 0.0;
  Clear_Buffer();
//  round_total_out = 0.0;
//  round_total_in  = 0.0;
}

void send_data(void)
{
  if ((error_flag == 0) && (ready_send_flag == 1))
  {
    sending_flag = 1;
    if (send_index < save_index)
    {
      // Ready to send the Data via BLE or UART
      // Get the Data from Buffer

//    	RPS_debug = (float) *(buff_T + send_index);
//    	RPS_debug=fabs(RPS_debug)/10000;
//    	RPS_de=(1/(2*RPS_debug));
//    	RPS_send=(uint16_t)RPS_de;

    	float T = (float) *(buff_T + send_index);
		T =  T / freq;	//  pulse cycles in second
		T = fabs(T);
		float T_period = T;
		T_debug=T_period;
      // Check the first value in buffer, Not send Data because don't have the T_prev in buffer
      // So can't Caculate the Flow tos send Data

      if (send_index  != 0)
      {

		float T_prev = (float) *(buff_T + send_index - 1);
		T_prev =  T_prev / freq;	//  pulse cycles in second
		T_prev = fabs(T_prev);
        // Convert from Pulse Cycles to Round Cycles
        T       = T * pulse_per_round;      // in seconds
        T_prev  = T_prev * pulse_per_round; // in seconds

        // Caculate the Flow
        float flow    = Caculate_Flow_Func(T, T_prev);
        // Caculate the Volume
        float volume_log  = Caculate_Volume_Func(T_period,flow);
        // convert float to string
        flow_str = (uint16_t)(flow*1000);
        T_str=(uint16_t)(T_prev*1000);
      // Send Data via BLE
        if (*(buff_T + send_index) > 0) // data out
        {
        	cnt_out++;
        	//buf_out[cnt_out]=flow_str;
        	buf_out[cnt_out]=RPS_send;
        	buf_Tout[cnt_out]=T_str;
        	if(cnt_out>=3){
        		HAL_UART_Transmit(&huart2, (uint8_t*)buf_out, 10, 10);
        		HAL_UART_Transmit(&huart2, (uint8_t*)buf_Tout, 10, 10);
//        		USART_Write(USART2, &buf_out , 10);
//        		USART_Write(USART2, &buf_Tout , 10);
        		cnt_out=0;
        		LL_mDelay(20);
        	}
        	else;
        // Expiration process
        // Adding volume_log to volume_ex
          volume_ex += volume_log;
        // Reset volume_in to Zero
          volume_ins = 0;
        }
        else // data in
        {
        	cnt_in++;
        	//buf_in[cnt_in]=flow_str;
        	buf_in[cnt_in]=RPS_send;
        	buf_Tin[cnt_in]=T_str;
        	if(cnt_in>=3){
//        		USART_Write(USART2, &buf_in , 10);
//        		USART_Write(USART2, &buf_Tin, 10);
        		HAL_UART_Transmit(&huart2, (uint8_t*)buf_in, 10, 10);
        		HAL_UART_Transmit(&huart2, (uint8_t*)buf_Tin, 10, 10);
        		cnt_in=0;
        		LL_mDelay(20);
        	}
        // Inspiration process
        // Adding volume_log to volume_in
            volume_ins += volume_log;
        // Reset volume_ex to Zero
            volume_ex = 0;
        //Send Data to BLE
        }
      }
      else;
      // Increase the Send Index
      send_index++;
    }
    // Already Sent the All Data in Buffer
    else {
    	sending_flag = 0;
    	//Reset_Buffer();
    }

  }
  else;
}

void Start_Measure_Func(void)
{
  ready_send_flag     = 1;
  //measure_flag        = 1;
  //first_measure_flag  = 1;

  buff_length = Length_buffer;

  buff_T    = (int16_t*) malloc(buff_length * sizeof(int16_t));
  if (buff_T == 0 )
  {
    error_flag = 1;
  }
  else;
}

 /**
  * @brief  Stop Measurement when received the stop command
  * @note
  * @retval None
  */
void Stop_Measure_Func(void)
{
 Reset_All();
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
