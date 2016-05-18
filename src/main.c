/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "stm32f334x8.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_hal_rcc_ex.h"
#include "stm32f3xx_hal_dma_ex.h"
#include "gpio_config.h"
#include "stdbool.h"
#include "KS0108.h"
//#include "justa2.h"
#include "iair.h"
#include "arm_math.h"
#include "mode_config.h"
#include "calc.h"
#include "fifo.h"
#include "menu.h"




#define ADC1_CHANNELS_NUMBER 3
#define ADC2_CHANNELS_NUMBER 3
#define ADC_BUFFER_LENGTH 60

#define volt_input_multiplier 8
#define current_input_multiplier 8

#define volt_output_multiplier 8
#define current_output_multiplier 8

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef 	hdac1;
DMA_HandleTypeDef  	g_DmaHandle1;
DMA_HandleTypeDef 	g_DmaHandle2;
DMA_HandleTypeDef  	g_DmaHandle3;
DMA_HandleTypeDef 	g_DmaHandle4;
TIM_HandleTypeDef 	htim6;


//uint32_t ticks;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t Analog_current_input1=0;
uint16_t Analog_voltage_input1=0;
uint16_t Analog_current_input2=0;
uint16_t Analog_voltage_input2=0;

float32_t AN1_Pot_Value;
float32_t AN2_Pot_Value;

uint32_t g_ADCBuffer1[ADC_BUFFER_LENGTH];
uint32_t g_ADCBuffer2[ADC_BUFFER_LENGTH];

uint16_t ADCValue;
volatile uint32_t DACValues1[2];
volatile uint32_t DACValues2[2];

float volt1[5];
float curr1[5];
float volt2[5];
float curr2[5];
//float pot1[10];
//float pot2[10];

volatile bool COUNTER_FLAG=false;
volatile uint32_t adc_1_value[ADC1_CHANNELS_NUMBER+ADC2_CHANNELS_NUMBER];
volatile uint32_t adc_2_value[ADC1_CHANNELS_NUMBER+ADC2_CHANNELS_NUMBER];
volatile uint32_t adc_2_value_last[ADC1_CHANNELS_NUMBER+ADC2_CHANNELS_NUMBER];
char temp[16];
//int MeasurementNumber;


//unsigned char test=0;

bool AUTO_MODE=false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void ConfigureDMA(void);
static void MX_TIM6_Init(void);

//fifo_t




/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  set_object_transmittance(1,39.107,0,3.893);
  set_disruption1_transmittance(1,39.107,0,0);
  set_disruption2_transmittance(-1,52.1,0,0);
  MX_GPIO_Init();
  ConfigureDMA();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  setDefaultConfig();


  GLCD_Initialize();
  GLCD_Delay();
  GLCD_ClearScreen();
  GLCD_Delay();

//  GLCD_GoToReversed(0,0);
  //GLCD_GoTo(63,63);
//  GLCD_Bitmap_Reversed(obrazek2,0,0,128,64);
//while(1);
  //GLCD_WriteStringNeg("HELLO WORLD TEST");

//  GLCD_Line(X1,X2,Y1,Y2);

  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer1, ADC_BUFFER_LENGTH);
  HAL_ADC_Start_DMA(&hadc2, g_ADCBuffer2, ADC_BUFFER_LENGTH);

  HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);

  //DAC1->DHR12R1 = 0b011111111111;
  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,(uint32_t*)DACValues1,1,DAC_ALIGN_12B_R);
  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_2,(uint32_t*)DACValues2,1,DAC_ALIGN_12B_R);

  int licznik=0;
//  setCurrentInputMode(1);
//  setCurrentInputMode(2);
//  setCurrentOutputMode(1);
//  setCurrentOutputMode(2);
//  setCurrentPullUpMode(1);
//  setCurrentPullUpMode(2);


  GLCD_Bitmap_Reversed(logo,0,0,128,64);
//  GLCD_Bitmap_Reversed(ewa,0,0,128,64);
  SysTickDelay(200);
  GLCD_ClearScreen();



  while (1)
  {
	  SysTickDelay(50);
	  GLCD_ClearScreen();
	  show_menu();

  if(AUTO_MODE==0)
  {

//	  SysTickDelay(50);
//	  for(licznik=0;licznik<(ADC1_CHANNELS_NUMBER+ADC2_CHANNELS_NUMBER);licznik++)
//	  {

//		  GLCD_ClearScreen();

//		  GLCD_GoToReversed(0,licznik);
//		  itoa((int)adc_2_value[licznik],temp,10);
//		  GLCD_ClearPage(licznik);
//		  GLCD_GoToReversed(0,licznik);//		  GLCD_WriteStringNeg(temp);
////		  GLCD_GoToReversed(0,licznik);
//		  GLCD_Delay();
//		  GLCD_GoToReversed(0,6);
//		  itoa((int)DACValues1[0],temp,10);
//		  GLCD_ClearPage(6);
//		  GLCD_GoToReversed(0,6);
//		  GLCD_WriteStringNeg(temp);
////		  GLCD_GoToReversed(0,licznik);
//	  }




  }
  }
}

/** System Clock Configuration
*/

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

	__HAL_RCC_ADC12_CLK_ENABLE();

	HAL_NVIC_SetPriority(ADC1_IRQn,0,0);
	HAL_NVIC_EnableIRQ(ADC1_IRQn);

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 0;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = DISABLE;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);
  //HAL_ADC_GetState()


    /**Configure Regular Channel 
    */
  /*Analog Pot1 Output*/
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  asm("bkpt 255");

  /*Analog Pot2 Output*/
  sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 2;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  asm("bkpt 255");

	/*Analog 1 Input Current Mode */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 3;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  asm("bkpt 255");



}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

__HAL_RCC_ADC2_CLK_ENABLE();
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = DISABLE;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc2);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);
  if(HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  	  asm("bkpt 255");

  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset=0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);
  if(HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  	  asm("bkpt 255");

  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 3;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset=0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);
  if(HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  	  asm("bkpt 255");



}

/* DAC1 init function */
void MX_DAC1_Init(void)
{
	__DAC1_CLK_ENABLE();

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  HAL_DAC_Init(&hdac1);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1);

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_Trigger= DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2);

}
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
  assert_param(IS_GPIO_AF(GPIO_AF));

  temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
}
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*DIGITAL INPUT CONFIG */

    GPIO_InitStructure.Pin = DIGITAL_INPUT_1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_INPUT_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DIGITAL_INPUT_2_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_INPUT_2_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = DIGITAL_INPUT_3_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_INPUT_3_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DIGITAL_INPUT_4_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_INPUT_4_PORT, &GPIO_InitStructure);


    /* DIGITAL OUTPUT CONFIG */

    GPIO_InitStructure.Pin = DIGITAL_OUTPUT_1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DIGITAL_OUTPUT_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DIGITAL_OUTPUT_2_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DIGITAL_OUTPUT_2_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = DIGITAL_OUTPUT_3_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DIGITAL_OUTPUT_3_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DIGITAL_OUTPUT_4_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DIGITAL_OUTPUT_4_PORT, &GPIO_InitStructure);


  /* MENU SWITCHES CONFIG */

    GPIO_InitStructure.Pin = SWITCH_MENU_UP_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SWITCH_MENU_UP_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SWITCH_MENU_DOWN_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SWITCH_MENU_DOWN_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = SWITCH_MENU_LEFT_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SWITCH_MENU_LEFT_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SWITCH_MENU_RIGHT_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SWITCH_MENU_RIGHT_PORT, &GPIO_InitStructure);

  /* CURRENT PULL UP CONFIG */
    GPIO_InitStructure.Pin = CURRENT_MODE_UP_1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CURRENT_MODE_UP_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = CURRENT_MODE_UP_2_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CURRENT_MODE_UP_2_PORT, &GPIO_InitStructure);

  /* CURRENT - VOLTAGE MODE CONFIG */

    GPIO_InitStructure.Pin = VOLT_CURRENT_SEL_1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VOLT_CURRENT_SEL_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = VOLT_CURRENT_SEL_1_1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VOLT_CURRENT_SEL_1_1_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = VOLT_CURRENT_SEL_2_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VOLT_CURRENT_SEL_2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = VOLT_CURRENT_SEL_2_1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VOLT_CURRENT_SEL_2_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = VOLT_CURRENT_INPUT_SEL_1_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VOLT_CURRENT_INPUT_SEL_1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = VOLT_CURRENT_INPUT_SEL_2_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VOLT_CURRENT_INPUT_SEL_2_PORT, &GPIO_InitStructure);


    /* DIGITAL OUTPUT SWITCHES CONFIG */


    GPIO_InitStructure.Pin = DIGITAL_OUTPUT_SWITCH_1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_OUTPUT_SWITCH_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DIGITAL_OUTPUT_SWITCH_2_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_OUTPUT_SWITCH_2_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = DIGITAL_OUTPUT_SWITCH_3_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_OUTPUT_SWITCH_3_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DIGITAL_OUTPUT_SWITCH_4_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_OUTPUT_SWITCH_4_PORT, &GPIO_InitStructure);

    /* ANALOG INPUTS CONFIG */
    GPIO_InitStructure.Pin = ANALOG_CURRENT_INPUT_1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ANALOG_CURRENT_INPUT_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = ANALOG_CURRENT_INPUT_2_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ANALOG_CURRENT_INPUT_2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ANALOG_VOLTAGE_INPUT_1_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ANALOG_VOLTAGE_INPUT_1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ANALOG_VOLTAGE_INPUT_2_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ANALOG_VOLTAGE_INPUT_2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ANALOG_POT_1_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ANALOG_POT_1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ANALOG_POT_2_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ANALOG_POT_2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ANALOG_OUTPUT_1_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ANALOG_OUTPUT_1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ANALOG_OUTPUT_2_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(ANALOG_OUTPUT_2_PORT, &GPIO_InitStructure);



  //current up default
  HAL_GPIO_WritePin(CURRENT_MODE_UP_1_PORT,CURRENT_MODE_UP_1_PIN,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CURRENT_MODE_UP_2_PORT,CURRENT_MODE_UP_2_PIN,GPIO_PIN_RESET);

  //volt_current select pins default
  HAL_GPIO_WritePin(VOLT_CURRENT_SEL_1_PORT,VOLT_CURRENT_SEL_1_PIN,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VOLT_CURRENT_SEL_1_1_PORT,VOLT_CURRENT_SEL_1_1_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(VOLT_CURRENT_SEL_2_PORT,VOLT_CURRENT_SEL_2_PIN,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VOLT_CURRENT_SEL_2_1_PORT,VOLT_CURRENT_SEL_2_1_PIN,GPIO_PIN_SET);

  //digital output pins default
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_1_PORT,DIGITAL_OUTPUT_1_PIN,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_2_PORT,DIGITAL_OUTPUT_2_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_3_PORT,DIGITAL_OUTPUT_3_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_4_PORT,DIGITAL_OUTPUT_4_PIN,GPIO_PIN_SET);




}

void ConfigureDMA()
{
    __DMA1_CLK_ENABLE();
    g_DmaHandle1.Instance=DMA1_Channel1;
    g_DmaHandle1.Init.Direction=DMA_PERIPH_TO_MEMORY;
    g_DmaHandle1.Init.PeriphInc=DMA_PINC_DISABLE;
    g_DmaHandle1.Init.MemInc=DMA_MINC_ENABLE;
    g_DmaHandle1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    g_DmaHandle1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    g_DmaHandle1.Init.Mode = DMA_CIRCULAR;

    HAL_DMA_Init(&g_DmaHandle1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, g_DmaHandle1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    g_DmaHandle2.Instance=DMA1_Channel2;
	g_DmaHandle2.Init.Direction=DMA_PERIPH_TO_MEMORY;
	g_DmaHandle2.Init.PeriphInc=DMA_PINC_DISABLE;
	g_DmaHandle2.Init.MemInc=DMA_MINC_ENABLE;
	g_DmaHandle2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	g_DmaHandle2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	g_DmaHandle2.Init.Mode = DMA_CIRCULAR;

	HAL_DMA_Init(&g_DmaHandle2);

	__HAL_LINKDMA(&hadc2, DMA_Handle, g_DmaHandle2);

	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	g_DmaHandle3.Instance=DMA1_Channel3;
	g_DmaHandle3.Init.Direction=DMA_MEMORY_TO_PERIPH;
	g_DmaHandle3.Init.PeriphInc=DMA_PINC_DISABLE;
	g_DmaHandle3.Init.MemInc=DMA_MINC_ENABLE;
	g_DmaHandle3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	g_DmaHandle3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	g_DmaHandle3.Init.Mode = DMA_CIRCULAR;
	g_DmaHandle3.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&g_DmaHandle3);
	__HAL_REMAPDMA_CHANNEL_ENABLE(HAL_REMAPDMA_TIM6_DAC1_CH1_DMA1_CH3);
	__HAL_LINKDMA(&hdac1, DMA_Handle1, g_DmaHandle3);

	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	g_DmaHandle4.Instance=DMA1_Channel4;
	g_DmaHandle4.Init.Direction=DMA_MEMORY_TO_PERIPH;
	g_DmaHandle4.Init.PeriphInc=DMA_PINC_DISABLE;
	g_DmaHandle4.Init.MemInc=DMA_MINC_ENABLE;
	g_DmaHandle4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	g_DmaHandle4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	g_DmaHandle4.Init.Mode = DMA_CIRCULAR;
	g_DmaHandle4.Init.Priority = DMA_PRIORITY_LOW;

	HAL_DMA_Init(&g_DmaHandle4);
	__HAL_REMAPDMA_CHANNEL_ENABLE(HAL_REMAPDMA_TIM7_DAC1_CH2_DMA1_CH4);
	__HAL_LINKDMA(&hdac1, DMA_Handle2, g_DmaHandle4);

	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/* TIM6 init function */
void MX_TIM6_Init(void)
{
	__TIM6_CLK_ENABLE();
  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65530;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  TIM6->CR2=0x02;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
//  TIM6->CR2=0x20;


}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
