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
#include "gpio_config.h"
#include "stdbool.h"
#include "KS0108.h"



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t Analog_current_input1=0;
uint32_t Analog_voltage_input1=0;
uint32_t Analog_current_input2=0;
uint32_t Analog_voltage_input2=0;
uint32_t X1=0;
uint32_t X2=32;
uint32_t Y1=0;
uint32_t Y2=32;

bool AUTO_MANUAL_MODE=false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);






/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
	  MX_GPIO_Init();
	  MX_ADC1_Init();
	  MX_ADC2_Init();
	  MX_DAC1_Init();

	//NVIC_SetPriority(ADC1_2_IRQn, 2);
	//NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();


  /* Initialize all configured peripherals */



  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_1_PORT,DIGITAL_OUTPUT_1_PIN,GPIO_PIN_SET);

  HAL_GPIO_WritePin(DIGITAL_OUTPUT_2_PORT,DIGITAL_OUTPUT_2_PIN,GPIO_PIN_SET);

  HAL_GPIO_WritePin(DIGITAL_OUTPUT_3_PORT,DIGITAL_OUTPUT_3_PIN,GPIO_PIN_SET);

  HAL_GPIO_WritePin(DIGITAL_OUTPUT_4_PORT,DIGITAL_OUTPUT_4_PIN,GPIO_PIN_SET);
//  	GPIO_PinAFConfig(GPIOA, 0x08, 0x00);



  GLCD_Initialize();
  GLCD_Delay();
  GLCD_ClearScreen();
  GLCD_WriteStringNeg("HELLO");

//  GLCD_Line(X1,X2,Y1,Y2);


  HAL_ADC_Start_IT(&hadc1);

  /* Configure the system clock */



  while (1)
  {
  if(AUTO_MANUAL_MODE==0)
  {
	  HAL_GPIO_WritePin(DIGITAL_OUTPUT_1_PORT,DIGITAL_OUTPUT_1_PIN,!(HAL_GPIO_ReadPin(DIGITAL_OUTPUT_SWITCH_1_PORT,DIGITAL_OUTPUT_SWITCH_1_PIN)));
	  HAL_GPIO_WritePin(DIGITAL_OUTPUT_2_PORT,DIGITAL_OUTPUT_2_PIN,!(HAL_GPIO_ReadPin(DIGITAL_OUTPUT_SWITCH_2_PORT,DIGITAL_OUTPUT_SWITCH_2_PIN)));
	  HAL_GPIO_WritePin(DIGITAL_OUTPUT_3_PORT,DIGITAL_OUTPUT_3_PIN,!(HAL_GPIO_ReadPin(DIGITAL_OUTPUT_SWITCH_3_PORT,DIGITAL_OUTPUT_SWITCH_3_PIN)));
	  HAL_GPIO_WritePin(DIGITAL_OUTPUT_4_PORT,DIGITAL_OUTPUT_4_PIN,!(HAL_GPIO_ReadPin(DIGITAL_OUTPUT_SWITCH_4_PORT,DIGITAL_OUTPUT_SWITCH_4_PIN)));
//
	  /*digital in to out*/
//	  HAL_GPIO_WritePin(DIGITAL_OUTPUT_1_PORT,DIGITAL_OUTPUT_1_PIN,HAL_GPIO_ReadPin(DIGITAL_INPUT_1_PORT,DIGITAL_INPUT_1_PIN));
//	  HAL_GPIO_WritePin(DIGITAL_OUTPUT_2_PORT,DIGITAL_OUTPUT_2_PIN,HAL_GPIO_ReadPin(DIGITAL_INPUT_2_PORT,DIGITAL_INPUT_2_PIN));
//	  HAL_GPIO_WritePin(DIGITAL_OUTPUT_3_PORT,DIGITAL_OUTPUT_3_PIN,HAL_GPIO_ReadPin(DIGITAL_INPUT_3_PORT,DIGITAL_INPUT_3_PIN));
//	  HAL_GPIO_WritePin(DIGITAL_OUTPUT_4_PORT,DIGITAL_OUTPUT_4_PIN,HAL_GPIO_ReadPin(DIGITAL_INPUT_4_PORT,DIGITAL_INPUT_4_PIN));
  }
  }
  /* USER CODE END 3 */

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

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);


    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc2);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset=0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);


  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset=0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);




}

/* DAC1 init function */
void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  HAL_DAC_Init(&hdac1);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1);

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
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
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_INPUT_1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DIGITAL_INPUT_2_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_INPUT_2_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = DIGITAL_INPUT_3_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DIGITAL_INPUT_3_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DIGITAL_INPUT_4_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
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



//
//    //current up default
//    HAL_GPIO_WritePin(CURRENT_MODE_UP_1_PORT,CURRENT_MODE_UP_1_PIN,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(CURRENT_MODE_UP_2_PORT,CURRENT_MODE_UP_2_PIN,GPIO_PIN_RESET);
//
//    //volt_current select pins default
//    HAL_GPIO_WritePin(VOLT_CURRENT_SEL_1_PORT,VOLT_CURRENT_SEL_1_PIN,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(VOLT_CURRENT_SEL_1_1_PORT,VOLT_CURRENT_SEL_1_1_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(VOLT_CURRENT_SEL_2_PORT,VOLT_CURRENT_SEL_2_PIN,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(VOLT_CURRENT_SEL_2_1_PORT,VOLT_CURRENT_SEL_2_1_PIN,GPIO_PIN_SET);
//
//    //digital output pins default
//    HAL_GPIO_WritePin(DIGITAL_OUTPUT_1_PORT,DIGITAL_OUTPUT_1_PIN,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(DIGITAL_OUTPUT_2_PORT,DIGITAL_OUTPUT_2_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(DIGITAL_OUTPUT_3_PORT,DIGITAL_OUTPUT_3_PIN,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(DIGITAL_OUTPUT_4_PORT,DIGITAL_OUTPUT_4_PIN,GPIO_PIN_SET);
  /*Configure GPIO pins : PC13 PC14 PC15 PC0
                           PC3 PC5 PC10 PC11
                           PC12 */
//  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
//                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11
//                          |GPIO_PIN_12;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PA0 PA1 PA2 PA3
//                           PA6 PA7 PA12 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
//                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PC4 PC6 PC7 PC8
//                           PC9 */
//  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
//                          |GPIO_PIN_9;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PB0 PB2 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PB1 PB10 PB11 PB4
//                           PB5 PB6 PB7 PB8
//                           PB9 */
//  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_4
//                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
//                          |GPIO_PIN_9;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
//  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PD2 */
//  GPIO_InitStruct.Pin = GPIO_PIN_2;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


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

  //HAL_RCC_MCOConfig(RCC_MCO,RCC_MCOSOURCE_SYSCLK,RCC_MCODIV_4);


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
