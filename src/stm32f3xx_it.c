/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "gpio_config.h"
#include "KS0108-STM32.h"
#include "stdlib.h"
#include "itoa.h"
#include "arm_math.h"
#include "stdbool.h"

#define ADC1_CHANNELS_NUMBER 3
#define ADC2_CHANNELS_NUMBER 3
#define ADC_BUFFER_LENGTH 60

uint32_t value=0;
uint8_t	adc_1_value_cnt;
uint8_t adc_2_value_cnt;
uint8_t	adc_1_cnt;
uint8_t adc_2_cnt;
int licznik=0;
int ind=0;

extern ADCValue;

extern unsigned char test;
extern float32_t AN1_Pot_Value;
extern float32_t AN2_Pot_Value;
extern volatile uint32_t DACValues1[2];
extern volatile uint32_t DACValues2[2];
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef g_DmaHandle1;
extern DMA_HandleTypeDef g_DmaHandle2;
extern DMA_HandleTypeDef g_DmaHandle3;
extern DMA_HandleTypeDef g_DmaHandle4;
extern TIM_HandleTypeDef 	htim6;

extern uint32_t g_ADCBuffer1[ADC_BUFFER_LENGTH];
extern uint32_t g_ADCBuffer2[ADC_BUFFER_LENGTH];

volatile uint32_t ticks;
extern volatile bool COUNTER_FLAG;
extern volatile uint8_t adc_1_conv_in_progress;
extern volatile uint8_t adc_2_conv_in_progress;
volatile uint8_t  adc_1_conv_in_progress;
volatile uint8_t  adc_2_conv_in_progress;
extern volatile uint32_t adc_1_value[ADC1_CHANNELS_NUMBER];
extern volatile uint32_t adc_2_value[ADC1_CHANNELS_NUMBER+ADC2_CHANNELS_NUMBER];


extern void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
		{
//		ADCValue=accumulate(g_ADCBuffer,g_ADCBuffer + ADC_BUFFER_LENGTH,0)/ADC_BUFFER_LENGTH;
	int i=0;
	int j=0;
	ind++;
	if(ind==2)
		ind=0;
	for(i=0;i<ADC_BUFFER_LENGTH;i+=ADC1_CHANNELS_NUMBER)
	{
		adc_1_value[0]+=g_ADCBuffer1[i];
		adc_1_value[1]+=g_ADCBuffer1[i+1];
		adc_1_value[2]+=g_ADCBuffer1[i+2];

		adc_1_value[3]+=g_ADCBuffer2[i];
		adc_1_value[4]+=g_ADCBuffer2[i+1];
		adc_1_value[5]+=g_ADCBuffer2[i+2];
	}
		adc_2_value[0]=adc_1_value[0]/(ADC_BUFFER_LENGTH/ADC1_CHANNELS_NUMBER);
		adc_2_value[1]=adc_1_value[1]/(ADC_BUFFER_LENGTH/ADC1_CHANNELS_NUMBER);
		adc_2_value[2]=adc_1_value[2]/(ADC_BUFFER_LENGTH/ADC1_CHANNELS_NUMBER);

		adc_2_value[3]=adc_1_value[3]/(ADC_BUFFER_LENGTH/ADC1_CHANNELS_NUMBER);
		adc_2_value[4]=adc_1_value[4]/(ADC_BUFFER_LENGTH/ADC1_CHANNELS_NUMBER);
		adc_2_value[5]=adc_1_value[5]/(ADC_BUFFER_LENGTH/ADC1_CHANNELS_NUMBER);

		DACValues1[ind]=adc_2_value[0];
		DACValues2[ind]=adc_2_value[1];

		adc_1_value[0]=0;
		adc_1_value[1]=0;
		adc_1_value[2]=0;

		adc_1_value[3]=0;
		adc_1_value[4]=0;
		adc_1_value[5]=0;

//		if(adc_1_cnt==2)
//			adc_1_cnt=0;

		AN1_Pot_Value=adc_1_value[0]*0.08056640625;
		AN2_Pot_Value=adc_1_value[1]*0.08056640625;
		}
extern void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* DacHandle)
		{
//while(1);
		}
extern void HAL_DAC_ConvCpltCallbackCh2(DAC_HandleTypeDef* DacHandle)
		{

		}





/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}


/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  if((COUNTER_FLAG)&(ticks!=0))
  ticks--;
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_1_PORT,DIGITAL_OUTPUT_1_PIN,!(HAL_GPIO_ReadPin(DIGITAL_OUTPUT_SWITCH_1_PORT,DIGITAL_OUTPUT_SWITCH_1_PIN)));
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_2_PORT,DIGITAL_OUTPUT_2_PIN,!(HAL_GPIO_ReadPin(DIGITAL_OUTPUT_SWITCH_2_PORT,DIGITAL_OUTPUT_SWITCH_2_PIN)));
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_3_PORT,DIGITAL_OUTPUT_3_PIN,!(HAL_GPIO_ReadPin(DIGITAL_OUTPUT_SWITCH_3_PORT,DIGITAL_OUTPUT_SWITCH_3_PIN)));
  HAL_GPIO_WritePin(DIGITAL_OUTPUT_4_PORT,DIGITAL_OUTPUT_4_PIN,!(HAL_GPIO_ReadPin(DIGITAL_OUTPUT_SWITCH_4_PORT,DIGITAL_OUTPUT_SWITCH_4_PIN)));



}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
//void DMA1_Channel1_IRQHandler()
//{
//	HAL_DMA_IRQHandler(&g_DmaHandle);
//
//}


void SysTickDelay(uint32_t i)
{
	COUNTER_FLAG=true;
	ticks=i;
	while(COUNTER_FLAG)
	{
		if(ticks==0x00)
			COUNTER_FLAG=false;
	}

}

void ADC1_2_IRQHandler()
{
	HAL_ADC_IRQHandler(&hadc1);

}


void DMA1_Channel1_IRQHandler()
{
	HAL_DMA_IRQHandler(&g_DmaHandle1);
}

void DMA1_Channel2_IRQHandler()
{
	HAL_DMA_IRQHandler(&g_DmaHandle2);
}
void DMA1_Channel3_IRQHandler()
{
	HAL_DMA_IRQHandler(&g_DmaHandle3);
}

void DMA1_Channel4_IRQHandler()
{
	HAL_DMA_IRQHandler(&g_DmaHandle4);
}
void TIM6_DAC1_IRQHandler()
{
	HAL_TIM_IRQHandler(&htim6);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
