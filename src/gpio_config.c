/*
 * gpio_config.c
 *
 *  Created on: Feb 6, 2016
 *      Author: kamil
 */

#include "gpio_config.h"




/*
 * (DIGITAL OUTPUT)
 * DO1 - PA7
 * DO2 - PB0
 * DO3 - PB2
 * DO4 - PB11
 *
 * IN:
 *
 * (DIGITAL OUTPUT SWITCHES)
 * SW5 - PB10
 * SW6 - PB1
 * SW7 - PC5
 * SW8 - PC4
 *
 * (MENU SWITCHES)
 * SW1 - PA9
 * SW2 - PA10
 * SW3 - PA11
 * SW4 - PA12
 *
 * (DIGITAL INPUTS)
 * DI1 - PC9
 * DI2 - PC8
 * DI3 - PC7
 * DI4 - PC6
 *
 *
 */
void GPIO_Config(void)
{

	GPIO_InitTypeDef        GPIO_InitStructure;


//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
__GPIOB_CLK_ENABLE;
__GPIOA_CLK_ENABLE;
__GPIOC_CLK_ENABLE;

  /*DIGITAL INPUT CONFIG */

  GPIO_InitStructure.Pin = DIGITAL_INPUT_1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(DIGITAL_INPUT_1_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = DIGITAL_INPUT_2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(DIGITAL_INPUT_2_PORT, &GPIO_InitStructure);


  GPIO_InitStructure.Pin = DIGITAL_INPUT_3_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(DIGITAL_INPUT_3_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = DIGITAL_INPUT_4_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(DIGITAL_INPUT_4_PORT, &GPIO_InitStructure);


  /* DIGITAL OUTPUT CONFIG */

  GPIO_InitStructure.Pin = DIGITAL_OUTPUT_1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(DIGITAL_OUTPUT_1_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = DIGITAL_OUTPUT_2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(DIGITAL_OUTPUT_2_PORT, &GPIO_InitStructure);


  GPIO_InitStructure.Pin = DIGITAL_OUTPUT_3_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(DIGITAL_OUTPUT_3_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = DIGITAL_OUTPUT_4_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(DIGITAL_OUTPUT_4_PORT, &GPIO_InitStructure);


/* MENU SWITCHES CONFIG */

  GPIO_InitStructure.Pin = SWITCH_MENU_UP_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(SWITCH_MENU_UP_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = SWITCH_MENU_DOWN_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(SWITCH_MENU_DOWN_PORT, &GPIO_InitStructure);


  GPIO_InitStructure.Pin = SWITCH_MENU_LEFT_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(SWITCH_MENU_LEFT_PORT, &GPIO_InitStructure);

//  GPIO_InitStructure.Pin = SWITCH_MENU_RIGHT_PIN;
//  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
//  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//  GPIO_InitStructure.Pull = GPIO_PULLUP;
//  GPIO_Init(SWITCH_MENU_RIGHT_PORT, &GPIO_InitStructure);

/* CURRENT PULL UP CONFIG */
  GPIO_InitStructure.Pin = CURRENT_MODE_UP_1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(CURRENT_MODE_UP_1_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = CURRENT_MODE_UP_2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(CURRENT_MODE_UP_2_PORT, &GPIO_InitStructure);

/* CURRENT - VOLTAGE MODE CONFIG */

  GPIO_InitStructure.Pin = VOLT_CURRENT_SEL_1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(VOLT_CURRENT_SEL_1_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = VOLT_CURRENT_SEL_1_1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(VOLT_CURRENT_SEL_1_1_PORT, &GPIO_InitStructure);


  GPIO_InitStructure.Pin = VOLT_CURRENT_SEL_2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(VOLT_CURRENT_SEL_2_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = VOLT_CURRENT_SEL_2_1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_Init(VOLT_CURRENT_SEL_2_1_PORT, &GPIO_InitStructure);


  /* DIGITAL OUTPUT SWITCHES CONFIG */


  GPIO_InitStructure.Pin = DIGITAL_OUTPUT_SWITCH_1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(DIGITAL_OUTPUT_SWITCH_1_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = DIGITAL_OUTPUT_SWITCH_2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(DIGITAL_OUTPUT_SWITCH_2_PORT, &GPIO_InitStructure);


  GPIO_InitStructure.Pin = DIGITAL_OUTPUT_SWITCH_3_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(DIGITAL_OUTPUT_SWITCH_3_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = DIGITAL_OUTPUT_SWITCH_4_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStructure.OType = GPIO_OType_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_Init(DIGITAL_OUTPUT_SWITCH_4_PORT, &GPIO_InitStructure);


  //current up default
  GPIO_ResetBits(CURRENT_MODE_UP_1_PORT,CURRENT_MODE_UP_1_PIN);
  GPIO_ResetBits(CURRENT_MODE_UP_2_PORT,CURRENT_MODE_UP_2_PIN);

  //volt_current select pins default
  GPIO_ResetBits(VOLT_CURRENT_SEL_1_PORT,VOLT_CURRENT_SEL_1_PIN);
  GPIO_SetBits(VOLT_CURRENT_SEL_1_1_PORT,VOLT_CURRENT_SEL_1_1_PIN);
  GPIO_ResetBits(VOLT_CURRENT_SEL_2_PORT,VOLT_CURRENT_SEL_2_PIN);
  GPIO_SetBits(VOLT_CURRENT_SEL_2_1_PORT,VOLT_CURRENT_SEL_2_1_PIN);

  //digital output pins default
  GPIO_ResetBits(DIGITAL_OUTPUT_1_PORT,DIGITAL_OUTPUT_1_PIN);
  GPIO_SetBits(DIGITAL_OUTPUT_2_PORT,DIGITAL_OUTPUT_2_PIN);
  GPIO_SetBits(DIGITAL_OUTPUT_3_PORT,DIGITAL_OUTPUT_3_PIN);
  GPIO_SetBits(DIGITAL_OUTPUT_4_PORT,DIGITAL_OUTPUT_4_PIN);

  GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << (11*2);
  GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_LOW;
//  GPIOB->OTYPER|= GPIO_OType_PP;
  GPIOB->PUPDR |= GPIO_NOPULL;
  GPIOB->BSRR = GPIO_Pin_11;









}
