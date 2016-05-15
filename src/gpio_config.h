/*
 * gpio_config.h
 *
 *  Created on: Feb 7, 2016
 *      Author: kamil
 */

#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_hal_rcc.h"
#ifndef GPIO_CONFIG_H_
#define GPIO_CONFIG_H_

/**
  * @brief  AF 0 selection
  */
#define GPIO_AF_0            ((uint8_t)0x00) /* JTCK-SWCLK, JTDI, JTDO/TRACESW0, JTMS-SWDAT,
                                                MCO, NJTRST, TRACED, TRACECK */

//#define PORT_GPIOx(_N)   			((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(_N)))
#define PIN_MASK(_N)             	(1 << (_N))
#define RCC_MASKx(_N)             	(RCC_AHBPeriph_GPIOA << (_N))
#define DIGITAL_INPUT_1_PORT	(GPIOC)
#define DIGITAL_INPUT_1_PIN		(GPIO_Pin_9)

#define DIGITAL_INPUT_2_PORT	(GPIOC)
#define DIGITAL_INPUT_2_PIN		(GPIO_Pin_8)

#define DIGITAL_INPUT_3_PORT	(GPIOC)
#define DIGITAL_INPUT_3_PIN		(GPIO_Pin_7)

#define DIGITAL_INPUT_4_PORT	(GPIOC)
#define DIGITAL_INPUT_4_PIN		(GPIO_Pin_6)


#define DIGITAL_OUTPUT_1_PORT	(GPIOA)
#define DIGITAL_OUTPUT_1_PIN	(GPIO_Pin_7)

#define DIGITAL_OUTPUT_2_PORT	(GPIOB)
#define DIGITAL_OUTPUT_2_PIN	(GPIO_Pin_0)

#define DIGITAL_OUTPUT_3_PORT	(GPIOB)
#define DIGITAL_OUTPUT_3_PIN	(GPIO_Pin_2)

#define DIGITAL_OUTPUT_4_PORT	(GPIOB)
#define DIGITAL_OUTPUT_4_PIN	(GPIO_Pin_11)

#define SWITCH_MENU_UP_PORT		(GPIOA)
#define SWITCH_MENU_UP_PIN		(GPIO_Pin_11)

#define SWITCH_MENU_DOWN_PORT	(GPIOA)
#define SWITCH_MENU_DOWN_PIN	(GPIO_Pin_10)

#define SWITCH_MENU_LEFT_PORT	(GPIOA)
#define SWITCH_MENU_LEFT_PIN	(GPIO_Pin_9)

#define SWITCH_MENU_RIGHT_PORT	(GPIOA)
#define SWITCH_MENU_RIGHT_PIN	(GPIO_Pin_8)


#define CURRENT_MODE_UP_1_PORT	(GPIOA)
#define CURRENT_MODE_UP_1_PIN	(GPIO_Pin_6)

#define CURRENT_MODE_UP_2_PORT	(GPIOC)
#define CURRENT_MODE_UP_2_PIN	(GPIO_Pin_3)

#define VOLT_CURRENT_SEL_1_PORT	(GPIOA)
#define VOLT_CURRENT_SEL_1_PIN	(GPIO_Pin_2)

#define VOLT_CURRENT_SEL_1_1_PORT	(GPIOA)
#define VOLT_CURRENT_SEL_1_1_PIN	(GPIO_Pin_3)

#define VOLT_CURRENT_SEL_2_PORT		(GPIOA)
#define VOLT_CURRENT_SEL_2_PIN		(GPIO_Pin_0)

#define VOLT_CURRENT_SEL_2_1_PORT	(GPIOA)
#define VOLT_CURRENT_SEL_2_1_PIN	(GPIO_Pin_1)

#define VOLT_CURRENT_INPUT_SEL_1_PORT		(GPIOC)
#define VOLT_CURRENT_INPUT_SEL_1_PIN		(GPIO_Pin_0)

#define VOLT_CURRENT_INPUT_SEL_2_PORT	(GPIOA)
#define VOLT_CURRENT_INPUT_SEL_2_PIN	(GPIO_Pin_12)

#define DIGITAL_OUTPUT_SWITCH_1_PORT	(GPIOC)
#define DIGITAL_OUTPUT_SWITCH_1_PIN		(GPIO_Pin_4)

#define DIGITAL_OUTPUT_SWITCH_2_PORT	(GPIOC)
#define DIGITAL_OUTPUT_SWITCH_2_PIN		(GPIO_Pin_5)

#define DIGITAL_OUTPUT_SWITCH_3_PORT	(GPIOB)
#define DIGITAL_OUTPUT_SWITCH_3_PIN		(GPIO_Pin_1)

#define DIGITAL_OUTPUT_SWITCH_4_PORT	(GPIOB)
#define DIGITAL_OUTPUT_SWITCH_4_PIN		(GPIO_Pin_10)


#define ANALOG_VOLTAGE_INPUT_1_PORT		(GPIOB)
#define ANALOG_VOLTAGE_INPUT_1_PIN		(GPIO_Pin_13)

#define ANALOG_CURRENT_INPUT_1_PORT		(GPIOB)
#define ANALOG_CURRENT_INPUT_1_PIN		(GPIO_Pin_12)

#define ANALOG_VOLTAGE_INPUT_2_PORT		(GPIOB)
#define ANALOG_VOLTAGE_INPUT_2_PIN		(GPIO_Pin_15)

#define ANALOG_CURRENT_INPUT_2_PORT		(GPIOB)
#define ANALOG_CURRENT_INPUT_2_PIN		(GPIO_Pin_14)

#define ANALOG_POT_1_PORT				(GPIOC)
#define ANALOG_POT_1_PIN				(GPIO_Pin_1)

#define ANALOG_POT_2_PORT				(GPIOC)
#define ANALOG_POT_2_PIN				(GPIO_Pin_2)

#define ANALOG_OUTPUT_1_PORT			(GPIOA)
#define ANALOG_OUTPUT_1_PIN				(GPIO_Pin_5)

#define ANALOG_OUTPUT_2_PORT			(GPIOA)
#define ANALOG_OUTPUT_2_PIN				(GPIO_Pin_4)

void GPIO_Config();

#endif /* GPIO_CONFIG_H_ */
