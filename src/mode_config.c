/*
 * mode_config.c
 *
 *  Created on: May 15, 2016
 *      Author: kamil
 */

#include "gpio_config.h"
#include "mode_config.h"

void setVoltageInputMode(uint8_t channel)
{
	switch(channel)
	{
	case 1:
		HAL_GPIO_WritePin(VOLT_CURRENT_INPUT_SEL_1_PORT,VOLT_CURRENT_INPUT_SEL_1_PIN,GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(VOLT_CURRENT_INPUT_SEL_2_PORT,VOLT_CURRENT_INPUT_SEL_2_PIN,GPIO_PIN_RESET);
		break;
	}

}
void setCurrentInputMode(uint8_t channel)
{
	switch(channel)
	{
		case 1:
			HAL_GPIO_WritePin(VOLT_CURRENT_INPUT_SEL_1_PORT,VOLT_CURRENT_INPUT_SEL_1_PIN,GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(VOLT_CURRENT_INPUT_SEL_2_PORT,VOLT_CURRENT_INPUT_SEL_2_PIN,GPIO_PIN_SET);
			break;
	}
}
void setVoltageOutputMode(uint8_t channel)
{
	switch(channel)
		{
			case 1:
				HAL_GPIO_WritePin(VOLT_CURRENT_SEL_1_PORT,VOLT_CURRENT_SEL_1_PIN,GPIO_PIN_SET);
				HAL_GPIO_WritePin(VOLT_CURRENT_SEL_1_1_PORT,VOLT_CURRENT_SEL_1_1_PIN,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(CURRENT_MODE_UP_1_PORT,CURRENT_MODE_UP_1_PIN,GPIO_PIN_RESET);

				break;
			case 2:
				HAL_GPIO_WritePin(VOLT_CURRENT_SEL_2_PORT,VOLT_CURRENT_SEL_2_PIN,GPIO_PIN_SET);
				HAL_GPIO_WritePin(VOLT_CURRENT_SEL_2_1_PORT,VOLT_CURRENT_SEL_2_1_PIN,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(CURRENT_MODE_UP_2_PORT,CURRENT_MODE_UP_2_PIN,GPIO_PIN_RESET);

				break;
		}
}
void setCurrentOutputMode(uint8_t channel)
{
	switch(channel)
			{
				case 1:
					HAL_GPIO_WritePin(VOLT_CURRENT_SEL_1_PORT,VOLT_CURRENT_SEL_1_PIN,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(VOLT_CURRENT_SEL_1_1_PORT,VOLT_CURRENT_SEL_1_1_PIN,GPIO_PIN_SET);

					break;
				case 2:
					HAL_GPIO_WritePin(VOLT_CURRENT_SEL_2_PORT,VOLT_CURRENT_SEL_2_PIN,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(VOLT_CURRENT_SEL_2_1_PORT,VOLT_CURRENT_SEL_2_1_PIN,GPIO_PIN_SET);

					break;
			}
}
void setCurrentPullUpMode(uint8_t channel)
{
	switch(channel)
				{
					case 1:
						HAL_GPIO_WritePin(CURRENT_MODE_UP_1_PORT,CURRENT_MODE_UP_1_PIN,GPIO_PIN_SET);
						break;
					case 2:
						HAL_GPIO_WritePin(CURRENT_MODE_UP_2_PORT,CURRENT_MODE_UP_2_PIN,GPIO_PIN_SET);
						break;
				}
}
void resetCurrentPullUpMode(uint8_t channel)
{
	switch(channel)
				{
					case 1:
						HAL_GPIO_WritePin(CURRENT_MODE_UP_1_PORT,CURRENT_MODE_UP_1_PIN,GPIO_PIN_RESET);
						break;
					case 2:
						HAL_GPIO_WritePin(CURRENT_MODE_UP_2_PORT,CURRENT_MODE_UP_2_PIN,GPIO_PIN_RESET);
						break;
				}
}
void setDefaultConfig()
{
	setVoltageInputMode(1);
	setVoltageInputMode(2);
	setVoltageOutputMode(1);
	setVoltageOutputMode(2);
	HAL_GPIO_WritePin(DIGITAL_OUTPUT_1_PORT,DIGITAL_OUTPUT_1_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIGITAL_OUTPUT_2_PORT,DIGITAL_OUTPUT_2_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIGITAL_OUTPUT_3_PORT,DIGITAL_OUTPUT_3_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIGITAL_OUTPUT_4_PORT,DIGITAL_OUTPUT_4_PIN,GPIO_PIN_SET);

}
void getOutputModeState(uint8_t channel)
{

}
void getInputModeState(uint8_t channel)
{

}
