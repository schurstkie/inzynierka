/*
 * mode_config.h
 *
 *  Created on: May 15, 2016
 *      Author: kamil
 */

#ifndef MODE_CONFIG_H_
#define MODE_CONFIG_H_

void setVoltageInputMode(uint8_t channel);
void setCurrentInputMode(uint8_t channel);
void setVoltageOutputMode(uint8_t channel);
void setCurrentOutputMode(uint8_t channel);
void setCurrentPullUpMode(uint8_t channel);
void resetCurrentPullUpMode(uint8_t channel);
void setDefaultConfig(void);

#endif /* MODE_CONFIG_H_ */
