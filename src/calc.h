/*
 * calc.h
 *
 *  Created on: May 15, 2016
 *      Author: kamil
 */

#ifndef CALC_H_
#define CALC_H_

void calc_object_transmittance_value(uint32_t *in, uint32_t *out);
void calc_disruption1_transmittance_value(uint32_t *in, uint32_t *out);
void calc_disruption2_transmittance_value(uint32_t *in, uint32_t *out);
void set_disruption1_transmittance(uint32_t gain,uint32_t t1,uint32_t t2,uint32_t delay);
void set_disruption2_transmittance(uint32_t gain,uint32_t t1,uint32_t t2,uint32_t delay);
void set_object_transmittance(uint32_t gain,uint32_t t1,uint32_t t2,uint32_t delay);
void calc_all_transmittance_value(void);

#endif /* CALC_H_ */
