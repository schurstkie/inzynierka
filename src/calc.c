/*
 * calc.c
 *
 *  Created on: May 9, 2016
 *      Author: kamil
 */


//uint32_t configTransmittance
#include "stdint.h"

#define SAMPLE_TIME 0.01


extern uint16_t DACValues1[1000];
extern uint16_t DACValues2[1000];

float object_gain;
float object_t1;
float object_t2;
float object_delay;

float disrupt1_gain;
float disrupt1_t1;//Tz
float disrupt1_t2;
float disrupt1_delay;

float disrupt2_gain;
float disrupt2_t1;
float disrupt2_t2;
float disrupt2_delay;

float sample_time;

void calc_object_transmittance_value(uint16_t *in, uint16_t *out)
{
	int i;
	for(i=1;i<sizeof(out);i++)
	{
		out[i]=((object_gain*SAMPLE_TIME)/object_t1) * in[i-1] + out[i-1] - (SAMPLE_TIME/object_t1)*out[i-1];
	}
}
void calc_disruption1_transmittance_value(uint32_t *in, uint32_t *out)
{

}
void calc_disruption2_transmittance_value(uint32_t *in, uint32_t *out)
{

}
void set_disruption1_transmittance(uint32_t gain,uint32_t t1,uint32_t t2,uint32_t delay)
{

}
void set_disruption2_transmittance(uint32_t gain,uint32_t t1,uint32_t t2,uint32_t delay)
{

}
void set_object_transmittance(float gain,float t1,float t2,float delay)
{
object_gain=gain;
object_t1=t1;
object_t2=t2;
object_delay=delay;
}
