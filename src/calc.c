/*
 * calc.c
 *
 *  Created on: May 9, 2016
 *      Author: kamil
 */


//uint32_t configTransmittance
#include "stdint.h"

#define SAMPLE_TIME 0.001


//extern uint16_t DACValues1[1000];
//extern uint16_t DACValues2[1000];

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

//float sample_time;
float out_last;
float out_last_disrupt1;
float out_last_disrupt2;
float out_last_all;
//uint16_t in_last;

uint16_t calc_object_transmittance_value(uint16_t in, uint16_t in_last, uint16_t out)
{
//	int i;
//	in_last=in;
	out_last=((object_gain*SAMPLE_TIME)/object_t1) * in_last + out_last - (SAMPLE_TIME/object_t1)*out_last;
	return out_last;

}
void calc_disruption1_transmittance_value(uint16_t in, uint16_t in_last, uint16_t out)
{
	out_last_disrupt1=((object_gain*SAMPLE_TIME)/object_t1) * in_last + out_last - (SAMPLE_TIME/object_t1)*out_last;
	return out_last_disrupt1;
}
void calc_disruption2_transmittance_value(uint16_t in, uint16_t in_last, uint16_t out)
{
	out_last_disrupt2=((object_gain*SAMPLE_TIME)/object_t1) * in_last + out_last - (SAMPLE_TIME/object_t1)*out_last;
	return out_last_disrupt2;
}
void set_disruption1_transmittance(float gain,float t1,float t2,float delay)
{
	disrupt1_gain=gain;
	disrupt1_t1=t1;
	disrupt1_t2=t2;
	disrupt1_delay=delay;
}
void set_disruption2_transmittance(float gain,float t1,float t2,float delay)
{
	disrupt2_gain=gain;
	disrupt2_t1=t1;
	disrupt2_t2=t2;
	disrupt2_delay=delay;
}
void set_object_transmittance(float gain,float t1,float t2,float delay)
{
object_gain=gain;
object_t1=t1;
object_t2=t2;
object_delay=delay;
}

uint16_t calc_sum()
{

	out_last_all=out_last+out_last_disrupt1+out_last_disrupt2;
	return out_last_all;

}
