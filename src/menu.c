/*
 * menu.c
 *
 *  Created on: May 18, 2016
 *      Author: kamil
 */
#include "stdint.h"
#include "graphic.h"
#include "KS0108.h"
#include "KS0108-STM32.h"
#include "stdbool.h"



uint8_t menu_position=5;
uint8_t menu_type;
extern bool AUTO_MODE;

void show_menu()
{

	switch(menu_type){
	case 0: //wybor trybu pracy
		GLCD_GoToReversed(127,7);
		GLCD_WriteStringNeg("Tryb pracy:");
		GLCD_GoToReversed(127,6);
		GLCD_WriteStringNeg("Automatyczny");
		GLCD_GoToReversed(127,5);
		GLCD_WriteStringNeg("Manualny");
		GLCD_GoToReversed(0,menu_position);
		GLCD_WriteStringNeg("#");
		break;
	case 1:
		GLCD_GoToReversed(127,7);
		GLCD_WriteStringNeg("AUTO");
		GLCD_GoToReversed(127,6);
		GLCD_WriteStringNeg("Symulacja obiektu");
		GLCD_GoToReversed(127,5);
		GLCD_WriteStringNeg("Stan zaklocen:");
		GLCD_GoToReversed(127,4);
		GLCD_WriteStringNeg("1:");
		GLCD_GoToReversed(127,3);
		GLCD_WriteStringNeg("2:");
		break;
	case 2:
		GLCD_GoToReversed(127,7);
		GLCD_WriteStringNeg("MANUAL");
		GLCD_GoToReversed(127,6);
		GLCD_WriteStringNeg("Poziom wyjsc");
		GLCD_GoToReversed(127,5);
		GLCD_WriteStringNeg("1:");
		GLCD_GoToReversed(127,4);
		GLCD_WriteStringNeg("2:");

		break;
	}

}
uint8_t get_menu_position()
{
	return menu_position;
}
void incr_menu_position()
{
	switch(menu_type)
	{
		case 0:
		{
			menu_position++;
			if(menu_position==7)
				menu_position=6;
			break;
		}
		case 1:
		{
			menu_position=6;
			break;
		}
		case 2:
		{
			menu_position=6;
			break;
		}
	}

}
void decr_menu_position()
{
	switch(menu_type)
		{
			case 0:
				menu_position--;
				if(menu_position==4)
				menu_position=5;
				break;
			case 1:
			{
				menu_position=6;
				break;
			}
			case 2:
			{
				menu_position=6;
				break;
			}
		}
}
void select_menu()
{
	switch(menu_type)
	{
	case 0:
		switch(menu_position)
		{
		case 6:
			AUTO_MODE=true;
			menu_type=1;
			break;
		case 5:
			AUTO_MODE=false;
			menu_type=2;
			break;
		}
	}
}
void back_menu()
{
	switch(menu_type)
	{
	case 1:
		menu_type=0;
		break;
	case 2:
		menu_type=0;
		break;
	}
}

