//-------------------------------------------------------------------------------------------------
// Universal KS0108 driver library
// STM32 MCU low-level driver
// (c) Rados³aw Kwiecieñ, radek@dxp.pl
//-------------------------------------------------------------------------------------------------
#include "stm32f3xx_hal.h"
#include "KS0108-STM32.h"
#include "stdbool.h"

#define KS0108_PORT_DATA  GPIOB

#define KS0108_RS_PIN    GPIO_Pin_10
#define KS0108_RW_PIN    GPIO_Pin_11
#define KS0108_EN_PIN    GPIO_Pin_12
#define KS0108_RS_PORT    GPIOC
#define KS0108_RW_PORT    GPIOC
#define KS0108_EN_PORT    GPIOC

#define KS0108_RST_PORT    GPIOC
#define KS0108_RST_PIN    GPIO_Pin_15


#define KS0108_CS1_PIN   GPIO_Pin_13
#define KS0108_CS2_PIN   GPIO_Pin_14
#define KS0108_CS1_PORT   GPIOC
#define KS0108_CS2_PORT   GPIOC

#define KS0108_D0_PORT    	GPIOD
#define KS0108_D0_PIN		GPIO_Pin_2
#define KS0108_D1_PORT    	GPIOB
#define KS0108_D1_PIN		GPIO_Pin_4
#define KS0108_D2_PORT    	GPIOB
#define KS0108_D2_PIN		GPIO_Pin_5
#define KS0108_D3_PORT    	GPIOB
#define KS0108_D3_PIN		GPIO_Pin_6
#define KS0108_D4_PORT    	GPIOB
#define KS0108_D4_PIN		GPIO_Pin_7
#define KS0108_D5_PORT    	GPIOB
#define KS0108_D5_PIN		GPIO_Pin_3
#define KS0108_D6_PORT    	GPIOB
#define KS0108_D6_PIN		GPIO_Pin_8
#define KS0108_D7_PORT    	GPIOB
#define KS0108_D7_PIN		GPIO_Pin_9

#define DISPLAY_STATUS_BUSY	0x80
static uint8_t halfScreen=63;
extern uint8_t screen_x;
extern uint8_t screen_y;
bool first;


GPIO_InitTypeDef GPIO_InitStructure;

//-------------------------------------------------------------------------------------------------
// Delay function /for 8MHz/
//-------------------------------------------------------------------------------------------------
void GLCD_Delay(void)
{
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
}
//-------------------------------------------------------------------------------------------------
// Enalbe Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_EnableController(unsigned char controller)
{
switch(controller){
	case 0 : 	HAL_GPIO_WritePin(KS0108_CS2_PORT, KS0108_CS2_PIN,GPIO_PIN_SET);
				HAL_GPIO_WritePin(KS0108_CS1_PORT, KS0108_CS1_PIN,GPIO_PIN_RESET);
				break;
	case 1 : 	HAL_GPIO_WritePin(KS0108_CS1_PORT, KS0108_CS1_PIN,GPIO_PIN_SET);
				HAL_GPIO_WritePin(KS0108_CS2_PORT, KS0108_CS2_PIN,GPIO_PIN_RESET);
				break;

	}
}
//-------------------------------------------------------------------------------------------------
// Disable Controller (0-2)
//-------------------------------------------------------------------------------------------------
void GLCD_DisableController(unsigned char controller)
{
switch(controller){
	case 0 : HAL_GPIO_WritePin(KS0108_CS1_PORT, KS0108_CS1_PIN,GPIO_PIN_SET); break;
	case 1 : HAL_GPIO_WritePin(KS0108_CS2_PORT, KS0108_CS2_PIN,GPIO_PIN_SET); break;

	}
}
//-------------------------------------------------------------------------------------------------
// Read Status byte from specified controller (0-2)
//-------------------------------------------------------------------------------------------------
/*NOT USED
 *
unsigned char GLCD_ReadStatus(unsigned char controller)
{
unsigned char status;

GPIO_StructInit(&GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin = 0xFF << KS0108_D0;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init(KS0108_PORT, &GPIO_InitStructure);

GPIO_SetBits(KS0108_PORT, KS0108_RW);
GPIO_ResetBits(KS0108_PORT, KS0108_RS);
GLCD_EnableController(controller);
GLCD_Delay();
GPIO_SetBits(KS0108_PORT, KS0108_EN);
GLCD_Delay();
status = ((GPIO_ReadInputData(KS0108_PORT) >> KS0108_D0) & 0xFF);
GPIO_ResetBits(KS0108_PORT, KS0108_EN);
GLCD_DisableController(controller);
return status;
}
*/

//-------------------------------------------------------------------------------------------------
// Write command to specified controller
//-------------------------------------------------------------------------------------------------
void GLCD_WriteCommand(unsigned char commandToWrite, uint8_t controller)
{

//if(controller)
//	HAL_GPIO_WritePin(KS0108_CS1_PORT,KS0108_CS1_PIN,GPIO_PIN_RESET);
//else
//	HAL_GPIO_WritePin(KS0108_CS2_PORT,KS0108_CS2_PIN,GPIO_PIN_RESET);

GLCD_EnableController(controller);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_RS_PORT, KS0108_RS_PIN,GPIO_PIN_RESET);
HAL_GPIO_WritePin(KS0108_RW_PORT, KS0108_RW_PIN,GPIO_PIN_RESET);
GLCD_Delay();
GLCD_SetCommandToPorts(commandToWrite);
HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_SET);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_RESET);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_CS2_PORT,KS0108_CS2_PIN,GPIO_PIN_SET);
HAL_GPIO_WritePin(KS0108_CS2_PORT,KS0108_CS2_PIN,GPIO_PIN_SET);
GLCD_Delay();
GLCD_SetCommandToPorts(0x00);
//HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_SET);
}

//-------------------------------------------------------------------------------------------------
// Read data from current position
//-------------------------------------------------------------------------------------------------
/*NOT  USED
 *
unsigned char GLCD_ReadData(void)
{
unsigned char tmp;
while(GLCD_ReadStatus(screen_x / 64)&DISPLAY_STATUS_BUSY);
GPIO_StructInit(&GPIO_InitStructure);  
GPIO_InitStructure.GPIO_Pin = 0xFF << KS0108_D0;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init(KS0108_PORT, &GPIO_InitStructure);

GPIO_SetBits(KS0108_PORT, KS0108_RS | KS0108_RW);

GLCD_EnableController(screen_x / 64);
GLCD_Delay();
GPIO_SetBits(KS0108_PORT, KS0108_EN);
GLCD_Delay();
tmp = ((GPIO_ReadInputData(KS0108_PORT) >> KS0108_D0) & 0xFF);
GPIO_ResetBits(KS0108_PORT, KS0108_EN);
GLCD_DisableController(screen_x / 64);
screen_x++;
return tmp;
}
*/
//-------------------------------------------------------------------------------------------------
// Write data to current position
//-------------------------------------------------------------------------------------------------
void GLCD_WriteData(unsigned char dataToWrite)
{

//if(!first)
//{
//	first=1;
//	screen_x=127;
//}


//if(screen_x>halfScreen)
//	HAL_GPIO_WritePin(KS0108_CS1_PORT,KS0108_CS1_PIN,GPIO_PIN_RESET);
//else
//	HAL_GPIO_WritePin(KS0108_CS2_PORT,KS0108_CS2_PIN,GPIO_PIN_RESET);

	if(screen_x>63)
	GLCD_EnableController(1);
	if(screen_x<=63)
	GLCD_EnableController(0);
	GLCD_Delay();
HAL_GPIO_WritePin(KS0108_RW_PORT, KS0108_RW_PIN,GPIO_PIN_RESET);
HAL_GPIO_WritePin(KS0108_RS_PORT, KS0108_RS_PIN,GPIO_PIN_SET);
GLCD_Delay();
GLCD_SetCommandToPorts(dataToWrite);
HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_SET);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_RESET);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_CS1_PORT,KS0108_CS1_PIN,GPIO_PIN_SET);
HAL_GPIO_WritePin(KS0108_CS2_PORT,KS0108_CS2_PIN,GPIO_PIN_SET);
GLCD_Delay();
GLCD_SetCommandToPorts(0x00);
screen_x++;
}
void GLCD_WriteData_Reversed(unsigned char dataToWrite)
{

//if(!first)
//{
//	first=1;
//	screen_x=127;
//}
//HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_SET);
//GLCD_Delay();
//HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_SET);
//if(screen_x>halfScreen)
//{
//
//	HAL_GPIO_WritePin(KS0108_CS2_PORT,KS0108_CS2_PIN,GPIO_PIN_RESET);
//}
//else
//{
//
//	HAL_GPIO_WritePin(KS0108_CS1_PORT,KS0108_CS1_PIN,GPIO_PIN_RESET);
//}
if(screen_x>63)
GLCD_EnableController(1);
if(screen_x<=63)
GLCD_EnableController(0);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_RW_PORT, KS0108_RW_PIN,GPIO_PIN_RESET);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_RS_PORT, KS0108_RS_PIN,GPIO_PIN_SET);
GLCD_Delay();

GLCD_SetCommandToPorts(dataToWrite);

GLCD_Delay();
HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_SET);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_EN_PORT, KS0108_EN_PIN,GPIO_PIN_RESET);
GLCD_Delay();
HAL_GPIO_WritePin(KS0108_CS1_PORT,KS0108_CS1_PIN,GPIO_PIN_SET);
HAL_GPIO_WritePin(KS0108_CS2_PORT,KS0108_CS2_PIN,GPIO_PIN_SET);
GLCD_Delay();
screen_x--;
GLCD_SetCommandToPorts(0x00);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
void GLCD_InitializePorts(void)
{

GPIO_InitTypeDef GPIO_InitStructure;

GPIO_InitStructure.Pin = KS0108_CS1_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_CS1_PORT, &GPIO_InitStructure);
HAL_GPIO_WritePin(KS0108_CS1_PORT,KS0108_CS1_PIN,GPIO_PIN_SET);

GPIO_InitStructure.Pin = KS0108_CS2_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_CS2_PORT, &GPIO_InitStructure);
HAL_GPIO_WritePin(KS0108_CS2_PORT,KS0108_CS2_PIN,GPIO_PIN_SET);

GPIO_InitStructure.Pin = KS0108_RST_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_RST_PORT, &GPIO_InitStructure);
HAL_GPIO_WritePin(KS0108_RST_PORT,KS0108_RST_PIN,GPIO_PIN_SET);




GPIO_InitStructure.Pin = KS0108_RS_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_RS_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_RW_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_RW_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_EN_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_EN_PORT, &GPIO_InitStructure);
HAL_GPIO_WritePin(KS0108_EN_PORT,KS0108_EN_PIN,GPIO_PIN_SET);

GPIO_InitStructure.Pin = KS0108_D0_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_D0_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_D1_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_D1_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_D2_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_D2_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_D3_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_D3_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_D4_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_D4_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_D5_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_D5_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_D6_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_D6_PORT, &GPIO_InitStructure);

GPIO_InitStructure.Pin = KS0108_D7_PIN;
GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
GPIO_InitStructure.Pull = GPIO_NOPULL;
HAL_GPIO_Init(KS0108_D7_PORT, &GPIO_InitStructure);

screen_x=127;
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------
unsigned char GLCD_ReadByteFromROMMemory(char * ptr)
{
  return *(ptr);
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------

void GLCD_SetCommandToPorts(unsigned char commandToWrite)
{
	HAL_GPIO_WritePin(KS0108_D0_PORT,KS0108_D0_PIN,(commandToWrite&0x01));
	HAL_GPIO_WritePin(KS0108_D1_PORT,KS0108_D1_PIN,((commandToWrite&0x02)>>1));
	HAL_GPIO_WritePin(KS0108_D2_PORT,KS0108_D2_PIN,((commandToWrite&0x04)>>2));
	HAL_GPIO_WritePin(KS0108_D3_PORT,KS0108_D3_PIN,((commandToWrite&0x08)>>3));
	HAL_GPIO_WritePin(KS0108_D4_PORT,KS0108_D4_PIN,((commandToWrite&0x10)>>4));
	HAL_GPIO_WritePin(KS0108_D5_PORT,KS0108_D5_PIN,((commandToWrite&0x20)>>5));
	HAL_GPIO_WritePin(KS0108_D6_PORT,KS0108_D6_PIN,((commandToWrite&0x40)>>6));
	HAL_GPIO_WritePin(KS0108_D7_PORT,KS0108_D7_PIN,((commandToWrite&0x80)>>7));


}
//void GLCD_Bitmap(char * bmp, unsigned char x, unsigned char y, unsigned char dx, unsigned char dy)
//{
//	unsigned char i,j;
//	for(j=0;j<dy/8;j++)
//	{
//		GLCD_GoTo(x,y+j);
//		for(i=0;i<dx;i++)
//		{
//			GLCD_WriteData(GLCD_ReadByteFromROMMemory(bmp++))
//		}
//	}
//}
