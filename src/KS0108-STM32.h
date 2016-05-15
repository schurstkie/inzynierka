

#ifndef KS0108-STM32_H_
#define KS0108-STM32_H_


void GLCD_SetCommandToPorts(unsigned char);
void GLCD_Delay(void);
void GLCD_EnableController(unsigned char);
void GLCD_DisableController(unsigned char controller);
void GLCD_WriteCommand(unsigned char commandToWrite, unsigned char controller);
void GLCD_WriteData(unsigned char dataToWrite);
void GLCD_WriteData_Reversed(unsigned char dataToWrite);
void GLCD_InitializePorts(void);
//void GLCD_Bitmap(char * bmp, unsigned char x, unsigned char y, unsigned char dx, unsigned char dy);
unsigned char GLCD_ReadByteFromROMMemory(char * ptr);

#endif
