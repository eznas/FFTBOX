#ifndef __MAX7219_H
#define __MAX7219_H

#include "debug.h"

#define SPI_IS_HARDWARE 1  //1：硬件SPI；0：软件模拟SPI

#define Max7219_pinDIN_GPIO GPIO_Pin_7
#define Max7219_pinCS_GPIO  GPIO_Pin_4
#define Max7219_pinCLK_GPIO GPIO_Pin_5
#define SPI_MISO_GPIO       GPIO_Pin_6//未使用到

#define Max7219_pinDIN GPIO_Pin_7
#define Max7219_pinCS  GPIO_Pin_4
#define Max7219_pinCLK GPIO_Pin_5

#define SPI_MOSI GPIO_Pin_7
#define SPI_CS  GPIO_Pin_4
#define SPI_CLK GPIO_Pin_5

void Max7219_GPIO_Init(void);
void Write_Max7219_byte(u8 data);
void Write_Max7219(u8 addr,u8 dat);
void Init_MAX7219(void);

#endif
