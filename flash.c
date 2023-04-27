/*
 * flash.c
 *
 *  Created on: Mar 27, 2023
 *      Author: MT
 */
#include "debug.h"
#define PAGE_WRITE_START_ADDR  ((uint32_t)0x0800FF00) /* Start from 32K */
void Flash_Init(void){
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;
    USART_Printf_Init(115200);
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP |FLASH_FLAG_WRPRTERR);
}

void Flash_DeInit(void){
    FLASH_Lock();
    RCC->CFGR0 &= ~(uint32_t)RCC_HPRE_DIV2;
     __enable_irq();
    USART_Printf_Init(115200);
}

void Flash_Write(u16 *data, u16    size) {
    Flash_Init();
    FLASH_ErasePage(PAGE_WRITE_START_ADDR);
    for (int i =0; i < size; i ++)  FLASH_ProgramHalfWord((PAGE_WRITE_START_ADDR + (i<<1)), data[i]);

    Flash_DeInit();
}

void Flash_Read(u16 *data, u16 size) {
    Flash_Init();
    for (int i =0; i < size; i ++)  {
        data[i] = (*(__IO uint16_t*) (PAGE_WRITE_START_ADDR  + (i <<1)));
    }

    Flash_DeInit();
}



