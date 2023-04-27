/*
 * flash.h
 *
 *  Created on: Mar 27, 2023
 *      Author: MT
 */

#ifndef USER_FLASH_H_
#define USER_FLASH_H_
#define PAGE_WRITE_START_ADDR  ((uint32_t)0x0800FF00) /* Start from 32K */
void Flash_Init(void);
void Flash_DeInit(void);
void Flash_Write(u16 *data, u16    size);
void Flash_Read(u16 *data, u16 size);

#endif /* USER_FLASH_H_ */
