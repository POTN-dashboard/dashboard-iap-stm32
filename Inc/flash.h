#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"


void Flash_ReadData(uint32_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);
void Flash_WriteData(uint32_t WriteAddr,uint32_t *Data);
void DATA32_Init(uint32_t *data);
void Flash_WriteWord(uint32_t WriteAddr,uint32_t Word,uint16_t WordNum);
void FLASH_STRUCT_Init(ST_Data *FLASH_DATA);
void transform(ST_Data *FLASH_DATA);
void transform_extra(ST_Data *FLASH_DATA);
#endif

