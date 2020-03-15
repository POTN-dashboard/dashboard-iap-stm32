#include "flash.h"




static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t Address = 0, PAGEError = 0;

extern uint32_t FLASH_ADDR;

//把接受到的 数据 转为小端 存储在 字中  每次转换一页  一页大小为1024字节
void transform(ST_Data *FLASH_DATA){

	if(FLASH_DATA->DATA_8_LEN >= FLASH_PAGE_SIZE){				
		for(int i = 0; i < FLASH_PAGE_SIZE/4 ; i++){				
			FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] = 0;							//STM32 FLASH 的读写是大端模式
			FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] += FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_HEAD+3];
			FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] <<= 8;
			FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] += FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_HEAD+2];
			FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] <<= 8;
			FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] += FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_HEAD+1];
			FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] <<= 8;
			FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] += FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_HEAD];	
			FLASH_DATA->DATA_32_INDEX ++;
			FLASH_DATA->DATA_8_LEN -= 4;
			FLASH_DATA->DATA_8_INDEX_HEAD += 4;
			if(FLASH_DATA->DATA_8_INDEX_HEAD >= MAX_round_queue) FLASH_DATA->DATA_8_INDEX_HEAD -= MAX_round_queue;  //循环队列					
		}
		FLASH_DATA->DATA_32_INDEX = 0;
	}
}

//把接受到的 数据 转为小端 存储在 字中 转换剩下不足一页的
void transform_extra(ST_Data *FLASH_DATA){
	
	uint32_t extra_byte = (FLASH_DATA->TOTAL_BYTE)%FLASH_PAGE_SIZE;          //最后一页有多少个字节
	
	while(FLASH_DATA->DATA_8_LEN%4 != 0){			 										//补满 字节数 为4 的整数
		FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_END] = 0xff;
		FLASH_DATA->DATA_8_INDEX_END++;
		FLASH_DATA->DATA_8_LEN++;
		extra_byte++;
	}
	
	for(int i = 0; i < extra_byte/4 ; i++){
		FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] = 0;							//STM32 FLASH 的读写是大端模式
		FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] += FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_HEAD+3];
		FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] <<= 8;
		FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] += FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_HEAD+2];
		FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] <<= 8;
		FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] += FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_HEAD+1];
		FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] <<= 8;
		FLASH_DATA->DATA_32[FLASH_DATA->DATA_32_INDEX] += FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_HEAD];	
		FLASH_DATA->DATA_32_INDEX ++;
		FLASH_DATA->DATA_8_LEN -= 4;
		FLASH_DATA->DATA_8_INDEX_HEAD += 4;
		if(FLASH_DATA->DATA_8_INDEX_HEAD >= MAX_round_queue) FLASH_DATA->DATA_8_INDEX_HEAD -= MAX_round_queue;  //循环队列
	}

}




void DATA32_Init(uint32_t *data){

	for(int i = 0; i < 256 ; i++){
		data[i] = 0xffffffff;
	}
}


void FLASH_STRUCT_Init(ST_Data *FLASH_DATA){

	FLASH_DATA->DATA_32_INDEX = 0;
	FLASH_DATA->DATA_8_INDEX_END = 0;
	FLASH_DATA->DATA_8_INDEX_HEAD = 0;
	FLASH_DATA->DATA_8_LEN = 0;
	
	FLASH_DATA->PACK_NUM = 1;
	FLASH_DATA->PACK_NUM_PC = 0;
	
	FLASH_DATA->TOTAL_BYTE = 0;
	FLASH_DATA->TOTAL_PACK = 0;
	FLASH_DATA->TOTAL_PAGE = 0;
//-------------------------------------	
//	FLASH_DATA->TOTAL_BYTE = 314;                   
//	if((FLASH_DATA->TOTAL_BYTE)%63 == 0)
//		FLASH_DATA->TOTAL_PACK = (FLASH_DATA->TOTAL_BYTE)/63;
//	else 
//		FLASH_DATA->TOTAL_PACK = ((FLASH_DATA->TOTAL_BYTE)/63)+1;
//-------------------------------------	
	
	DATA32_Init(FLASH_DATA->DATA_32);
	
	for(int i = 0; i < (MAX_round_queue); i++){
		FLASH_DATA->DATA_8[i] = 0;	
	}
	
	FLASH_ADDR = APP_FLASH_START;

}


uint8_t FLASH_ReadByte(uint32_t faddr)
{
	return *(__IO uint8_t*)faddr; 
}


uint32_t FLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr; 
}

//从指定位置开始读
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:字节(8位)数
void Flash_ReadData(uint32_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)   	
{
	uint16_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=FLASH_ReadByte(ReadAddr);//读1个字节
		ReadAddr+=1;// 地址偏移+1
	}
}

//写一个字
// WriteAddr  写地址的 页号
//	Word  要写入的 字	
//  WordNum  这个字在 这一页的 序号  一页共有0~255个字  1K=1024   1024/4 = 256
void Flash_WriteWord(uint32_t WriteAddr,uint32_t Word,uint16_t WordNum){
	uint16_t i;
	uint32_t Data[256];
	uint32_t ReadAddr = WriteAddr;
	
	
	for(i = 0;i < 256; i++){
		Data[i] = FLASH_ReadWord(ReadAddr);						// 先读
		ReadAddr += 4;   //地址 偏移 +4		
	}
	if(WordNum > 255) WordNum = 255;						
	Data[WordNum] = Word;														//修改后
	Flash_WriteData(WriteAddr,Data);								//再写
		
}


//写一页  1K = 1024 Byte
//WriteAddr 	页号
//Data 这一页的数据
void Flash_WriteData(uint32_t WriteAddr,uint32_t *Data){
	
	uint32_t i = 0;

	   /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = WriteAddr;
  EraseInitStruct.NbPages     = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PAGEError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
    /* Infinite loop */
    while (1)
    {
      /* Make LED2 blink (100ms on, 2s off) to indicate error in Erase operation */
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);	
      HAL_Delay(100);	
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);	
      HAL_Delay(2000);
    }
  }

  /* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  Address = WriteAddr;

  while (Address < WriteAddr+FLASH_PAGE_SIZE)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data[i]) == HAL_OK)
    {
      Address = Address + 4;
			i++;
    }
    else
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
      while (1)
      {
        /* Make LED2 blink (100ms on, 2s off) to indicate error in Write operation */
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);	
        HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);	
        HAL_Delay(2000);
      }
    }
  }
 
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock(); 

}




