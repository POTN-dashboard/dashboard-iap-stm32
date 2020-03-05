/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "gpio.h"



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h" //?????????
#include "flash.h"


#define FLASH_ADDR ADDR_FLASH_PAGE_127						//写入页的地址


ST_Data FLASH_Data;
ST_Data *FLASH_DATA = &FLASH_Data;



extern USBD_HandleTypeDef hUsbDeviceFS; //????USB????
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t send_buf[64] = {0};
uint8_t USB_Recive_Buffer[64] = {0}; //USB????


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Variable used for Erase procedure*/





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static uint8_t Status = 0;
  /* USER CODE END 1 */
  
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	
	FLASH_STRUCT_Init(FLASH_DATA);
	
  /* USER CODE END 2 */
 

	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		send_buf[0] = UPGRED_READY_PACK;   //初始化完成 通知上位机 升级就绪
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
		send_buf[0] = 0;
		
		
		if(USB_Recive_Buffer[0] != 0 && USB_Recive_Buffer[0] == FLASH_DATA->PACK_NUM){
			
			switch (Status){
				
				case 0: 																												
					break;
				
				
				case 1:
					
					break;
			
			}
			
			
			
			if(FLASH_DATA->PACK_NUM != FLASH_DATA->TOTAL_PACK || (FLASH_DATA->TOTAL_BYTE%63) == 0){  //不是最后一包 或最后一包刚好装满
					for(int i = 1; i < 64 ; i++){
						FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_END] = USB_Recive_Buffer[i];
						FLASH_DATA->DATA_8_INDEX_END++;
						if(FLASH_DATA->DATA_8_INDEX_END >= MAX_round_queue) FLASH_DATA->DATA_8_INDEX_END -= MAX_round_queue;   //循环队列
						
						FLASH_DATA->DATA_8_LEN++;
						
						USB_Recive_Buffer[i] = 0;								
					}			
			}
			
			else {																							//最后一包 且 未装满
				for(int i = 1; i < ((FLASH_DATA->TOTAL_BYTE%63)+1); i++){
					FLASH_DATA->DATA_8[FLASH_DATA->DATA_8_INDEX_END] = USB_Recive_Buffer[i];
					FLASH_DATA->DATA_8_INDEX_END++;
					if(FLASH_DATA->DATA_8_INDEX_END >= MAX_round_queue) FLASH_DATA->DATA_8_INDEX_END -= MAX_round_queue;    //循环队列
					FLASH_DATA->DATA_8_LEN++;
					
					USB_Recive_Buffer[i] = 0;
				
				}	
			}			
			transform(FLASH_DATA);					//字节 转换为字 
			
			
		FLASH_DATA->PACK_NUM++;	
		}
		if((FLASH_DATA->PACK_NUM-1) == FLASH_DATA->TOTAL_PACK){
			transform_extra(FLASH_DATA);
			
			
			Flash_WriteData(FLASH_ADDR,FLASH_DATA->DATA_32);
			DATA32_Init(FLASH_DATA->DATA_32);
			FLASH_DATA->DATA_32_INDEX = 0;
			
			
			Flash_ReadData(FLASH_ADDR,send_buf,64);
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
			HAL_Delay(1000);
			Flash_ReadData(FLASH_ADDR+64,send_buf,64);
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
			HAL_Delay(1000);
			Flash_ReadData(FLASH_ADDR+128,send_buf,64);
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
			HAL_Delay(1000);
			Flash_ReadData(FLASH_ADDR+128+64,send_buf,64);
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
			HAL_Delay(1000);
			Flash_ReadData(FLASH_ADDR+128+128,send_buf,64);
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
			HAL_Delay(1000);
			Flash_ReadData(FLASH_ADDR+128*2+64,send_buf,64);
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));			
			
			
			FLASH_DATA->PACK_NUM = 0x01;	
		}
		
//			if(FLASH_DATA->DATA_32_INDEX == 128/4){
//				Flash_WriteData(FLASH_ADDR,FLASH_DATA->DATA_32);
//				DATA32_Init(FLASH_DATA->DATA_32);
//				FLASH_DATA->DATA_32_INDEX = 0;
//				
//				Flash_ReadData(FLASH_ADDR,send_buf,64);
//				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
//				HAL_Delay(1000);
//				Flash_ReadData(FLASH_ADDR+64,send_buf,64);
//				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
//			}
		
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

















/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
