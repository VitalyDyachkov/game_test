/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ERROR_SET -1
#define SET_OK 1
#define IN_PAST_SET 0
#define SET_ZX 1
#define RESET_ZX 0
#define X 'x'
#define Z '0'
#define CLR ' '
#define CLEAR_STRING 9

#define WIN 10
#define LOSS 20
#define DRAW 30
#define NEXT_STEP 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
xQueueHandle queue_usb;
xQueueHandle queue_in_usb;
uint8_t data_buff_input_usb[64];
uint8_t data_buff_to_usb[64];

struct msg_for_game{
uint8_t invite_string[64];
uint8_t game_field[6][10];	
}struct_for_send;


Control_CMD_t result_search;
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void Error_Handler(void);
void PrepareMove(Control_CMD_t move);
uint8_t CheckFild(uint8_t num_cell,uint8_t *ptr);
int ModifyGameField(uint8_t num_cell,char symbol);
uint8_t AlgoritmGame(uint8_t num_cell);
void SetMoveInMsg(uint8_t move, uint8_t symbol);
bool SearchWinComb(uint8_t num_cell);
void MoveComb(uint8_t num_cell,char source[],char destination[]);
void RemoveComb(uint8_t num_cell,char* source);
uint8_t SearchLineBlock(uint8_t num_cell);
void SeparateLine(uint8_t num_cell);
void ResetAllDataGame(void);
uint8_t z_coordinates[9] = {RESET_ZX};
uint8_t x_coordinates[9] = {RESET_ZX};

uint8_t abc_xy[9][2] = {{1,2},{3,2},{5,2},
												{1,3},{3,3},{5,3},
                         {1,4},{3,4},{5,4}};

uint8_t  step = 0;
struct Combination
{
uint8_t cnt_comb ;
char buff_comb[8][3];
};

struct Combination first_state = {8,
																	1,2,3,
																	1,5,9,
																	1,4,7,
																	2,5,8,
																	4,5,6,
																	7,5,3,
																	9,6,3,
																	9,8,7
																	};

struct Combination block_comb = {0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0
};

struct Combination norm_comb = {0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0,
																		0,0,0
};

struct msg_for_game msg_full;
bool flag_start_game = false;

uint8_t your_step_my_step[]       = {"YOUR MOVE   , MY MOVE   \r\n"};

uint8_t invite[]       = {"ENTER YOUR MOVE,OR SEND 'END'\r\n"};
uint8_t you_won[]        = {"YOU WON, CONGRATULATIONS!\r\n"};
uint8_t i_won[]          = {"YOU HAVE LOST, TRY AGAIN!\r\n"};
uint8_t draw[]           = {"DRAW, FRIENDSHIP WON!\r\n"};

uint8_t game_field[6][10] = {{"-A-B-C--\r\n"},
                             {"--------\r\n"},
                             {"| | | |1\r\n"},
                             {"| | | |2\r\n"},
                             {"| | | |3\r\n"},
                             {"--------\r\n"}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*отправка пакетов в USB*/
/*разбор пакета принятого в USB*/
void Task_Pars_USB (void *pvParametrs)
{
	
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = 50;
	portBASE_TYPE retVal;

for(;;)
	{

		retVal = xQueueReceive(queue_in_usb,&data_buff_input_usb, 0);
		if(retVal == pdPASS)
		{
				result_search = ParsCMD(data_buff_input_usb);
				switch (result_search)
				{
					/*установка скорости CAN*/
								case HELLO:
								{
									PrepareMove(HELLO);	
									/*ответим плодтверждением*/							
									break;
								}		
								case A1:
								{														
									/*ответим плодтверждением*/		
									PrepareMove(A1);									
									break;
								}
								case A2:
								{
									/*ответим плодтверждением*/	
									PrepareMove(A2);									
									break;
								}
								case A3:
								{
									/*ответим плодтверждением*/
									PrepareMove(A3);									
									break;
								}
								case B1:
								{	
									/*ответим плодтверждением*/
									PrepareMove(B1);																
									break;
								}
								case B2:
								{
									/*ответим плодтверждением*/							
									PrepareMove(B2);
									break;
								}
								case B3:
								{
									
									/*ответим плодтверждением*/
									PrepareMove(B3);									
									break;
								}	
								case C1:
								{	
									/*ответим плодтверждением*/
									PrepareMove(C1);																
									break;
								}
								case C2:
								{
									/*ответим плодтверждением*/
								  PrepareMove(C2);									
									break;
								}
								case C3:
								{
									/*ответим плодтверждением*/
									PrepareMove(C3);									
									break;
								}								
								case END:
								{
									flag_start_game = false;
									/*ответим плодтверждением*/
									PrepareMove(END);												
									break;
								}			
								
					default :
					{
						break;
					}
				}

		}
		osDelay(1);
	}

}


void Task_Send_USB (void *pvParametrs)
{
	portBASE_TYPE retVal;
	uint8_t data_to_usb[64];
	uint8_t answer = 0;
	for(;;)
	{
//	  retVal = xQueueReceive(queue_usb,&data_to_usb, strlen(queue_usb));
//		if(retVal == pdPASS )
//		{
//			CDC_Transmit_FS(game_field[0],sizeof(game_field));
//			do{
//						answer = CDC_Transmit_FS(data_to_usb,strlen((char*)data_to_usb));
//			  }while(answer != USBD_OK);

//			memset(data_to_usb,0,64);
//		}
			  retVal = xQueueReceive(queue_usb,&msg_full, strlen(queue_usb));
		if(retVal == pdPASS )
		{
			CDC_Transmit_FS(msg_full.game_field[0],sizeof(msg_full.game_field));
			do{
						answer = CDC_Transmit_FS(msg_full.invite_string,strlen((char*)msg_full.invite_string));
			  }while(answer != USBD_OK);

			memset(data_to_usb,0,64);
		}
		else
		{

		}
    osDelay(1);	
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	//queue_usb = xQueueCreate(8,sizeof(data_buff_to_usb));
	queue_usb = xQueueCreate(8,sizeof(struct_for_send));
	queue_in_usb = xQueueCreate(8,sizeof(data_buff_input_usb));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	xTaskCreate( Task_Send_USB, (const char *) "Task_Send_USB", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( Task_Pars_USB, (const char *) "Task_Pars_USB", configMINIMAL_STACK_SIZE, NULL, 0, NULL );
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void PrepareMove(Control_CMD_t move)
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	uint8_t result_step = 0;
	
	switch (move) {
				case HELLO:
								/*вернем поле в исходное состояние*/
				
								ResetAllDataGame();
				
								memcpy(msg_full.game_field,game_field,sizeof(game_field));
								memcpy(msg_full.invite_string,invite,sizeof(invite));			
				
								xQueueSend(queue_usb,&msg_full,xHigherPriorityTaskWoken);	
								
				break;
				case A1:
					/*если не занято ноликом*/
								if(z_coordinates[0] != SET_ZX)
								{
									if(x_coordinates[0] != SET_ZX)
									{
										result_step = AlgoritmGame(1);
									}	
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}
								}
								else
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}								
				break;
				case A2:
					/*если не занято ноликом*/
								if(z_coordinates[3] != SET_ZX)
								{
									if(x_coordinates[3] != SET_ZX)
									{
										result_step = AlgoritmGame(4);									
									}	
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}
								}							
								else
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}	
				break;
				case A3:
					/*если не занято ноликом*/
								if(z_coordinates[6] != SET_ZX)
								{
									if(x_coordinates[6] != SET_ZX)
									{
										result_step = AlgoritmGame(7);									
									}
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}									
								}							
								else	
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}
				break;
				case B1:
					/*если не занято ноликом*/
								if(z_coordinates[1] != SET_ZX)
								{
									if(x_coordinates[1] != SET_ZX)
									{									
										result_step = AlgoritmGame(2);
									}	
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}
								}							
								else
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}			
				break;
				case B2:
					/*если не занято ноликом*/
								if(z_coordinates[4] != SET_ZX)
								{
									if(x_coordinates[4] != SET_ZX)
									{
										result_step = AlgoritmGame(5);									
									}	
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}
								}							
								else
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}
				break;
				case B3:
					/*если не занято ноликом*/
								if(z_coordinates[7] != SET_ZX)
								{
									if(x_coordinates[7] != SET_ZX)
									{
										result_step = AlgoritmGame(8);								
									}	
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}
								}							
								else
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}
				break;
				case C1:
					/*если не занято ноликом*/
								if(z_coordinates[2] != SET_ZX)
								{
									if(x_coordinates[2] != SET_ZX)
									{
										result_step = AlgoritmGame(3);											
									}	
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}
								}							
								else
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}
				break;
				case C2:
					/*если не занято ноликом*/
								if(z_coordinates[5] != SET_ZX)
								{
									if(x_coordinates[5] != SET_ZX)
									{
										result_step = AlgoritmGame(6);									
									}	
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}
								}							
								else
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}
				break;
				case C3:
					/*если не занято ноликом*/
								if(z_coordinates[8] != SET_ZX)
								{
									if(x_coordinates[8] != SET_ZX)
									{
										result_step = AlgoritmGame(9);										
									}	
									else
									{
										CDC_Transmit_FS(invite,sizeof(invite));
									}
								}							
								else
								{
									CDC_Transmit_FS(invite,sizeof(invite));
								}
				break;
				case END:
								/*вернем поле в исходное состояние*/			
								ResetAllDataGame();										
				break;
				default:
					
				break;
		
	}
	if(result_step != 0)
	{
		switch (result_step) 
		{
			case WIN:
				
				memcpy(msg_full.game_field,game_field,sizeof(game_field));
				memcpy(msg_full.invite_string,i_won,sizeof(i_won));
			
				xQueueSend(queue_usb,&msg_full,xHigherPriorityTaskWoken);
			
				ResetAllDataGame();
				
				memcpy(msg_full.game_field,game_field,sizeof(game_field));
				memcpy(msg_full.invite_string,invite,sizeof(invite));			
				
			  xQueueSend(queue_usb,&msg_full,xHigherPriorityTaskWoken);	
			
				break;
			case DRAW:
				
				memcpy(msg_full.game_field,game_field,sizeof(game_field));
				memcpy(msg_full.invite_string,draw,sizeof(draw));
			
				xQueueSend(queue_usb,&msg_full,xHigherPriorityTaskWoken);
			
				ResetAllDataGame();
				
				memcpy(msg_full.game_field,game_field,sizeof(game_field));
				memcpy(msg_full.invite_string,invite,sizeof(invite));			
				
			  xQueueSend(queue_usb,&msg_full,xHigherPriorityTaskWoken);	
			
				break;
			case LOSS:	
				
				memcpy(msg_full.game_field,game_field,sizeof(game_field));
				memcpy(msg_full.invite_string,you_won,sizeof(you_won));			
				
			  xQueueSend(queue_usb,&msg_full,xHigherPriorityTaskWoken);		
				
			  ResetAllDataGame();
				
			  memcpy(msg_full.game_field,game_field,sizeof(game_field));
				memcpy(msg_full.invite_string,invite,sizeof(invite));			
				
				xQueueSend(queue_usb,&msg_full,xHigherPriorityTaskWoken);	
				
			break;		
			default:
				
				memcpy(msg_full.game_field,game_field,sizeof(game_field));
				memcpy(msg_full.invite_string,your_step_my_step,sizeof(your_step_my_step));			
				
			  xQueueSend(queue_usb,&msg_full,xHigherPriorityTaskWoken);	
				
			break;			
		}	
	}	

}
//bool ModifyGameField(uint8_t x,uint8_t y,char symbol)
int ModifyGameField(uint8_t num_cell,char symbol)
{
	int  result ;
	uint8_t copy_num_cell = num_cell - 1;
	/*получим координаты вставки*/
	uint8_t x = abc_xy[copy_num_cell][0];
  uint8_t y = abc_xy[copy_num_cell][1];
	/*проверим на возможность вставки символа в поле*/
	if(symbol == Z)
	{
		if(x_coordinates[copy_num_cell] != SET_ZX)
		{
			if(z_coordinates[copy_num_cell] != SET_ZX)
			{
				z_coordinates[copy_num_cell] = SET_ZX;
				game_field[y][x] = symbol;
				result = SET_OK;
				/*произведем очистку буфера блокируемых комбинаций*/
				RemoveComb(num_cell,&block_comb.buff_comb[0][0]);
				/*в массив внесем ход*/
				SetMoveInMsg(copy_num_cell,Z);
			}
			/*символ ранее установлен*/
			else
			{
				result = IN_PAST_SET;
			}			
		}
		/*ячейка занята соперником*/
		else
		{
			/*вернуть признак невозможности установки*/
			result = ERROR_SET;
		}
	}
	/*проверим на возможность вставки символа в поле*/
	else if (symbol == X)
	{
		if(z_coordinates[copy_num_cell] != SET_ZX)
		{
			if(x_coordinates[copy_num_cell] != SET_ZX)
			{
				x_coordinates[copy_num_cell] = SET_ZX;
				game_field[y][x] = symbol;
				result = SET_OK;
				/*произведем перемещение в буфер блокируемых комбинаций*/
				MoveComb(num_cell,&norm_comb.buff_comb[0][0],&block_comb.buff_comb[0][0]);
				/*произведем запись в буфер блокируемых комбинаций*/
				RemoveComb(num_cell,&norm_comb.buff_comb[0][0]);
				/*в массив внесем ход*/
				SetMoveInMsg(copy_num_cell,X);
			}
			/*символ ранее установлен*/
			else
			{
				result = IN_PAST_SET;
			}			
		}
		/*ячейка занята моим символом*/
		else
		{
			/*вернуть признак невозможности установки*/
			result = ERROR_SET;
		}
	}
	/*очистим поле*/
	else if (symbol == CLR)
	{
		x_coordinates[copy_num_cell] = RESET_ZX;
		z_coordinates[copy_num_cell] = RESET_ZX;
		game_field[y][x] = symbol;
		result = SET_OK;
	}
	return result;
}

uint8_t CheckFild(uint8_t num_cell,uint8_t *ptr)
{
	uint8_t result = 0;
	num_cell--;
	result = ptr[num_cell];
	
	return result;
}
uint8_t AlgoritmGame(uint8_t num_cell)
{
			/*правила игры*/
		/* 1)крестики в линию?
		    -блокируем*/
		/* 2)крестик в углу?
		    -если центр не занят занимаем
				-определим кратчайший путь и займем угол 
		   2.1 крестики в линию?
				-блокируем*/
		/* 3)символы в одну линию?
				-при диагональном совпадении ставим в ячейки кроме углов
		    -при совпадении в линию ставим в свободный угол*/
		/* 4)противник занял центр и угол?
		    -занимаем угол если не требуется блокировать*/
			//int cell_idx = rand() % 10 + 1;	
	
	int cell_idx = 0;
	/*проверим номер хода*/
	if(step == 0){
		/*сделаем ход*/
	  if(ModifyGameField(num_cell,X) == SET_OK)
		{
			SeparateLine(num_cell);

				if(num_cell == 5)
				{

				cell_idx = 3;									
				}
				else if(num_cell == 1 || 
		      num_cell == 3 || 
	        num_cell == 7 || 
	        num_cell == 9)
				{
						/*если первый шаг*/
						if(step == 0)
						{
							cell_idx = 5;									
						}
						else
						{
						
						}
					}
					else if(num_cell == 4 || 
									num_cell == 2 || 
									num_cell == 8 || 
									num_cell == 6)
					{
						
						/*если первый шаг*/
						if(step == 0)
						{
							cell_idx = 5;	
							
						}
						else
						{

						}
					}				
					ModifyGameField(cell_idx,Z);
				}
			}
		
	else
	{
		/*сделаем ход*/
	  if(ModifyGameField(num_cell,X) == SET_OK)
		{			
			cell_idx  = SearchLineBlock(num_cell);
			if(cell_idx == LOSS)
			{
				return LOSS;
			}
		}	
		/*необходимо блокировать*/		
		if(cell_idx != 0)
		{
			if(SearchWinComb(num_cell) == true)
			{
				return WIN;
			}	
			ModifyGameField(cell_idx,Z);
		}
		/*пробуем выиграть*/
		else
		{
			if(SearchWinComb(num_cell) == true)
			{
				return WIN;
			}
					/*победных комбинаций нет*/
					/*определим ход из этого определим стратегию*/
					switch (num_cell)
					{
						case 1:	
							/*1-2-3*/
							if(CheckFild(2,z_coordinates) == SET || CheckFild(2,x_coordinates) == SET)
							{
								if(CheckFild(3,x_coordinates) == SET || CheckFild(3,z_coordinates) == SET)
								{
									/*проверим наличие линии не по диагонали*/
										if((ModifyGameField(7,Z)) != SET_OK)
										{
											if((ModifyGameField(9,Z)) != SET_OK)
											{
											
											}
										}											
								}
							}
							/*1-4-7*/
							else if(CheckFild(4,z_coordinates) == SET)
							{															
								if(CheckFild(7,z_coordinates) == SET)
									{
										if((ModifyGameField(3,Z)) != SET_OK)
										{
											if((ModifyGameField(9,Z)) != SET_OK)
											{
														
											}
										}
									}															
							}
							/*1-5-9*/							
							else if(CheckFild(5,z_coordinates) == SET)
							{																					
								if(CheckFild(9,z_coordinates) == SET)
								{
											if((ModifyGameField(2,Z)) != SET_OK)
											{
												if((ModifyGameField(6,Z)) != SET_OK)
												{
													if((ModifyGameField(4,Z)) != SET_OK)
													{
														if((ModifyGameField(8,Z)) != SET_OK)
														{
											
														}
													}
												}
											}							
								}
							/*1-5->3||7*/
							else
							{
								if((ModifyGameField(3,Z)) != SET_OK)
								{
									if((ModifyGameField(7,Z)) != SET_OK)
									{
											
									}
								}
							}								
							}

							break;

						case 2:
							if(CheckFild(9,x_coordinates) == SET)
							{
								if((ModifyGameField(3,Z)) != SET_OK)
								{
									if((ModifyGameField(1,Z)) != SET_OK)
									{
										if((ModifyGameField(7,Z)) != SET_OK)
										{
										if((ModifyGameField(4,Z)) != SET_OK)
										{
													if((ModifyGameField(6,Z)) != SET_OK)
										{
											
										}								
										}											
										}
								  }
								}									
							}
							else if (CheckFild(6,x_coordinates) == SET)
							{
								if((ModifyGameField(4,Z)) != SET_OK)
										{
											
										}
							}								
							else if(CheckFild(7,x_coordinates))
							{
								if((ModifyGameField(1,Z)) != SET_OK)
								{
									if((ModifyGameField(3,Z)) != SET_OK)
									{
										if((ModifyGameField(9,Z)) != SET_OK)
										{
											
										}
								  }
								}
							}
							else if (CheckFild(4,x_coordinates) == SET)
							{
								if((ModifyGameField(6,Z)) != SET_OK)
										{
											
										}
							}
							else if (CheckFild(5,x_coordinates) == SET || CheckFild(5,z_coordinates) == SET)
							{
								if (CheckFild(8,x_coordinates) == SET || CheckFild(8,z_coordinates) == SET)
								{
										if((ModifyGameField(1,Z)) != SET_OK)
										{
												if((ModifyGameField(3,Z)) != SET_OK)
												{
													if((ModifyGameField(7,Z)) != SET_OK)
													{
														if((ModifyGameField(9,Z)) != SET_OK)
														{
												
														}
													}
												}
										}
								}								
							}
							break;
						case 3:
							/*3-2-1*/
							if(CheckFild(2,z_coordinates) == SET || CheckFild(2,x_coordinates) == SET)
							{
								if(CheckFild(1,x_coordinates) == SET || CheckFild(1,z_coordinates) == SET)
								{
									/*проверим наличие линии не по диагонали*/
										if((ModifyGameField(7,Z)) != SET_OK)
										{
											if((ModifyGameField(9,Z)) != SET_OK)
											{
											
											}
										}											
								}
							}
							/*3-6-9*/
							else if(CheckFild(6,z_coordinates) == SET)
							{															
								if(CheckFild(9,z_coordinates) == SET)
									{
										if((ModifyGameField(1,Z)) != SET_OK)
										{
											if((ModifyGameField(7,Z)) != SET_OK)
											{
														
											}
										}
									}															
							}
							/*3-5-7*/							
							else if(CheckFild(5,z_coordinates) == SET)
							{																					
								if(CheckFild(7,z_coordinates) == SET)
								{
											if((ModifyGameField(2,Z)) != SET_OK)
											{
												if((ModifyGameField(6,Z)) != SET_OK)
												{
													if((ModifyGameField(4,Z)) != SET_OK)
													{
														if((ModifyGameField(8,Z)) != SET_OK)
														{
											
														}
													}
												}
											}							
								}
							/*3-5->1||9*/
							else
							{
								if((ModifyGameField(1,Z)) != SET_OK)
								{
									if((ModifyGameField(9,Z)) != SET_OK)
									{
											
									}
								}
							}								
							}

						
							break;						
						case 4:
							if(CheckFild(3,x_coordinates) == SET)
							{
								if((ModifyGameField(1,Z)) != SET_OK)
								{
									if((ModifyGameField(7,Z)) != SET_OK)
									{
										if((ModifyGameField(9,Z)) != SET_OK)
										{
											
										}
								  }
								}									
							}
							else if (CheckFild(2,x_coordinates) == SET)
							{
								if((ModifyGameField(8,Z)) != SET_OK)
										{
											
										}
							}								
							else if(CheckFild(9,x_coordinates))
							{
								if((ModifyGameField(7,Z)) != SET_OK)
								{
									if((ModifyGameField(1,Z)) != SET_OK)
									{
										if((ModifyGameField(9,Z)) != SET_OK)
										{
											
										}
								  }
								}
							}
							else if (CheckFild(8,x_coordinates) == SET)
							{
								if((ModifyGameField(2,Z)) != SET_OK)
										{
											
										}
							}
							else if (CheckFild(5,x_coordinates) == SET || CheckFild(5,z_coordinates) == SET)
							{
								if (CheckFild(6,x_coordinates) == SET || CheckFild(6,z_coordinates) == SET)
								{
										if((ModifyGameField(1,Z)) != SET_OK)
										{
												if((ModifyGameField(3,Z)) != SET_OK)
												{
													if((ModifyGameField(7,Z)) != SET_OK)
													{
														if((ModifyGameField(9,Z)) != SET_OK)
														{
												
														}
													}
												}
										}
								}								
							}
							break;
						case 6:
							if(CheckFild(1,x_coordinates) == SET)
							{
								if((ModifyGameField(3,Z)) != SET_OK)
								{
									if((ModifyGameField(7,Z)) != SET_OK)
									{
										if((ModifyGameField(9,Z)) != SET_OK)
										{
											if((ModifyGameField(2,Z)) != SET_OK)
											{
												if((ModifyGameField(4,Z)) != SET_OK)
												{
													if((ModifyGameField(8,Z)) != SET_OK)
													{
											
													}
												}
											}
										}
								  }
								}									
							}
							else if (CheckFild(2,x_coordinates) == SET)
							{
										if((ModifyGameField(3,Z)) != SET_OK)
										{
												if((ModifyGameField(9,Z)) != SET_OK)
												{
													if((ModifyGameField(1,Z)) != SET_OK)
													{
														if((ModifyGameField(1,Z)) != SET_OK)
													{
											
													}
													}
												}
										}
							}								
							else if(CheckFild(7,x_coordinates))
							{
								if((ModifyGameField(7,Z)) != SET_OK)
								{
									if((ModifyGameField(1,Z)) != SET_OK)
									{
										if((ModifyGameField(3,Z)) != SET_OK)
										{
											
										}
								  }
								}
							}
							else if (CheckFild(4,x_coordinates) == SET)
							{
								if((ModifyGameField(2,Z)) != SET_OK)
										{
											
										}
							}
							else if (CheckFild(5,x_coordinates) == SET || CheckFild(5,z_coordinates) == SET)
							{
								if (CheckFild(4,x_coordinates) == SET || CheckFild(4,z_coordinates) == SET)
								{
										if((ModifyGameField(1,Z)) != SET_OK)
										{
												if((ModifyGameField(3,Z)) != SET_OK)
												{
													if((ModifyGameField(7,Z)) != SET_OK)
													{
														if((ModifyGameField(9,Z)) != SET_OK)
														{
												
														}
													}
												}
										}
								}								
							}
							break;
						case 7:

							/*7-4-1*/
							if(CheckFild(4,z_coordinates) == SET || CheckFild(4,x_coordinates) == SET)
							{
								if(CheckFild(1,x_coordinates) == SET || CheckFild(1,z_coordinates) == SET)
								{
									/*проверим наличие линии не по диагонали*/
										if((ModifyGameField(3,Z)) != SET_OK)
										{
											if((ModifyGameField(9,Z)) != SET_OK)
											{
											
											}
										}											
								}
							}
							/*7-8-9*/
							else if((CheckFild(8,z_coordinates) == SET) || (CheckFild(8,x_coordinates) == SET))
							{															
								if(CheckFild(9,z_coordinates) == SET)
									{
										if((ModifyGameField(1,Z)) != SET_OK)
										{
											if((ModifyGameField(3,Z)) != SET_OK)
											{
														
											}
										}
									}															
							}
							/*7-5-3*/							
							else if(CheckFild(5,z_coordinates) == SET)
							{																					
								if((CheckFild(3,z_coordinates) == SET) || (CheckFild(3,x_coordinates) == SET))
								{
											if((ModifyGameField(2,Z)) != SET_OK)
											{
												if((ModifyGameField(6,Z)) != SET_OK)
												{
													if((ModifyGameField(4,Z)) != SET_OK)
													{
														if((ModifyGameField(8,Z)) != SET_OK)
														{
											
														}
													}
												}
											}							
								}	
							/*7-5->1||9*/
							else
							{
								if((ModifyGameField(1,Z)) != SET_OK)
								{
									if((ModifyGameField(9,Z)) != SET_OK)
									{
											
									}
								}
							}									
							}
					
							break;
						case 8:
							if(CheckFild(1,x_coordinates) == SET)
							{
								if((ModifyGameField(7,Z)) != SET_OK)
								{
									if((ModifyGameField(3,Z)) != SET_OK)
									{
										if((ModifyGameField(9,Z)) != SET_OK)
										{
											
										}
								  }
								}									
							}
							else if (CheckFild(4,x_coordinates) == SET)
							{
								if((ModifyGameField(2,Z)) != SET_OK)
										{
											
										}
							}								
							else if(CheckFild(3,x_coordinates))
							{
								if((ModifyGameField(9,Z)) != SET_OK)
								{
									if((ModifyGameField(1,Z)) != SET_OK)
									{
										if((ModifyGameField(7,Z)) != SET_OK)
										{
											
										}
								  }
								}
							}
							else if (CheckFild(2,x_coordinates) == SET)
							{
								if((ModifyGameField(4,Z)) != SET_OK)
										{
											
										}
							}
							else if (CheckFild(5,x_coordinates) == SET || CheckFild(5,z_coordinates) == SET)
							{
								if (CheckFild(2,x_coordinates) == SET || CheckFild(2,z_coordinates) == SET)
								{
										if((ModifyGameField(1,Z)) != SET_OK)
										{
												if((ModifyGameField(3,Z)) != SET_OK)
												{
													if((ModifyGameField(7,Z)) != SET_OK)
													{
														if((ModifyGameField(9,Z)) != SET_OK)
														{
												
														}
													}
												}
										}
								}								
							}
							break;
						case 9:

							/*9-8-7*/
							if(CheckFild(8,z_coordinates) == SET || CheckFild(8,x_coordinates) == SET)
							{
								if(CheckFild(7,x_coordinates) == SET || CheckFild(7,z_coordinates) == SET)
								{
									/*проверим наличие линии не по диагонали*/
										if((ModifyGameField(1,Z)) != SET_OK)
										{
											if((ModifyGameField(3,Z)) != SET_OK)
											{
											
											}
										}											
								}
							}
							/*9-6-3*/
							else if(CheckFild(6,z_coordinates) == SET)
							{															
								if(CheckFild(3,z_coordinates) == SET)
									{
										if((ModifyGameField(1,Z)) != SET_OK)
										{
											if((ModifyGameField(7,Z)) != SET_OK)
											{
														
											}
										}
									}															
							}
							/*9-5-1*/							
							else if(CheckFild(5,z_coordinates) == SET)
							{																					
								if(CheckFild(1,z_coordinates) == SET)
								{
											if((ModifyGameField(2,Z)) != SET_OK)
											{
												if((ModifyGameField(6,Z)) != SET_OK)
												{
													if((ModifyGameField(4,Z)) != SET_OK)
													{
														if((ModifyGameField(8,Z)) != SET_OK)
														{
											
														}
													}
												}
											}							
								}
							/*9-5->3||7*/
							else
							{
								if((ModifyGameField(3,Z)) != SET_OK)
								{
									if((ModifyGameField(7,Z)) != SET_OK)
									{
											
									}
								}
							}								
							}

							break;
					}			
					
		}

}
	step++;
if(step == 5)
{
	return DRAW;
}
return NEXT_STEP;
}

void SeparateLine(uint8_t num_cell)
{
			char buff_comb[8][3];
			memcpy(buff_comb,first_state.buff_comb,sizeof(first_state.buff_comb));
			/*произведем поиск комбинации которую необходимо блокировать*/
			for(int i = 0 ; i < first_state.cnt_comb; i++)
			{
				for(int j = 0; j < 3;j++)
				{
					if(first_state.buff_comb[i][j] == num_cell)
					{
						/*переместим в структуру блокирования*/
						for(int k = 0 ; k < 3;k++)
						{
							block_comb.buff_comb[block_comb.cnt_comb][k] = first_state.buff_comb[i][k];
							buff_comb[i][k] = 0;
							
						}	
					
							/*инкрементируем счетчик данных в структуре*/
							block_comb.cnt_comb++;
					}
				}
			}
			
			/*произведем поиск нормальных комбинаций*/
			for(int i = 0 ; i < first_state.cnt_comb; i++)
			{		
				if(buff_comb[i][0] != 0)
				{					
						/*переместим в структуру блокирования*/
						for(int k = 0 ; k < 3;k++)
						{
							norm_comb.buff_comb[norm_comb.cnt_comb][k] = first_state.buff_comb[i][k];
						}						
							/*инкрементируем счетчик данных в структуре*/
						norm_comb.cnt_comb++;
				}
					
			 }
}

bool SearchWinComb(uint8_t num_cell)
{
	bool result;
				norm_comb.cnt_comb = 0;
					for(int i = 0 ; i < 8; i++)
					{
						for(int j = 0; j < 3;j++)
						{
							/*если не пусто призведем проверку на возможность победы*/
							if(norm_comb.buff_comb[i][j] != 0)
							{
								/*проверим поле на наличие нолика*/
								for(int k = 0; k < 3; k++)
								{
									if(CheckFild(norm_comb.buff_comb[i][k],z_coordinates) == SET)
									{
											norm_comb.cnt_comb++;
									}									
								}
								if(norm_comb.cnt_comb == 2)
								{
										/*произведем установку нолика*/
										for(int k = 0; k < 3; k++)
										{
												ModifyGameField(norm_comb.buff_comb[i][k],Z);									
										}
											return true;
								}
								else
								{
									norm_comb.cnt_comb = 0;
								}
							}
						}
					}
	return result;
}

uint8_t SearchLineBlock(uint8_t num_cell)
{
	uint8_t result_search = 0;
	int cnt_x = 0;
				/*произведем поиск индекса в массиве блокируемых комбинаций для дальнейшего блокирования и удаления*/
					for(int i = 0 ; i < 8; i++)
					{
						for(int j = 0; j < 3;j++)
						{
							if(block_comb.buff_comb[i][j] == num_cell)
							{
								/*проверим поле на наличие крестика*/
								for(int k = 0; k < 3; k++)
								{
									if(CheckFild(block_comb.buff_comb[i][k],x_coordinates) != SET && 
										 CheckFild(block_comb.buff_comb[i][k],z_coordinates) != SET)
									{
										/*определим индекс линии которую необходимо блокировать*/
										result_search = block_comb.buff_comb[i][k];										
									}
									if(CheckFild(block_comb.buff_comb[i][k],x_coordinates) == SET)
									{
										cnt_x++;
									}									
								}
								if(cnt_x == 2 && result_search != 0)
								{
									return result_search;
								}
								if(cnt_x == 3)
								{
									return LOSS;
								}
								result_search = 0;
								cnt_x = 0;								
							}
						}
					 }
  			return result_search;		
}
void MoveComb(uint8_t num_cell,char* source,char* destination)
{

					/*произведем поиск комбинации которую необходимо перенести в список блокирования*/
					for(int i = 0 ; i < 8; i++)
					{
						for(int j = 0; j < 3;j++)
						{
							if(source[j] == num_cell)
							{
								/*проверим линию на наличие нолика*/
								for(int k = 0; k < 3; k++)
								{ 
										if(CheckFild(source[k],z_coordinates) == SET)
										{
											/*удалим линию*/
											for(int clear_idx = 0 ; clear_idx < 3 ;clear_idx++)
											{
												source[clear_idx] = 0;
											}
											break;
										}									
								}	
								/*произведем поиск пустого места в массиве блокирования*/
								for(int idx = 0; idx < 8; idx++)
								{
									if(destination[idx] == 0)
									{										
										break;
									}
									destination += 3;
								}
								/*переместим в структуру блокирования*/
								for(int k = 0 ; k < 3;k++)
								{
									destination[k] = source[k];
									source[k] = 0;
								}	
							}
						}
						source += 3;
					}	
}

void RemoveComb(uint8_t num_cell,char* source)
{

		/*произведем поиск комбинации которую необходимо удалить из блокировки*/
		for(int i = 0 ; i < 8; i++)
		{
			for(int j = 0; j < 3;j++)
			{
				if(source[j] == num_cell)
				{
					/*переместим в структуру блокирования*/
					for(int k = 0 ; k < 3;k++)
					{
						source[k] *= 0;
					}						
				}
			}
			source +=3;
		}
}

void SetMoveInMsg(uint8_t move, uint8_t symbol)
{
	
	uint8_t x1 = 0;
	uint8_t x2 = 0;
	uint8_t y1 = 0;
	uint8_t y2 = 0;
	char *ptr_array;
	char *offset_ptr_array;
	uint8_t x_y_symbol[10][2] =     {{0,0},{0,1},{0,2},
														      {1,0},{1,1},{1,2},
														      {2,0},{2,1},{2,2},
																	{3,0}};
	
  static char *array_symbol[][3] = {{"A1","B1","C1"},
																		{"A2","B2","C2"},
																		{"A3","B3","C3"},
																		{"  "}};
	
	offset_ptr_array = ptr_array = array_symbol[x_y_symbol[move][0]][x_y_symbol[move][1]];
	++offset_ptr_array;
																		
	if(symbol == Z)
	{
		x1 = 22;
		x2 = 23;
	}
	
	else if(symbol == X)
	{
		x1 = 10;
		x2 = 11;
	}
	
	your_step_my_step[x1] = *ptr_array;//*array_symbol[x_y_symbol[move][0]][x_y_symbol[move][1]];
	your_step_my_step[x2] = *offset_ptr_array;//*(++array_symbol[x_y_symbol[move][0]][x_y_symbol[move][1]]);
	
}

void ResetAllDataGame(void)
{
		
		for(int i = 1; i < 10; i++)
		{
			ModifyGameField(i,CLR);						
		}
		for(int i = 0; i < 8;i++)
		{
			for(int j = 0; j < 3; j++)
		  {
				block_comb.buff_comb[i][j] = 0;
				norm_comb.buff_comb[i][j] = 0;
			}								
		}
		for(int i = 0; i < 10; i++)
		{
				z_coordinates[i] = 0;
				x_coordinates[i] = 0;		
		}
		step = 0;	
		SetMoveInMsg(CLEAR_STRING,Z);
		SetMoveInMsg(CLEAR_STRING,X);
		flag_start_game = true;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

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
