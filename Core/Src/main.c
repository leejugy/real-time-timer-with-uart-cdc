/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	CDC = 0,
	UART1,
}send_type;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PC13_ON   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
#define PC13_OFF  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
#define PC13_TOGGLE  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
#define PC13_SET(X)  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,X);


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t cdc_rx_buffer[5];
bool cdc_rx_flag = false;

uint8_t uart_rx_buffer;
bool uart_rx_flag;

uint8_t tx_buffer[32];

uint8_t real_time_timer[3]={0,0,0};
bool is_afternoon=false;
bool input_current_time_flag=false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void send(send_type Send_Type,char *fmt,...);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint8_t rt_divider;
	if(htim == &htim3 && input_current_time_flag){
		if(is_afternoon){
			send(UART1, "Afternoon ");
			rt_divider=12;
		}
		else{
			send(UART1, "Noon ");
			rt_divider=13;
		}
		real_time_timer[0]++;
		if(real_time_timer[0]>=60){
			real_time_timer[0] = 0;
			real_time_timer[1]++;
		}
		if(real_time_timer[1]>=60){
			real_time_timer[1] = 0;
			real_time_timer[2]++;
		}
		if(real_time_timer[2]>=rt_divider){
			real_time_timer[2]%=rt_divider;
			is_afternoon=!is_afternoon;
		}
		send(UART1, "%02d:%02d:%02d\n",
				real_time_timer[2],real_time_timer[1],real_time_timer[0]);
	}
  UNUSED(htim);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
		uart_rx_flag = true;
		HAL_UART_Receive_DMA(&huart1,&uart_rx_buffer,1);
	}
  UNUSED(huart);
}


void send(send_type Send_Type,char *fmt,...){
	va_list arg;
	va_start(arg,fmt);
	vsnprintf((char *)tx_buffer,32,fmt,arg);

	if(Send_Type==CDC){
		CDC_Transmit_FS(&tx_buffer[0], 32);
	}
	else if(Send_Type==UART1){
		HAL_UART_Transmit(&huart1, &tx_buffer[0], 32, 10);
	}

	va_end(arg);
	memset(tx_buffer,0,32);
}

uint16_t unsigned_atoi(const uint8_t *number,size_t len){
  uint16_t val = 0;
  for(size_t i=0;i<len;i++){
    if(number[i]<='9' && number[i]>='0'){
      val = number[i]-'0'+val*10;
    }
    else{
      return 'f';
    }
  }
  return val;
}

void init_timer(){
  uint8_t count = 0;
  send(UART1,"\x1b[2J\n");
  send(CDC,"\x1b[2J\ninput currnet time : ");
  input_current_time_flag = false;
	if(!input_current_time_flag){
		cdc_rx_flag = false;
		while (1){
			if(cdc_rx_flag){
				switch (count){
				case 0:
					is_afternoon = unsigned_atoi(&cdc_rx_buffer[0],1);
					count++;
					break;
				case 1:
					real_time_timer[2]= unsigned_atoi(&cdc_rx_buffer[0],1)*10;
					count++;
					break;
				case 2:
					real_time_timer[2]+= unsigned_atoi(&cdc_rx_buffer[0],1);
					count++;
					break;
				case 3:
					real_time_timer[1]= unsigned_atoi(&cdc_rx_buffer[0],1)*10;
					count++;
					break;
				case 4:
					real_time_timer[1]+= unsigned_atoi(&cdc_rx_buffer[0],1);
					count++;
					break;
				}
				send(CDC,"%c",cdc_rx_buffer[0]);
				cdc_rx_buffer[0] = 0;
				cdc_rx_flag = 0;
				if(count == 5){
					send(CDC,"\n");
					input_current_time_flag = true;
					real_time_timer[0] = 0;
					htim3.Instance->CNT = 0;
					break;
				}
			}
		}
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA(&huart1, &uart_rx_buffer, 1);
  uint32_t pretime=HAL_GetTick();
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if(HAL_GetTick()-pretime>300){
  		PC13_TOGGLE
			pretime = HAL_GetTick();
  	}
  	if(cdc_rx_flag){
  		if(cdc_rx_buffer[0]=='i'){
  			init_timer();
  		}
  	  uart_rx_flag = false;
  	}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
