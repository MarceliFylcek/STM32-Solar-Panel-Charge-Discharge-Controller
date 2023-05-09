/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t vRead;
uint16_t aRead;
float cell_V = 0.0f;
float cell_A = 0.0f;
float cell_V_avg = 0.0f;
float cell_A_avg = 0.0f;
float cell_A_offset = 2.4420f;
float V_table [10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float A_table [10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float duty_cycle = 50.0f;
float discharge_i = -25.0f;
float charge_i = 100.0f;
float i_ref = 0.0f;
float error = 0.0f;
float PID_P = 0.0f;
float PID_I = 0.0f;
float Kp = 0.00002f;
float Ki = 0.0000000000007f;
uint16_t duty_cycle_int = 0;
uint8_t enable = 0;
uint8_t bc = 0;
uint8_t diode_state = 0;
uint8_t mode_changes = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float get_avg(float new, float* table)
{
	table[0] = table[1];
	table[1] = table[2];
	table[2] = table[3];
	table[3] = table[4];
	table[4] = table[5];
	table[5] = table[6];
	table[6] = table[7];
	table[7] = table[8];
	table[8] = table[9];
	table[9] = new;

	float sum = 0.0f;
	for(int i = 0; i<10; i++)
	{
		sum += table[i];
	}

	float avg = sum / 10.0f;
	return avg;
}

void balanceVoltage()
{
	error = i_ref - cell_A_avg;

	PID_P = Kp * error;
    PID_I += Ki*error;
    if(PID_I > 3000.0f)
    {
    	PID_I = 3000.0f;
    }

    if (PID_I < -3000.0f)
    {
    	PID_I = -3000.0f;
    }

	duty_cycle += ((PID_P+PID_I)/1000.0f);

	if (duty_cycle > 0.90f)       //OGRANICZENIE 0,15 - 0,85 (630-3570)
	{
		duty_cycle = 0.90f;
	}

	else if (duty_cycle < 0.10f)
	{
		duty_cycle = 0.10f;
	}


	duty_cycle_int = (int)(duty_cycle *4200.0f);  //KONWERSJA 0,15 - 0,85
}

void control() //Glowna funkcja
{
	 cell_V = (float)vRead/1024.0f*3.3f*2.0f;	//KONWERSJA
	 cell_V_avg = get_avg(cell_V, V_table);		//UŚREDNIENIE


	 cell_A = ((float)aRead/4095.0f*3.3f - cell_A_offset)*1000000.0f/185.0f;  //KONWERSJA
	 cell_A = cell_A * -1.0f;
	 cell_A_avg = get_avg(cell_A, A_table);						   //UŚREDNIENIE


	balanceVoltage(); //REGULACJA PRADU


	//duty_cycle_int
	//duty_cycle_int
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle_int); //ZMIANA WYPELNIENIA

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//  HAL_GPIO_WritePin(enPin_GPIO_Port, enPin_Pin, GPIO_PIN_SET);
//  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle_int); //DUTY CYCLE (0-20000)
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_GPIO_WritePin(enPin_GPIO_Port, enPin_Pin, GPIO_PIN_SET);
  enable = 1;
  i_ref = discharge_i;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (HAL_ADC_PollForConversion(&hadc1, 10))  //POMIAR NAPIECIA BATERII
	 {
		 vRead = HAL_ADC_GetValue(&hadc1);
		 HAL_ADC_Start(&hadc1);
	 }

	  if (HAL_ADC_PollForConversion(&hadc3, 10)) //POMIAR NAPIECIA CZUJNIKA PRADU
	 {
		 aRead = HAL_ADC_GetValue(&hadc3);
		 HAL_ADC_Start(&hadc3);
	 }

	  if(HAL_GPIO_ReadPin(diode_GPIO_Port, diode_Pin))
	  {
		  if (diode_state == 0)
		  {
			  diode_state = 1;
			  i_ref = charge_i;
			  mode_changes++;

		  }

	  }
	  else
	  {
		  if (diode_state == 1)
		  {
			  diode_state = 0;
			  i_ref = discharge_i;
			  mode_changes++;

		  }
	  }

	  if (duty_cycle > 0.899f && cell_A_avg < -160.0f)
	  {
	  		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	  		enable = 0;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim2)
	{
		control();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == btn_Pin)
	{
		bc = bc +1;

	}
}


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
