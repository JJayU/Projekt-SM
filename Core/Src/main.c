/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMPXX80.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "lcd_i2c.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef float float32_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float temperature;
int32_t pressure;
char text[] = "12345";
int sterowanie = 0;
float pid_output;

float Kp = 200;
float Ki = 5;
float Kd = 1;
float dt = 0.1;
float previous_error = 0;
float previous_integral = 0;

float target_temperature;

int pid_output_int;

struct lcd_disp disp;

int impulsy;
int target_enkoder;

char rx_buffer[2];
int target_uart = 25;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float calculate_discrete_pid(float setpoint, float measured)
{
	float u=0, P, I, D, error, integral, derivative;

	// Uchyb
	error = setpoint-measured;

	// Czlon proporcjonalny
	P = Kp * error;

	// Czlon calkujacy
	integral = previous_integral + (error+previous_error) ;
	if(integral < 0)
		integral = 0;
	if(integral > 2000)
		integral = 2000;
	previous_integral = integral;
	I = Ki*integral*(dt/2.0);

	// Czlon rozniczkujacy
	derivative = (error - previous_error)/dt;
	previous_error = error;
	D = Kd*derivative;

	// Suma czlonow
	u = P * (1.0 + I + D);

	// Ograniczenie sygnalu sterujacego
	if(u>2000)
		return 2000;
	else if(u<0)
		return 0;
	return u;
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Inicjalizacja czujnika BMP280
  BMP280_Init(&hi2c2, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);

  // Inicjalizacja timerow oraz wyjscia PWM
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  // Obsluga LCD
  disp.addr = (0x27 << 1);
  disp.bl = true;
  lcd_init(&disp);

  // Enkoder - inicjalizacja
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim1, 1000);

  // Zmienna zatwierdzajaca od przycisku
  int x = 1;

  // Inicjalizacja przerwania do odczytu danych z UART
  HAL_UART_Receive_IT(&huart3, rx_buffer, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Odczyt temperatury z czujnika
	BMP280_ReadTemperatureAndPressure(&temperature, &pressure);

	// Wysylanie aktualnej temperatury przez UART
	sprintf((char*)text, "%.2f, ", temperature);
	HAL_UART_Transmit(&huart3, (uint8_t*)text, strlen(text), 1000);

	// Obsluga enkodera
	impulsy = __HAL_TIM_GET_COUNTER(&htim1);
	target_enkoder = (impulsy/2) - 500 + 25;

	// Wyswietlanie aktualnej temperatury
	sprintf((char*)disp.s_line, "Aktualna = %.2f", temperature);

	// Obsluga przycisku i zmiana trybu dzialania
	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
		x = x+1;

	switch (x){
		case 1:
			target_temperature = target_uart;
			sprintf((char*)disp.f_line, "Zadana (PC) = %u", target_uart);
			break;
		case 2:
			target_temperature = target_enkoder;
			sprintf((char*)disp.f_line, "Zadana (EN) = %u", target_enkoder);
			break;
		case 3:
			x = 1;
			break;
	}

	// Wyswietlanie na LCD
	lcd_display(&disp);
	HAL_Delay(200);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Przerwanie co 100ms odpowiedzialne za policzenie sygnalu sterujacego
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		// Migajaca dioda sygnalizujaca dzialanie
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		// Wywolanie funkcji PID
		pid_output = calculate_discrete_pid(target_temperature, temperature);
		pid_output_int = pid_output;

		// Ustawienie obliczonej wartosci na wyjsciu PWM
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, pid_output_int);
	}
}

// Przerwanie od odbioru danych z UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Restart przerwania
	HAL_UART_Receive_IT(&huart3, rx_buffer, 2);

	// Konwersja na Int
	target_uart = atoi(rx_buffer);
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
