/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "ws2812b.h"
#include "pca9536.h"
#include <stdio.h>

#include "usbd_cdc_if.h"
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

/*
 * Global variables
 */
char buff[17];            // buffer for LCD display

uint32_t led_colors[12] = { 0 };      // this is buffer for all 12 rgb leds
uint8_t rotary = 1;                  // inc or dec when rotary encoder is rotated

uint32_t Trimpot[2];
enum
{
  Task1 = 1, Task2, Task3
} menu = 1;

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;

uint16_t frequency = 1000;
uint8_t duty1 = 0, duty2 = 0;

uint8_t usbReceiveBuf[64];
uint8_t usbSendBuf[20];

uint8_t timeoutRcv = 0;
boolean noMessage = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void changePWMFrequency (uint16_t frequencyP)
{
  uint16_t prescalerValue = (float) (320000 / frequencyP);
  __HAL_TIM_SET_PRESCALER(&htim22, prescalerValue);
}

void clearReceivedMessage ()
{
  memset (usbReceiveBuf, ' ', 10);
  noMessage = true;
}

void writeWeekDayToDisplay (uint8_t weekDayP)
{
  switch (weekDayP)
  {
	case RTC_WEEKDAY_MONDAY:
	  lcd_putstr ("MON");
	  break;
	case RTC_WEEKDAY_TUESDAY:
	  lcd_putstr ("TUE");
	  break;
	case RTC_WEEKDAY_WEDNESDAY:
	  lcd_putstr ("WED");
	  break;
	case RTC_WEEKDAY_THURSDAY:
	  lcd_putstr ("THU");
	  break;
	case RTC_WEEKDAY_FRIDAY:
	  lcd_putstr ("FRI");
	  break;
	case RTC_WEEKDAY_SATURDAY:
	  lcd_putstr ("SAT");
	  break;
	case RTC_WEEKDAY_SUNDAY:
	  lcd_putstr ("SUN");
	  break;
	default:
	  break;
  }
}

void getRtcDateTime ()
{
  HAL_RTC_GetDate (&hrtc, &currentDate, RTC_FORMAT_BIN);
  HAL_RTC_GetTime (&hrtc, &currentTime, RTC_FORMAT_BIN);
}

void builtInRtc ()
{
  lcd_goto_rc (0, 0);
  sprintf (buff, "%02d/%02d/%02d     ", currentDate.Date, currentDate.Month, currentDate.Year);
  lcd_putstr (buff);
  writeWeekDayToDisplay (currentDate.WeekDay);

  lcd_goto_rc (1, 0);
  sprintf (buff, "    %02d:%02d:%02d    ", currentTime.Hours, currentTime.Minutes, currentTime.Seconds);
  lcd_putstr (buff);
}

void serialCom ()
{
  lcd_goto_rc (0, 0);

  uint8_t len = strlen (usbReceiveBuf);
  if (timeoutRcv == 0) clearReceivedMessage ();
  else
  {
	//Fill with empty spaces to wipe out LCD previous messages
	for (uint8_t i = 0; i < (9 - len); i++)
	  usbReceiveBuf[len + i] = 32;
  }
  sprintf (buff, "Rec: %s ", usbReceiveBuf);
  lcd_putstr (buff);

  lcd_goto_rc (1, 0);
  sprintf (buff, "Timeout: %02ds    ", timeoutRcv);
  lcd_putstr (buff);
}

void dutyFromTrimpot ()
{
  duty1 = (float) (4095 - Trimpot[0]) * 0.0245;
  duty2 = (float) (4095 - Trimpot[1]) * 0.0245;

  __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_1, duty1);
  __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, duty2);
}

void pwm ()
{
  dutyFromTrimpot ();

  lcd_goto_rc (0, 0);
  sprintf (buff, "CH1   %04d   CH2", frequency);
  lcd_putstr (buff);

  lcd_goto_rc (1, 0);
  sprintf (buff, "%03d%%   Hz   %03d%%", duty1, duty2);
  lcd_putstr (buff);
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM22_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start (&hadc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA (&hadc, Trimpot, 2);
  HAL_TIM_PWM_Start (&htim22, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim22, TIM_CHANNEL_2);

  lcd_init ();
  lcd_write_cmd (LCD_CLEAR_DISPLAY);
  lcd_write_cmd (LCD_CURSOR_OFF);

  pca9536_configure_port (&hi2c1, 0xC0);
  pca9536_write_port_outputs (&hi2c1, 0x02);

  HAL_Delay (100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	switch (menu)
	{
	  case Task1:
		builtInRtc ();
		break;
	  case Task2:
		serialCom ();
		break;
	  case Task3:
		pwm ();
		break;
	  default:
		break;
	}

	getRtcDateTime ();
	HAL_Delay (10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  if (timeoutRcv > 0) timeoutRcv--;
  else HAL_TIM_Base_Stop_IT (&htim6);
}

void HAL_GPIO_EXTI_Callback (uint16_t pin)
{
  if (pin == RE_A_Pin)
  {
	//Clockwise
	if (HAL_GPIO_ReadPin (RE_B_GPIO_Port, RE_B_Pin) == 1) menu++;
	else menu--;

	if (menu > 3) menu = 1;
	if (menu < 1) menu = 3;

	if (menu != Task3)
	{
	  duty1 = 0;
	  duty2 = 0;
	  frequency = 1000;
	  __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SET_COMPARE(&htim22, TIM_CHANNEL_2, 0);
	}

	if (menu != Task2) timeoutRcv = 0;
  }

  //ESC Button
  if (pin == GPIO_PIN_1)
  {
	if (menu == Task2)
	{
	  clearReceivedMessage ();
	  timeoutRcv = 0;
	}

	if (menu == Task3)
	{
	  frequency -= 100;
	  if (frequency <= 100) frequency = 100;
	  changePWMFrequency (frequency);
	}

  }

  //ENTER Button
  if (pin == GPIO_PIN_0)
  {
	if (menu == Task2 && noMessage == false)
	{
	  timeoutRcv++;
	  if (timeoutRcv > 10) timeoutRcv = 10;
	}

	if (menu == Task3)
	{
	  frequency += 100;
	  if (frequency >= 1000) frequency = 1000;
	  changePWMFrequency (frequency);
	}
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
  __disable_irq ();
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
