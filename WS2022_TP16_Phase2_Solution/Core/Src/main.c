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
#include "aht20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALIGN_ACCESS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char buff[17];
uint32_t led_colors[12] = { 0 };
uint8_t rotary = 1;

uint32_t Trimpot[2];

enum
{
  celsius = 1, fahrenheit
} tempUnit = 1;

enum
{
  sensor = 0, trimpot
} sources = 0;

enum
{
  reportCurrent = 1, reportAverage, reportLowest, reportHighest, clearEeprom, reportUnavailable, reportError, reportMenu
} reportOptions = 1;

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;

uint8_t usbReceivedString[10];
uint8_t usbReceiveBuf[10];
uint8_t usbSendBuf[400];

uint16_t tempAndHum[2];
uint8_t tenAverageMeasures = 0;

uint8_t oneTimePressed = 0;
uint16_t averageTempAndHum[2];

// 0      1      2     3      4       5
//LHum, LTemp, HHum, HTemp, AvgHum, AvgTemp
uint32_t persistentTempAndHum[6];
uint32_t resetBuf[4] = { 333, 0, 0, 0 };
uint32_t BaseEEPROMAddr = 0x08080000U;
uint8_t persistentAddr = 0x00;
uint8_t isDifferent = 0;
uint8_t isResetRequested = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setCurrentTimeForTest ()
{
  currentTime.Hours = 12;
  currentTime.Minutes = 05;
  currentTime.Seconds = 20;
}

void writeToEEPROM (uint8_t addrP, uint32_t valueP)
{
//  Cortex-M0 does not support unaligned read/write operations

//  0x08080002 halfword aligned
//  0x08080004 word aligned

  HAL_FLASHEx_DATAEEPROM_Unlock ();
  HAL_FLASHEx_DATAEEPROM_Erase (DATA_EEPROM_BASE | (addrP * ALIGN_ACCESS));
  HAL_FLASHEx_DATAEEPROM_Program (FLASH_TYPEPROGRAMDATA_WORD, (DATA_EEPROM_BASE | (addrP * ALIGN_ACCESS)), valueP);
  HAL_FLASHEx_DATAEEPROM_Lock ();
  HAL_Delay (10);
}

uint32_t readFromEEPROM (uint8_t addrP)
{
  return (*(__IO uint32_t*) (DATA_EEPROM_BASE | (addrP * ALIGN_ACCESS)));
}

void readEepromForPersistentValues (uint32_t *persistentValuesP)
{
  for (uint8_t i = 0; i < 4; i++)
	persistentValuesP[i] = readFromEEPROM (i);
}

void writePersistentValuesToEeprom (uint32_t *persistentValuesP)
{
  for (uint8_t i = 0; i < 4; i++)
	writeToEEPROM (i, persistentValuesP[i]);
}

void clearLeds ()
{
  led_colors[0] = COLOR_BLACK;
  led_colors[1] = COLOR_BLACK;
  led_colors[2] = COLOR_BLACK;
  led_colors[3] = COLOR_BLACK;
  led_colors[4] = COLOR_BLACK;
  led_colors[5] = COLOR_BLACK;
  led_colors[6] = COLOR_BLACK;
  led_colors[7] = COLOR_BLACK;
  led_colors[8] = COLOR_BLACK;
  led_colors[9] = COLOR_BLACK;
  led_colors[10] = COLOR_BLACK;
  led_colors[11] = COLOR_BLACK;
  ws2812b_display_all_led_colors (led_colors);
}

void showTimeOnLeds ()
{

  uint8_t minuteLed = currentTime.Minutes / 5;
  //Minutes
  for (uint8_t i = 0; i < 12; i++)
  {
	if (i == currentTime.Minutes / 5) led_colors[i] = COLOR_BLUE;
	else led_colors[i] = COLOR_BLACK;
  }

  //Hours
  for (uint8_t i = 0; i < 12; i++)
  {
	if (currentTime.Hours >= 12)
	{
	  //PM
	  if (i <= (currentTime.Hours - 12))
	  {
		if (minuteLed != i) led_colors[i] = COLOR_ORANGE;
		else led_colors[i] = COLOR_VIOLET;
	  }
	}
	else
	{
	  //AM
	  if (i <= currentTime.Hours)
	  {
		if (minuteLed != i) led_colors[i] = COLOR_RED;
		else led_colors[i] = COLOR_VIOLET;
	  }
	}
  }

  //Seconds
  for (uint8_t i = 0; i < 12; i++)
  {
	if (i == currentTime.Seconds / 5) led_colors[i] = COLOR_GREEN;
  }

  ws2812b_display_all_led_colors (led_colors);
}

void HAL_GPIO_EXTI_Callback (uint16_t pin)
{
  if (pin == RE_A_Pin)
  {
	//Clockwise
	if (HAL_GPIO_ReadPin (RE_B_GPIO_Port, RE_B_Pin) == 1) tempUnit++;
	else tempUnit--;

	if (tempUnit > 2) tempUnit = 1;
	if (tempUnit < 1) tempUnit = 2;
  }

  //ENTER Button
  if (pin == GPIO_PIN_0)
  {
	sources++;
	if (sources > 1) sources = 0;
  }

  //ESC Button
  if (pin == GPIO_PIN_1)
  {
	if (!oneTimePressed) tenAverageMeasures = 10;
	oneTimePressed = 1;
  }
}

void getRtcTime ()
{
  HAL_RTC_GetTime (&hrtc, &currentTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate (&hrtc, &currentDate, RTC_FORMAT_BIN);
}

void convertTrimpotValues ()
{
  tempAndHum[0] = (float) (4095 - Trimpot[1]) * 0.0244;
  tempAndHum[1] = (float) (4095 - Trimpot[0]) * 0.004 + 6;
}

uint16_t convertCelsiusToFahrenheit (uint16_t temperatureP)
{
  return (float) (temperatureP * 1.8 + 32);
}

void getLowestAndHighestValues ()
{
  //Finds the lowest Temp and Hum
  if (tempAndHum[1] < persistentTempAndHum[1] || persistentTempAndHum[1] == 0)
  {
	persistentTempAndHum[1] = tempAndHum[1]; //Temp
	isDifferent = 1;
  }
  if (tempAndHum[0] < persistentTempAndHum[0] || persistentTempAndHum[0] == 333)
  {
	persistentTempAndHum[0] = tempAndHum[0]; //Hum
	isDifferent = 1;
  }

  //Finds the highest Temp and Hum
  if (tempAndHum[1] > persistentTempAndHum[3] || persistentTempAndHum[3] == 0)
  {
	persistentTempAndHum[3] = tempAndHum[1]; //Temp
	isDifferent = 1;
  }
  if (tempAndHum[0] > persistentTempAndHum[2] || persistentTempAndHum[2] == 0)
  {
	persistentTempAndHum[2] = tempAndHum[0]; //Hum
	isDifferent = 1;
  }

  if (isDifferent)
  {
	isDifferent = 0;
	writePersistentValuesToEeprom (persistentTempAndHum);
  }
}

void readTempAndHumidity ()
{
  if (sources) convertTrimpotValues ();
  else getTemperatureAndHumidity (tempAndHum);

  getLowestAndHighestValues ();
}

void weatherSupervisory ()
{
  if (!HAL_GPIO_ReadPin (TEST_GPIO_Port, TEST_Pin)) setCurrentTimeForTest ();
  showTimeOnLeds ();

  lcd_goto_rc (0, 0);
  sprintf (buff, "%02d", tempUnit == 1 ? tempAndHum[1] : convertCelsiusToFahrenheit (tempAndHum[1]));
  lcd_putstr (buff);
  lcd_putchar (0xF2); //Degree symbol
  lcd_putchar (tempUnit == 1 ? 'C' : 'F'); //Unit letter
  lcd_putstr ("    ");
  lcd_putchar (tenAverageMeasures ? '*' : ' '); //Average happening
  sprintf (buff, "    %02d%%", tempAndHum[0]);
  lcd_putstr (buff);

  lcd_goto_rc (1, 0);
  sprintf (buff, "    %02d:%02d:%02d    ", currentTime.Hours, currentTime.Minutes, currentTime.Seconds);
  lcd_putstr (buff);
}

void sendReport ()
{
  if (reportOptions == reportAverage && (averageTempAndHum[0] == 0 || averageTempAndHum[1] == 0))
  {
	reportOptions = reportError;
  }

  uint16_t formattedValues[8];
  if (tempUnit == fahrenheit)
  {
	formattedValues[1] = convertCelsiusToFahrenheit (tempAndHum[1]); //CurentTem
	formattedValues[3] = convertCelsiusToFahrenheit (averageTempAndHum[1]); //AvgTem
	formattedValues[5] = convertCelsiusToFahrenheit (persistentTempAndHum[1]);	//LowestTem
	formattedValues[7] = convertCelsiusToFahrenheit (persistentTempAndHum[3]); //HighestTem
  }
  else
  {
	formattedValues[1] = tempAndHum[1]; //CurentTem
	formattedValues[3] = averageTempAndHum[1]; //AvgTem
	formattedValues[5] = persistentTempAndHum[1];	//LowestTem
	formattedValues[7] = persistentTempAndHum[3]; //HighestTem
  }
  formattedValues[0] = tempAndHum[0]; //CurrentHum
  formattedValues[2] = averageTempAndHum[0]; //AvgHum
  formattedValues[4] = persistentTempAndHum[0]; //LowestHum
  formattedValues[6] = persistentTempAndHum[2]; //HighestHum

  switch (reportOptions)
  {
	case reportCurrent:
	  sprintf (usbSendBuf, "\n----Current Temperature and Humidity----\n\n"
			   "Temperature: %02d%c%c\n"
			   "Humidity: %02d%%\n"
			   "----------------------------------------\n\n",
			   formattedValues[1], 0xB0, tempUnit == 1 ? 'C' : 'F', formattedValues[0]);
	  break;

	case reportAverage:
	  sprintf (usbSendBuf, "\n----Average Temperature and Humidity for 10 measurements----\n\n"
			   "Average Temperature: %02d%c%c\n"
			   "Average Humidity: %02d%%\n"
			   "------------------------------------------------------------\n\n",
			   formattedValues[3], 0xB0, tempUnit == 1 ? 'C' : 'F', formattedValues[2]);
	  break;

	case reportLowest:
	  sprintf (usbSendBuf, "\n----Lowest Temperature and Humidity registered----\n\n"
			   "Lowest Temperature: %02d%c%c\n"
			   "Lowest Humidity: %02d%%\n"
			   "--------------------------------------------------\n\n",
			   formattedValues[5], 0xB0, tempUnit == 1 ? 'C' : 'F', formattedValues[4]);
	  break;

	case reportHighest:
	  sprintf (usbSendBuf, "\n----Highest Temperature and Humidity registered----\n\n"
			   "Highest Temperature: %02d%c%c\n"
			   "Highest Humidity: %02d%%\n"
			   "---------------------------------------------------\n\n",
			   formattedValues[7], 0xB0, tempUnit == 1 ? 'C' : 'F', formattedValues[6]);
	  break;

	case clearEeprom:
	  writePersistentValuesToEeprom (resetBuf);
	  readEepromForPersistentValues (persistentTempAndHum);
	  sprintf (usbSendBuf, "\n----EEPROM has been erased----\n\n");
	  break;

	case reportUnavailable:
	  sprintf (usbSendBuf, "\n----Sorry, no report corresponds to the request!----\n\n");
	  break;

	case reportError:
	  sprintf (usbSendBuf, "\n----Average Temperature and Humidity for 10 measurements----\n\n"
			   "No average temperature/humidity available,\n"
			   "please execute averaging measurement\n"
			   "------------------------------------------------------------\n\n");
	  break;

	case reportMenu:
	  sprintf (usbSendBuf, "\nWeather Station - WorldSkills Special Edition 2022\n\n"
			   "1.Current Temperature and Humidity\n"
			   "2.Average Temperature and Humidity for 10 measurements\n"
			   "3.Lowest Temperature and Humidity registered\n"
			   "4.Highest Temperature and Humidity registered\n"
			   "5.Erase EEPROM values\n");
	  break;
  }

  CDC_Transmit_FS (usbSendBuf, strlen (usbSendBuf));
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
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start (&hadc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA (&hadc, Trimpot, 2);

  HAL_TIM_Base_Start_IT (&htim6);

  lcd_init ();
  lcd_write_cmd (LCD_CLEAR_DISPLAY);
  lcd_write_cmd (LCD_CURSOR_OFF);

  pca9536_configure_port (&hi2c1, 0xC0);
  pca9536_write_port_outputs (&hi2c1, 0x02);

  initAHT20 ();
  getRtcTime ();

  readEepromForPersistentValues (persistentTempAndHum);

  HAL_Delay (2000);

  reportOptions = reportMenu;
  sendReport ();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (isResetRequested)
	{
	  isResetRequested = 0;
	  sendReport ();
	}

	readTempAndHumidity ();
	getRtcTime ();
	weatherSupervisory ();
	HAL_Delay (100);
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
  if (tenAverageMeasures)
  {
	averageTempAndHum[0] += tempAndHum[0];
	averageTempAndHum[1] += tempAndHum[1];
	tenAverageMeasures--;

	if (tenAverageMeasures == 0)
	{
	  averageTempAndHum[0] /= 10;
	  averageTempAndHum[1] /= 10;
	  oneTimePressed = 0;
	}
  }

//  currentTime.Seconds++;
//  if (currentTime.Seconds > 59)
//  {
//	currentTime.Seconds = 0;
//	currentTime.Minutes++;
//  }
//  if (currentTime.Minutes > 59)
//  {
//	currentTime.Minutes = 0;
//	currentTime.Hours++;
//  }
//  if (currentTime.Hours > 23)
//  {
//	currentTime.Hours = 0;
//  }
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
