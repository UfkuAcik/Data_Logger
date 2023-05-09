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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280_STM32.h"
#include "stdio.h"   //Sprintf icin gerekli olacak(SD CARD)
#include "fatfs_sd.h"
#include "string.h"

#include "lwgps/lwgps.h"

#include "max30102_for_stm32_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float spo2=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//// printf() function
//int __io_putchar(int ch)
//{
//  uint8_t temp = ch;
//  HAL_UART_Transmit(&huart1, &temp, 1, HAL_MAX_DELAY);
//  return ch;
//}

// Override plot function
void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{
    // printf("ir:%u\n", ir_sample);                  // Print IR only
    // printf("r:%u\n", red_sample);                  // Print Red only
   // printf("ir:%u,r:%u\n", ir_sample, red_sample);    // Print IR and Red
    spo2 = 100 *  ((float)red_sample / (float)(ir_sample));
}

// MAX30102 object
max30102_t max30102;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////////////////////////// GPS //////////////////////////////////////////////////////////
lwgps_t gps;

uint8_t rx_buffer[128];
uint8_t rx_index = 0;
uint8_t rx_data = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //hızı knot cinsinden veriyor dikkat et
{
	if(huart == &huart2) {
		if(rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_data;
		} else {
			lwgps_process(&gps, rx_buffer, rx_index+1);
			rx_index = 0;
			rx_data = 0;
		}
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}
///////////////////////////  Nabiz   ///////////////////////////////
int milisaniye=0;
int nabiz_bpm=0;

int counterOutside=0;
int counterInside=0;
int previousMillis=0;
int currentMillis=0;
/////////////////////// SD CARD ////////////////////////////////
int indx=0;
int indx_counter=0;
int part=1;

char yazi[200];
char kayit_dosya_ismi[50];
////////////////////   BME280     //////////////////////////////
float Temperature, Pressure, Humidity;
//////////////////////   DS18B20          ////////////////////
float Temperature_DS;
uint8_t Temp_byte1, Temp_byte2;
uint16_t TEMP;
/////////////////////// DS18B20 MicroDelay     ///////////////////////////////
void microDelay (uint16_t delay)  //Mikrosaniye Delay
{
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  while (__HAL_TIM_GET_COUNTER(&htim5) < delay);  //ds18b20 icin timer source
}
//////////////////////// ONE WIRE PIN OUTPUT-INPUT AYARLAMA   DS18B20 ///////////////////////////////////////////
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
///////////////////////// DS18B20 Fonksiyonlar  ////////////////////////////////////////

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	microDelay(480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	microDelay(80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	microDelay(480); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			microDelay(1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			microDelay(50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			microDelay(50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the data pin LOW
		microDelay(1);  // wait for > 1us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		microDelay(50);  // wait for 60 us
	}
	return value;
}
///////////////////////////  FATFS   /////////////////////////////////////////////

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];  // to store strings..

int i=0;

int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
}

void send_uart (char *string)
{
	uint8_t len = strlen (string);
	HAL_UART_Transmit(&huart1, (uint8_t *) string, len, HAL_MAX_DELAY);  // transmit in blocking mode
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////



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
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  ///////////////////////// DS18B20 /////////////////////////////////
  HAL_TIM_Base_Start(&htim5);
  ///////////////////////////// BME280 ////////////////////////////////
  BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
  ///////////////////////////////////////////////////////////////////
  HAL_Delay (500);

//    fresult = f_mount(&fs, "/", 1);
//    	if (fresult != FR_OK) send_uart ("ERROR!!! in mounting SD CARD...\n\n");
//    	else send_uart("SD CARD mounted successfully...\n\n");
//
//
//    	/*************** Card capacity details ********************/
//
//    	/* Check free space */
//    	f_getfree("", &fre_clust, &pfs);
//
//    	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
//    	sprintf (buffer, "SD CARD Total Size: \t%lu\n",total);
//    	send_uart(buffer);
//    	clear_buffer();
//    	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
//    	sprintf (buffer, "SD CARD Free Space: \t%lu\n\n",free_space);
//    	send_uart(buffer);
//    	clear_buffer();
//
//
//
//    	/************* The following operation is using PUTS and GETS *********************/
//
//    	/* Open file to write/ create a file if it doesn't exist */
//        fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//
//    	/* Writing text */
//    	f_puts("This data is from the FILE1.txt. And it was written using ...f_puts... ", &fil);
//
//    	/* Close file */
//    	fresult = f_close(&fil);
//
//    	if (fresult == FR_OK)send_uart ("File1.txt created and the data is written \n");
//
//    	/* Open file to read */
//    	fresult = f_open(&fil, "file1.txt", FA_READ);
//
//    	/* Read string from the file */
//    	f_gets(buffer, f_size(&fil), &fil);
//
//    	send_uart("File1.txt is opened and it contains the data as shown below\n");
//    	send_uart(buffer);
//    	send_uart("\n\n");
//
//    	/* Close file */
//    	f_close(&fil);
//
//    	clear_buffer();
//
//
//
//
//    	/**************** The following operation is using f_write and f_read **************************/
//
//    	/* Create second file with read write access and open it */
//    	fresult = f_open(&fil, "file2.txt", FA_CREATE_ALWAYS | FA_WRITE);
//
//    	/* Writing text */
//    	strcpy (buffer, "This is File2.txt, written using ...f_write... and it says Hello from Gokhan\n");
//
//    	fresult = f_write(&fil, buffer, bufsize(buffer), &bw);
//
//    	send_uart ("File2.txt created and data is written\n");
//
//    	/* Close file */
//    	f_close(&fil);
//
//
//
//    	// clearing buffer to show that result obtained is from the file
//    	clear_buffer();
//
//    	/* Open second file to read */
//    	fresult = f_open(&fil, "file2.txt", FA_READ);
//    	if (fresult == FR_OK)send_uart ("file2.txt is open and the data is shown below\n");
//
//    	/* Read data from the file
//    	 * Please see the function details for the arguments */
//    	f_read (&fil, buffer, f_size(&fil), &br);
//    	send_uart(buffer);
//    	send_uart("\n\n");
//
//    	/* Close file */
//    	f_close(&fil);
//
//    	clear_buffer();
//
//
//    	/*********************UPDATING an existing file ***************************/
//
//    	/* Open the file with write access */
//    	fresult = f_open(&fil, "file2.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
//
//    	/* Move to offset to the end of the file */
//    	fresult = f_lseek(&fil, f_size(&fil));
//
//    	if (fresult == FR_OK)send_uart ("About to update the file2.txt\n");
//
//    	/* write the string to the file */
//    	fresult = f_puts("This is updated data and it should be in the end", &fil);
//
//    	f_close (&fil);
//
//    	clear_buffer();
//
//    	/* Open to read the file */
//    	fresult = f_open (&fil, "file2.txt", FA_READ);
//
//    	/* Read string from the file */
//    	fresult = f_read (&fil, buffer, f_size(&fil), &br);
//    	if (fresult == FR_OK)send_uart ("Below is the data from updated file2.txt\n");
//    	send_uart(buffer);
//    	send_uart("\n\n");
//
//    	/* Close file */
//    	f_close(&fil);
//
//    	clear_buffer();
//
//
//    	/*************************REMOVING FILES FROM THE DIRECTORY ****************************/
//
//    	fresult = f_unlink("/file1.txt");
//    	if (fresult == FR_OK) send_uart("file1.txt removed successfully...\n");
//
//    	fresult = f_unlink("/file2.txt");
//    	if (fresult == FR_OK) send_uart("file2.txt removed successfully...\n");
//
//    	/* Unmount SDCARD */
//    	fresult = f_mount(NULL, "/", 1);
//    	if (fresult == FR_OK) send_uart ("SD CARD UNMOUNTED successfully...\n");

////////////////////////////////////////////////////////////////////////////////////////////////////
  HAL_TIM_Base_Start_IT(&htim2);
//////////////////////////////////////////////////////////////////////////////////////////////////
  lwgps_init(&gps);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
///////////////////////////////////////////////////////
  // Initiation
     max30102_init(&max30102, &hi2c1);
     max30102_reset(&max30102);
     max30102_clear_fifo(&max30102);
     max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);

     // Sensor settings
     max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
     max30102_set_adc_resolution(&max30102, max30102_adc_2048);
     max30102_set_sampling_rate(&max30102, max30102_sr_800);
     max30102_set_led_current_1(&max30102, 6.2);
     max30102_set_led_current_2(&max30102, 6.2);

     // Enter SpO2 mode
     max30102_set_mode(&max30102, max30102_spo2);
     max30102_set_a_full(&max30102, 1);

     // Initiate 1 temperature measurement
     max30102_set_die_temp_en(&max30102, 1);
     max30102_set_die_temp_rdy(&max30102, 1);

     uint8_t en_reg[2] = {0};
     max30102_read(&max30102, 0x00, en_reg, 1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /////////////////// oksimetre ////////////////
	  if (max30102_has_interrupt(&max30102))
	  	  // Run interrupt handler to read FIFO
	  	  max30102_interrupt_handler(&max30102);
	  ///////////// BME 280 ////////////////////////
	  BME280_Measure();
	  /////////////////////   DS18B20    //////////////////
	  DS18B20_Start ();
	  HAL_Delay (1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t
	  HAL_Delay (800);  //RTOS GEREKLI

	  DS18B20_Start ();
	  HAL_Delay(1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad

	  Temp_byte1 = DS18B20_Read();
	  Temp_byte2 = DS18B20_Read();
	  TEMP = (Temp_byte2<<8)|Temp_byte1;
	  Temperature_DS = (float)TEMP/16;   //DS sporcunun vucüt sıcaklığını ölcüyor
	  /////////////////////////////////////////
      //Partsiz kayit alma
	  ////////////////////////////////////////////////////////////////////SD CARD mount  NOT:TEK DIZINDE TOPLAM 7KB USTU DOSYA(YAKLASIK 180 SATIR) ISTEMIYOR GIBI
      //////// BU SATIRLAR ACIKKEN SD CARD TAKILI OLMAZSA DONUYOR ////////////////////////////////
        	sprintf (kayit_dosya_ismi,"file_1.txt");
	        fresult = f_mount(&fs, "/", 1);
	          ///////////////////////////////////////////////////////////////////////////////  //dosya olusturma
	      	fresult = f_open(&fil, kayit_dosya_ismi, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
        	fresult = f_close(&fil); //kapama
	      	////////////////////////////////////////////////////////////////////////////////// dosya sonuna yazi ekleme
	      	fresult = f_open(&fil, kayit_dosya_ismi, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	      	/* Move to offset to the end of the file */
	      	fresult = f_lseek(&fil, f_size(&fil));
	      	/* write the string to the file */
	      	if(indx == 0)
	      	{
	      		sprintf (yazi,"Index,Ortam Sicakligi,Nem,Basinc,Vucut Sicakligi,Nabiz(bpm)\n");
	      	}
	      	if (indx > 0)
	      	{
	      	sprintf (yazi,"%d,%.2f,%.2f,%.2f,%.2f,%d \n", indx, Temperature, Humidity, Pressure, Temperature_DS,nabiz_bpm);
	      	}
	      	indx++;
	      	fresult = f_puts(yazi , &fil);
	      	f_close (&fil);
	      	////////////////////////////////////////////////////////////////////////   sd card unmount
	      	fresult = f_mount(NULL, "/", 1);
	        /////////////////////////////////////////
	      	HAL_Delay (100);  // 1000 ms,100 ms de hardfaulta girmedi

///////////////////////////////////////////////////////////////////////
	  ///  Partli kayit alma
	  ////////////////////////////////////////////////////////////////////SD CARD mount
//	          if(part==1)
//	          {
//	          	sprintf (kayit_dosya_ismi,"/part1/file_%d.txt", part);
//	  	        fresult = f_mount(&fs, "/", 1);
//	  	          ///////////////////////////////////////////////////////////////////////////////  //dosya olusturma
//	  	      	fresult = f_open(&fil, kayit_dosya_ismi, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//	          	fresult = f_close(&fil); //kapama
//	  	      	////////////////////////////////////////////////////////////////////////////////// dosya sonuna yazi ekleme
//	  	      	fresult = f_open(&fil, kayit_dosya_ismi, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
//	  	      	/* Move to offset to the end of the file */
//	  	      	fresult = f_lseek(&fil, f_size(&fil));
//	  	      	/* write the string to the file */
//	  	      	if(indx == 0)
//	  	      	{
//	  	      		sprintf (yazi,"Index,Ortam Sicakligi,Nem,Basinc,Vucut Sicakligi\n");
//	  	      	}
//	  	      	if (indx > 0)
//	  	      	{
//	  	      	sprintf (yazi,"%d,%.2f,%.2f,%.2f,%.2f \n", indx, Temperature, Humidity, Pressure, Temperature_DS);
//	  	      	}
//	  	      	//strcpy(yazi, "Hello, world!");
//	  	      	indx++;
//	  	      	indx_counter++;
//	  	      	fresult = f_puts(yazi , &fil);
//	  	      	f_close (&fil);
//	  	      	clear_buffer();
//	  	      	////////////////////////////////////////////////////////////////////////   sd card unmount
//	  	      	fresult = f_mount(NULL, "/", 1);
//	  	        /////////////////////////////////////////
//	  	      	HAL_Delay (100);  // 1000 ms,100 ms de hardfaulta girmedi
//	          }
//	          ///////////////////////////////////////////////////////////////////////////////////////////
//	          if(indx_counter>150)
//	          {
//	          	indx_counter=0;
//	          	part++;
//	          }
//	          ////////////////////////////////////////////////////////////////////////////////////////
//	          if(part>1 )
//	          {
//	          sprintf (kayit_dosya_ismi,"/part%d/file_%d.txt", part, part);
//	          fresult = f_mount(&fs, "/", 1);
//	          ///////////////////////////////////////////////////////////////////////////////  //dosya olusturma
//	          fresult = f_open(&fil, kayit_dosya_ismi, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//	          fresult = f_close(&fil); //kapama
//	          ////////////////////////////////////////////////////////////////////////////////// dosya sonuna yazi ekleme
//	          fresult = f_open(&fil, kayit_dosya_ismi, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
//	          /* Move to offset to the end of the file */
//	          fresult = f_lseek(&fil, f_size(&fil));
//	          /* write the string to the file */
//	          sprintf (yazi,"%d,%.2f,%.2f,%.2f,%.2f \n", indx, Temperature, Humidity, Pressure, Temperature_DS);
//	          //strcpy(yazi, "Hello, world!");
//	          indx++;
//	          indx_counter++;
//	          fresult = f_puts(yazi , &fil);
//	          f_close (&fil);
//	          clear_buffer();
//	          ////////////////////////////////////////////////////////////////////////   sd card unmount
//	          fresult = f_mount(NULL, "/", 1);
//	          /////////////////////////////////////////
//	          HAL_Delay (100);  // 1000 ms,100 ms de hardfaulta girmedi
//	          }
//////////////////////////////////////////////////////////////////////////////////////////////////////

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS_Sensor_GPIO_Port, DS_Sensor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DS_Sensor_Pin */
  GPIO_InitStruct.Pin = DS_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS_Sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Oksimetre_Interrupt_Pin */
  GPIO_InitStruct.Pin = Oksimetre_Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Oksimetre_Interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  //Nabiz hesaplama,Not:EXTI hattında oksimetre 2(counteroutside)*6=12 bpm ekliyor.
{
	counterOutside++; //For testing only
	  currentMillis = HAL_GetTick();
	  if (GPIO_Pin == GPIO_PIN_13 && (currentMillis - previousMillis > 30))
	  {
	    counterInside++; //For testing only
	    previousMillis = currentMillis;
	  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM2)
	{
		milisaniye++;
		if(milisaniye==10000)
		{
			nabiz_bpm=counterOutside*6;
			milisaniye=0;
			counterOutside=0;
			counterInside=0;
		}
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
