/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DSP/SigmaStudioFW.h"

#include "DSP/DSP_IC_1.h"
#include "DSP/DSP_IC_1_PARAM.h"

#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_POT 15
#define VOL_ARRAY 11
#define LOUD_LOW_ARRAY 12
#define LOUD_HIGH_ARRAY 13
#define LOUD_GRL_ARRAY 14

#define ADC_ADDR 0x94
#define DAC_ADDR 0x98
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef stat;

uint8_t auxData[2];

uint32_t value[ADC_POT]; // To store ADC values
uint16_t pote[ADC_POT];  // Potentiometer values 0-29
uint16_t log_in_table[30];
uint16_t linear_in_table[30];
uint16_t flag[ADC_POT];  // Flag for different potentiometers
uint8_t update = 0;

ADI_REG_TYPE aux[4];
ADI_REG_TYPE data_SafeLoad[4]; // Per slot
ADI_REG_TYPE address_SafeLoad[4]; // 2 Bytes per address
ADI_REG_TYPE num_SafeLoad[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void delay_us(uint16_t us);
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
	  uint16_t k = 0;
	  uint16_t pote_aux = 0;
	  uint16_t BandAddress[ADC_POT]; // Addresses of filters
	  uint32_t vol_data[30]; // Fixed volume values
	  uint32_t boost_data[30]; // Fixed boost loudness values
	  uint32_t comp_data[30]; // Fixed compensation values

	  BandAddress[0] = MOD_BAND32_SEL_DCINPALG145X1VALUE_ADDR;
	  BandAddress[1] = MOD_BAND64_SEL_DCINPALG145X2VALUE_ADDR;
	  BandAddress[2] = MOD_BAND128_SEL_DCINPALG145X3VALUE_ADDR;
	  BandAddress[3] = MOD_BAND256_SEL_DCINPALG145X4VALUE_ADDR;
	  BandAddress[4] = MOD_BAND512_SEL_DCINPALG145X5VALUE_ADDR;
	  BandAddress[5] = MOD_BAND1K_SEL_DCINPALG145X6VALUE_ADDR;
	  BandAddress[6] = MOD_BAND2K_SEL_DCINPALG145X7VALUE_ADDR;
	  BandAddress[7] = MOD_BAND4K_SEL_DCINPALG145X8VALUE_ADDR;
	  BandAddress[8] = MOD_BAND8K_SEL_DCINPALG145X9VALUE_ADDR;
	  BandAddress[9] = MOD_BAND16K_SEL_DCINPALG145X10VALUE_ADDR;
	  BandAddress[10] = MOD_BANDSUB_SEL_DCINPALG145X11VALUE_ADDR;
	  BandAddress[VOL_ARRAY] = MOD_VOL_GAINALGNS145X1GAIN_ADDR;
	  BandAddress[LOUD_LOW_ARRAY] = MOD_LOUD_ALG0_LEVEL0_ADDR;
	  BandAddress[LOUD_HIGH_ARRAY] = MOD_LOUD_ALG0_LEVEL1_ADDR;
	  BandAddress[LOUD_GRL_ARRAY] = MOD_LOUD_SEL_DCINPALG145X12VALUE_ADDR;

	  // 8.24 FixPoint
	  vol_data[29] = 0x01000000; // 0dB
	  vol_data[28] = 0x00e42904; // -1dB
	  vol_data[27] = 0x00cb5918; // -2dB
	  vol_data[26] = 0x00b53bee; // -3dB
	  vol_data[25] = 0x00a1866a; // -4dB
	  vol_data[24] = 0x008ff598; // -5dB
	  vol_data[23] = 0x00804dce; // -6dB
	  vol_data[22] = 0x006fbf80; // -7dB (-7.2dB)
	  vol_data[21] = 0x0065ea58; // -8dB
	  vol_data[20] = 0x005ad50c; // -9dB
	  vol_data[19] = 0x0050f44c; // -10dB
	  vol_data[18] = 0x0048268c; // -11dB
	  vol_data[17] = 0x00404de6; // -12dB
	  vol_data[16] = 0x00394fae; // -13dB
	  vol_data[15] = 0x00331426; // -14dB
	  vol_data[14] = 0x002d8620; // -15dB
	  vol_data[13] = 0x002892c0; // -16dB
	  vol_data[12] = 0x00242934; // -17dB
	  vol_data[11] = 0x00203a7e; // -18dB
	  vol_data[10] = 0x001cb942; // -19dB
	  vol_data[9] = 0x00199998; // -20dB
	  vol_data[8] = 0x0016d0e6; // -21dB
	  vol_data[7] = 0x001455b4; // -22dB
	  vol_data[6] = 0x00121f96; // -23dB
	  vol_data[5] = 0x0010270a; // -24dB
	  vol_data[4] = 0x000e655c; // -25dB
	  vol_data[3] = 0x000cd494; // -26dB
	  vol_data[2] = 0x000b6f62; // -27dB
	  vol_data[1] = 0x000a3108; // -28dB
	  vol_data[0] = 0x0009154e; // -29dB

	  // 8.24 Compensation FixPoint
	  comp_data[29] = 0x01000000; // 0dB
	  comp_data[28] = 0x011F3B64; // +1dB
	  comp_data[27] = 0x014248E8; // +2dB
	  comp_data[26] = 0x01699C0F; // +3dB
	  comp_data[25] = 0x0195BB8C; // +4dB
	  comp_data[24] = 0x01C73D51; // +5dB
	  comp_data[23] = 0x02000000; // +6dB
	  comp_data[22] = 0x023D1CD1; // +7dB
	  comp_data[21] = 0x02830AFD; // +8dB
	  comp_data[20] = 0x02D1818B; // +9dB
	  comp_data[19] = 0x03298B07; // +10dB
	  comp_data[18] = 0x038C5280; // +11dB
	  comp_data[17] = 0x03FB2783; // +12dB
	  comp_data[16] = 0x0477828F; // +13dB
	  comp_data[15] = 0x05030A10; // +14dB
	  comp_data[14] = 0x059F9802; // +15dB
	  comp_data[13] = 0x064F4034; // +16dB
	  comp_data[12] = 0x07145759; // +17dB
	  comp_data[11] = 0x07F17AF3; // +18dB
	  comp_data[10] = 0x08E99A36; // +19dB
	  comp_data[9] = 0x0A000000; // +20dB
	  comp_data[8] = 0x0B385E03; // +21dB
	  comp_data[7] = 0x0C96D95B; // +22dB
	  comp_data[6] = 0x0E20189B; // +23dB
	  comp_data[5] = 0x0FD9539A; // +24dB
	  comp_data[4] = 0x11C86531; // +25dB
	  comp_data[3] = 0x13F3DF1C; // +26dB
	  comp_data[2] = 0x16632049; // +27dB
	  comp_data[1] = 0x191E6DE4; // +28dB
	  comp_data[0] = 0x1C2F0F70; // +29dB

	  // 8.24 FixPoint
	  boost_data[29] = 0x02800000; // 2.50
	  boost_data[28] = 0x02666666; // 2.40
	  boost_data[27] = 0x02570a3c; // 2.34
	  boost_data[26] = 0x02451eb8; // 2.27
	  boost_data[25] = 0x02333332; // 2.20
	  boost_data[24] = 0x022147ae; // 2.13
	  boost_data[23] = 0x020f5c28; // 2.06
	  boost_data[22] = 0x02000000; // 2.00
	  boost_data[21] = 0x01ee147a; // 1.93
	  boost_data[20] = 0x01dc28f4; // 1.86
	  boost_data[19] = 0x01ca3d70; // 1.79
	  boost_data[18] = 0x01b851ea; // 1.72
	  boost_data[17] = 0x01a8f5c2; // 1.66
	  boost_data[16] = 0x01970a3c; // 1.59
	  boost_data[15] = 0x01851eb8; // 1.52
	  boost_data[14] = 0x01733332; // 1.45
	  boost_data[13] = 0x016147ae; // 1.38
	  boost_data[12] = 0x0151eb84; // 1.32
	  boost_data[11] = 0x01400000; // 1.25
	  boost_data[10] = 0x012e147a; // 1.18
	  boost_data[9] =  0x011c28f4; // 1.11
	  boost_data[8] =  0x010a3d70; // 1.04
	  boost_data[7] =  0x00fae146; // 0.98
	  boost_data[6] =  0x00e8f5c2; // 0.91
	  boost_data[5] =  0x00d70a3c; // 0.84
	  boost_data[4] =  0x00c51eb8; // 0.77
	  boost_data[3] =  0x00b33332; // 0.70
	  boost_data[2] =  0x00a3d70a; // 0.64
	  boost_data[1] =  0x0091eb84; // 0.57
	  boost_data[0] =  0x00800000; // 0.50

	  for(k=0; k<30; k++)
	  {
		  log_in_table[k] = 4096.0*log10(1.0+(3.0*k/10.0));
		  linear_in_table[k] = 4096*k/30;
	  }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//  - Keeping this just to remember order
//  MX_GPIO_Init();
//  MX_I2C1_Init();
//  MX_DMA_Init();
//  MX_I2C2_Init();
//  MX_ADC1_Init();
//  MX_I2C3_Init();
//  MX_TIM2_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim4);
  // Disable DSP
  HAL_GPIO_WritePin(nRST_DSP_GPIO_Port, nRST_DSP_Pin, GPIO_PIN_RESET);

  // Disable CLK
  HAL_GPIO_WritePin(EN_SCK_GPIO_Port, EN_SCK_Pin, GPIO_PIN_RESET);

  // Configure Sampling Rate SR = Standard => 0
  //							  Double   => 1
  HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_SET);
  // Configure System Clock SCKO1 CSEL = 0 (default)
  HAL_GPIO_WritePin(CSEL_GPIO_Port, CSEL_Pin, GPIO_PIN_RESET);
  // Configure Sampling Frequency Group = 32KHz => 10
  HAL_GPIO_WritePin(FS2_GPIO_Port, FS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FS1_GPIO_Port, FS1_Pin, GPIO_PIN_RESET);

  // Enable CLK
  HAL_GPIO_WritePin(EN_SCK_GPIO_Port, EN_SCK_Pin, GPIO_PIN_SET);

  HAL_Delay(500);

  // Configure DAC format to I2S 16-24 bit, slow roll-off
  auxData[0] = 0b00000100;
  auxData[1] = 0x00;
  stat = HAL_I2C_Mem_Write(&hi2c3, DAC_ADDR, 0x09, 1, auxData, 1, 1000);

  auxData[0] = 0b00101001;
  auxData[1] = 0x00;
  stat = HAL_I2C_Mem_Write(&hi2c3, DAC_ADDR, 0x0A, 1, auxData, 1, 1000);

  // Configure DAC over-sampling wide, sharp roll-off
  auxData[0] = 0b10000000;
  auxData[1] = 0x00;
  stat = HAL_I2C_Mem_Write(&hi2c3, DAC_ADDR, 0x0C, 1, auxData, 1, 1000);

  // Enable DSP
  HAL_GPIO_WritePin(nRST_DSP_GPIO_Port, nRST_DSP_Pin, GPIO_PIN_SET);

  HAL_Delay(100);

  default_download_IC_1();
  HAL_Delay(500);
  //Configure ADCs clock settings
//  auxData[0] = 0b01000000;
//  stat = HAL_I2C_Mem_Write(&hi2c1, ADC_ADDR, 0x20, 1, auxData, 1, 1000);
//
//  auxData[0] = 0b00000001;
//  stat = HAL_I2C_Mem_Write(&hi2c1, ADC_ADDR, 0x0D, 1, auxData, 1, 1000);
//  stat = HAL_I2C_Mem_Write(&hi2c2, ADC_ADDR, 0x0D, 1, auxData, 1, 1000);
//  stat = HAL_I2C_Mem_Write(&hi2c3, ADC_ADDR, 0x0D, 1, auxData, 1, 1000);
//
//  stat = HAL_I2C_Mem_Write(&hi2c1, ADC_ADDR, 0x0E, 1, auxData, 1, 1000);
//  stat = HAL_I2C_Mem_Write(&hi2c2, ADC_ADDR, 0x0E, 1, auxData, 1, 1000);
//  stat = HAL_I2C_Mem_Write(&hi2c3, ADC_ADDR, 0x0E, 1, auxData, 1, 1000);

  for(k=0; k<ADC_POT; k++)
  {
	  flag[k] = 1;
  }

  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, value, ADC_POT);

  HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  data_SafeLoad[2] = 0x00;
	  data_SafeLoad[1] = 0x00;
	  data_SafeLoad[0] = 0x00;

	  address_SafeLoad[1] = 0x00;
	  address_SafeLoad[0] = 0x00;

	  num_SafeLoad[3] = 0x01;
 	  num_SafeLoad[2] = 0x00;
	  num_SafeLoad[1] = 0x00;
	  num_SafeLoad[0] = 0x00;

	  if(update == 1)
	  {
		  for(k=0; k<VOL_ARRAY; k++) // Filters 32Hz - 16KHz + Subwoofer
		  {
			  if(flag[k] == 1)
			  {
				  flag[k] = 0;
				  data_SafeLoad[3] = 29 - pote[k];
				  address_SafeLoad[3] = 0xFF & (BandAddress[k]);
				  address_SafeLoad[2] = 0xFF & ((BandAddress[k])>>8);
				  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
				  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
				  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
				  delay_us(100);
			  }

		  }

		  if(flag[VOL_ARRAY] == 1)
		  {
			  flag[VOL_ARRAY] = 0;
			  pote_aux = 29 - pote[VOL_ARRAY];

			  data_SafeLoad[3] = 0xFF & (vol_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((vol_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((vol_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((vol_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (BandAddress[VOL_ARRAY]);
			  address_SafeLoad[2] = 0xFF & ((BandAddress[VOL_ARRAY])>>8);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
			  delay_us(100);
		  }

		  if(flag[LOUD_LOW_ARRAY] == 1)
		  {
			  flag[LOUD_LOW_ARRAY] = 0;
			  pote_aux = 29 - pote[LOUD_LOW_ARRAY];

			  data_SafeLoad[3] = 0xFF & (boost_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((boost_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((boost_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((boost_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (BandAddress[LOUD_LOW_ARRAY]);
			  address_SafeLoad[2] = 0xFF & ((BandAddress[LOUD_LOW_ARRAY])>>8);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
			  delay_us(100);
		  }

		  if(flag[LOUD_HIGH_ARRAY] == 1)
		  {
			  flag[LOUD_HIGH_ARRAY] = 0;
			  pote_aux = 29 - pote[LOUD_HIGH_ARRAY];

			  data_SafeLoad[3] = 0xFF & (boost_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((boost_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((boost_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((boost_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (BandAddress[LOUD_HIGH_ARRAY]);
			  address_SafeLoad[2] = 0xFF & ((BandAddress[LOUD_HIGH_ARRAY])>>8);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
			  delay_us(100);
		  }

		  if(flag[LOUD_GRL_ARRAY] == 1)
		  {
			  flag[LOUD_GRL_ARRAY] = 0;
			  pote_aux = pote[LOUD_GRL_ARRAY];

			  data_SafeLoad[3] = 0xFF & (vol_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((vol_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((vol_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((vol_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (BandAddress[LOUD_GRL_ARRAY]);
			  address_SafeLoad[2] = 0xFF & ((BandAddress[LOUD_GRL_ARRAY])>>8);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
			  delay_us(100);

			  data_SafeLoad[3] = 0xFF & (comp_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((comp_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((comp_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((comp_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (MOD_LOUDCOMP_GAINALGNS145X2GAIN_ADDR);
			  address_SafeLoad[2] = 0xFF & ((MOD_LOUDCOMP_GAINALGNS145X2GAIN_ADDR)>>8);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
			  SIGMA_WRITE_REGISTER_BLOCK(DEVICE_ADDR_IC_1, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
			  delay_us(100);
		  }

		  update = 0;
		  HAL_ADC_Start_DMA(&hadc1, value, ADC_POT);
	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 27;
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 15;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 15;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hi2c2.Init.ClockSpeed = 400000;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 123;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nRST_DSP_GPIO_Port, nRST_DSP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_SCK_GPIO_Port, EN_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSEL_Pin|SR_Pin|FS2_Pin|FS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : nRST_DSP_Pin EN_SCK_Pin */
  GPIO_InitStruct.Pin = nRST_DSP_Pin|EN_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CSEL_Pin SR_Pin FS2_Pin FS1_Pin */
  GPIO_InitStruct.Pin = CSEL_Pin|SR_Pin|FS2_Pin|FS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO0_Pin */
  GPIO_InitStruct.Pin = GPIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO1_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SIGMA_WRITE_REGISTER_BLOCK(uint16_t devAddress, uint16_t address, uint16_t length, uint8_t *pData)
{
	stat = HAL_I2C_Mem_Write(&hi2c1, devAddress, address, 2, pData, length, 1000);
}

void SIGMA_WRITE_DELAY(uint16_t devAddress, uint16_t length, uint8_t *pData)
{
	HAL_Delay(11);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t k = 0;


	for(k=VOL_ARRAY; k<ADC_POT; k++) // For volume, loud low, loud high, loud general
	{
		for(i=0; i<30; i++)
		{
			if((i == 0) && (value[k] < (linear_in_table[i+1])-15))
			{
				if(pote[k] != i)
				{
					pote[k] = i;
					flag[k] = 1;
				}
			}
			else if((i > 0) && (i < 29) && (value[k] > (linear_in_table[i]+15)) && (value[k] < (linear_in_table[i+1])-15))
			{
				if(pote[k] != i)
				{
					pote[k] = i;
					flag[k] = 1;
				}
			}
			else if((i == 29) && (value[k] > (linear_in_table[i]+15)))
			{
				if(pote[k] != i)
				{
					pote[k] = i;
					flag[k] = 1;
				}
			}
		}
	}


	for(j=0; j<(VOL_ARRAY); j++) // For filter 32Hz - 16KHz + Subwoofer
	{
		for(i=0; i<30; i++)
		{
			if((i == 0) && (value[j] < (log_in_table[i+1])-15))
			{
				if(pote[j] != i)
				{
					pote[j] = i;
					flag[j] = 1;
				}
			}
			else if((i > 0) && (i < 29) && (value[j] > (log_in_table[i]+15)) && (value[j] < (log_in_table[i+1])-15))
			{
				if(pote[j] != i)
				{
					pote[j] = i;
					flag[j] = 1;
				}
			}
			else if((i == 29) && (value[j] > (log_in_table[i]+15)))
			{
				if(pote[j] != i)
				{
					pote[j] = i;
					flag[j] = 1;
				}
			}
		}

	}

	update = 1;
}

void delay_us(uint16_t us)
{
	htim4.Instance->CNT = 0;
	while((htim4.Instance->CNT) < us);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
