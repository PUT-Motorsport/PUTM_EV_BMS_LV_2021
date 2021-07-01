/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMPERATURE_WARNING_LIMIT	45.0f
#define TEMPERATURE_ERROR_LIMIT	50.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

uint8_t ltcConfig[6] = {0xFC, (uint8_t)(1874 & 0xff), (uint8_t)((1874>>4)|(2625<<4)), (uint8_t)(2625>>4), 0, 0};

uint8_t acuState=0;
uint16_t cellValues[6];
uint16_t lowestValue;
uint8_t cellValuesCAN[6];
uint16_t cellValuesErr[6];
uint32_t cellValuesSum = 0;
uint8_t cellValuesSumCAN = 0;
uint8_t cellDischarge[6];
uint16_t tempValues[6];
uint16_t tempValuesAvr = 0;
uint8_t canFlag = 0;
uint8_t errorFlag = 0;
float tempValuesFloat[6];
volatile uint16_t adcValues[6];


uint32_t ltc_refresh_next_tick = 0;
uint8_t ltc_new_data = 0;

uint8_t discharge_activation=0;

uint8_t balance_activation = 0;
uint8_t balance_test = 0;
float balance_cell_target = 41.5;

const int temperatureMap[26][2] = {
//		ltc value ,  temperature *C
		{4096	,	-50} ,
		{3713	,	-20} ,
		{3602	,	-15} ,
		{3469	,	-10} ,
		{3313	,	-5}	,
		{3136	,	0}	,
		{2939	,	5}	,
		{2726	,	10}	,
		{2503	,	15}	,
		{2275	,	20}	,
		{2048	,	25}	,
		{1828	,	30}	,
		{1618	,	35}	,
		{1424	,	40}	,
		{1245	,	45}	,
		{1085	,	50}	,
		{942	,	55}	,
		{816	,	60}	,
		{706	,	65}	,
		{611	,	70}	,
		{528	,	75}	,
		{458	,	80}	,
		{397	,	85}	,
		{344	,	90}	,
		{299	,	95}	,
		{261	,	100}

};

uint16_t cell_voltage_target = 38000;

float cell_voltages[6];
float cell_temperatures[6];
uint8_t cellDischarge[6];
float total_voltage = 0.0;

uint8_t caution_low_voltage = 0;
uint8_t caution_high_voltage = 0;
uint8_t caution_high_temp = 0;
uint8_t caution_low_temp = 0;

uint8_t can_20hz_flag = 0;

float minimum_voltage = 0.0;
float max_temperature = 0.0;


uint8_t errorCounter1[6] = {0,0,0,0,0,0};
uint8_t errorCounter2[6] = {0,0,0,0,0,0};
uint8_t errorCounter3[6] = {0,0,0,0,0,0};
uint8_t errorCounter4[6] = {0,0,0,0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pec15Table[256];
uint16_t CRC15_POLY = 0x4599;
void init_PEC15_Table()
{
	uint16_t remainder;
	for (int i = 0; i < 256; i++)
	{
		remainder = i << 7;
		for (int bit = 8; bit > 0; --bit)
		{
			if (remainder & 0x4000)
			{
				remainder = ((remainder << 1));
				remainder = (remainder ^ CRC15_POLY);
			}
			else
			{
				remainder = ((remainder << 1));
			}
		}
		pec15Table[i] = remainder&0xFFFF;
	}
}

uint16_t pec15(char *data , int len)
{
	uint16_t remainder,address;
	remainder = 16;//PEC seed
	for (int i = 0; i < len; i++)
	{
		address = ((remainder >> 7) ^ data[i]) & 0xff;//calculate PEC table address
		remainder = (remainder << 8 ) ^ pec15Table[address];
	}
	return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}



/**
 * Brief:	Send wakeup for LTC, BLOCKING MODE
 * Param:	None
 * Retval:	None
 */
void LTC_Wakeup()
{
	uint8_t tab[2] = {0xFF};

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 2, 1);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);
}


/**
 * Brief:	Send adc config for ltc and start conversion, BLOCKING MODE
 * Param:	None
 * Retval:	None
 */
void LTC_StartCellADC()
{
		//uint8_t *tab;
		uint8_t tab[12];
		uint16_t pec;
		//tab = malloc(12); // 4 + n * (6 data + 2 pec)

		uint16_t cmd = (1<<15) | 0x01;
		// configuration
		tab[0] = (cmd>>8);
		tab[1] = cmd;
		pec = pec15((char*)tab, 2);
		tab[2] = pec >> 8;
		tab[3] = pec;

		tab[4] = ltcConfig[0];
		tab[5] = ltcConfig[1];
		tab[6] = ltcConfig[2];
		tab[7] = ltcConfig[3];
		tab[8] = ltcConfig[4];
		tab[9] = ltcConfig[5];
		pec = pec15((char*)&tab[4], 6);
		tab[10] = pec >> 8;
		tab[11] = pec;

	  LTC_Wakeup();

	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, tab, 12, 100);
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);


	  // adc conversion
	  memset(tab, 0, 12);

	  cmd = 0b1001100000 | (0b00 << 7); // discharge not permitted
	  //uint16_t cmd = 0b1001110000 | (0b00 << 7); // discharge permitted
	  tab[0] = cmd>>8;
	  tab[1] = cmd;
	  pec = pec15((char*)tab, 2);
	  tab[2] = pec >> 8;
	  tab[3] = pec;

	  //LTC_Wakeup();

	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi1, tab, 4, 100);
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);

	  //free(tab);
}



/**
 * Brief:	Receveing adc data from ltc, BLOCKING MODE
 * Param:	None
 * Retval:	None
 */
void LTC_GetValuesADC()
{
	  //uint8_t *tab, *rx_tab;
	  uint8_t tab[100], rx_tab[100];
	  uint16_t pec;//, rx_pec;

	  //tab = malloc(12);
	  //rx_tab = malloc(12);

	  // read cell voltage group A
	  uint16_t cmd = (1<<15) | 0b100;
	  memset(tab, 0, 12);
	  tab[0] = (cmd>>8);
	  tab[1] = cmd;
	  pec = pec15((char*)tab, 2);
	  tab[2] = pec >> 8;
	  tab[3] = pec;

	  LTC_Wakeup();

	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tab, rx_tab, 12, 100);
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);


	  cellValues[0] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	  cellValues[1] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	  cellValues[2] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);


	  // read cell voltage group B
	  cmd = (1<<15) | 0b110;
	  memset(tab, 0, 12);
	  tab[0] = (cmd>>8);
	  tab[1] = cmd;
	  pec = pec15((char*)tab, 2);
	  tab[2] = pec >> 8;
	  tab[3] = pec;

	  //LTC_Wakeup();

	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tab, rx_tab, 12, 100);
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);

		cellValues[3] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
		cellValues[4] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
		cellValues[5] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);

		cellValuesSum  = cellValues[0];

		//cell calculations
		for(int i = 1; i < 6; i++)
		{
				cellValuesSum = cellValuesSum + cellValues[i];
		}
		cellValuesSumCAN = cellValuesSum / 1000;
		//free(tab);
		//free(rx_tab);

}


/**
 * Brief:	Calculation of temperature, from value of measured voltage
 * Param:	ltc_value:	Value of voltage, LSB -> 0.1 mV
 * Retval:	Temperature in *C
 */
float tempCalculate(uint16_t ltc_value)
{
	float retval = 0.0;
	for(int i = 1; i < 28; i++)
	{
		if(ltc_value >= (uint16_t)temperatureMap[i][0])
		{
			// approximation
			retval = (float)temperatureMap[i][1] - 5.0 * ((float)ltc_value-(float)temperatureMap[i][0]) / ((float)temperatureMap[i-1][0] - (float)temperatureMap[i][0]);
			break;
		}
	}
	return retval;
}

void GetTemp()
{
	tempValuesAvr=0;
	for(int i = 0; i < 6; i++)
	  		  		  {
	  		  			  tempValues[i]=tempCalculate(adcValues[i]);
	  		  			  tempValuesAvr=tempValuesAvr+tempValues[i];
	  		  		  }
	  		  		  tempValuesAvr=tempValuesAvr/6;
}

/**
 * Brief:	Send discharge configuration and start the discharge, BLOCKING MODE
 * Param:	None
 * Retval:	None
 */
void LTC_TurnOnDischarge()
{
	uint8_t *tab, *rx_tab;
	uint16_t pec;

	tab = malloc(12);
	rx_tab = malloc(12);


	uint16_t cmd = (1<<15) | 0b10100;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;


		  tab[4] = (cellDischarge[0] << 3) | (cellDischarge[1] << 7); // 1, 2
		  tab[5] = (cellDischarge[2] << 3) | (cellDischarge[3] << 7); // 3, 4
		  tab[6] = (cellDischarge[4] << 3) | (cellDischarge[5] << 7); // 5, 6
		  pec = pec15((char*)&tab[4], 3);
		  tab[7] = pec >> 8;
		  tab[8] = pec;

		  LTC_Wakeup();

	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tab, rx_tab, 12, 100);
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);


	  cmd = (1<<15) | 0b11001;
	  memset(tab, 0, 12);

	  tab[0] = (cmd>>8);
	  tab[1] = cmd;
	  pec = pec15((char*)tab, 2);
	  tab[2] = pec >> 8;
	  tab[3] = pec;

	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tab, rx_tab, 4, 100);
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);

	  free(tab);
	  free(rx_tab);

}

/**
 * Brief:	Turn off discharge, BLOCKING MODE
 * Param:	None
 * Retval:	None
 */
void LTC_TurnOffDischarge()
{

		uint8_t *tab, *rx_tab;
		uint16_t pec;

		tab = malloc(12);
		rx_tab = malloc(12);


	uint16_t cmd = (1<<15) | 0b10100;
	  tab[0] = (cmd>>8);
	  tab[1] = cmd;
	  pec = pec15((char*)tab, 2);
	  tab[2] = pec >> 8;
	  tab[3] = pec;

	  tab[4] = 0b00000000;
	  tab[5] = 0b00000000;
	  tab[6] = 0b00000000;
	  pec = pec15((char*)&tab[4], 3);
	  tab[7] = pec >> 8;
	  tab[8] = pec;


	  LTC_Wakeup();

	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tab, rx_tab, 12, 100);
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);

	  memset(tab, 0, 12);
	  cmd = (1<<15) | 0b11001;

	  tab[0] = (cmd>>8);
	  tab[1] = cmd;
	  pec = pec15((char*)tab, 2);
	  tab[2] = pec >> 8;
	  tab[3] = pec;

	  //ltc_wakeup();

	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	  HAL_SPI_TransmitReceive(&hspi1, tab, rx_tab, 4, 100);
	  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);

	  free(tab);
	  free(rx_tab);

}



void can_init()
{
		// CAN 1 FOR CHARGING
	CAN_FilterTypeDef filter;
	filter.FilterActivation = ENABLE;
	filter.FilterBank = 10;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterIdHigh = 0x00;
	filter.FilterIdLow = 0x00;
	filter.FilterMaskIdHigh = 0x00;
	filter.FilterMaskIdLow = 0x00;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.SlaveStartFilterBank = 10;

	HAL_CAN_ConfigFilter(&hcan1, &filter);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		can_20hz_flag = 1;
	}

}

/**
 * @brief	Checking values and switching off safety
 * @param	None
 * @retval	None
 */
void CheckError()
{
	//accumulator

		lowestValue=cellValues[0];
		for(int i = 1; i < 6; i++)
		{
			if(lowestValue > cellValues[i])
			{
				lowestValue = cellValues[i];
			}
		}

		for(int i = 0; i < 6; i++)
				{
					if((cellValues[i] - lowestValue) > 2000)
					{
						  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
					}
				}


		for(int i = 0; i < 6; i++)
		{

			if(cellValues[i] < 35000)
			{
				//HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
				errorCounter1[i]++;
			}else{
				errorCounter1[i]=0;
				acuState=0;
			}
			//if(cellValues[i] < 35.5)
			//	caution_low_voltage = 1;
			if(cellValues[i] > 42300)
			{
				//caution_high_voltage = 1;
				//HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
				errorCounter2[i]++;
			}else{
				errorCounter2[i]=0;
				acuState=0;
			}
			if(tempValues[i] > TEMPERATURE_ERROR_LIMIT)
			{
				//caution_high_voltage = 1;
				//HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
				errorCounter3[i]++;
			}else{
				errorCounter3[i]=0;
				acuState=0;
			}
			if(cellValues[i] < 30000)
						{
							//sleep mode

						}
			if((cellValues[i] - lowestValue) > 2000)
						{
							errorCounter4[i]++;
						}else{
							errorCounter4[i]=0;
							acuState=0;
						}

		}

		for(int i = 0; i < 6; i++)
			  					{
			  						if(errorCounter1[i]>=9)
			  						{
			  							acuState = 0b1; //too low voltage
			  							if(errorCounter1[i]==10)
			  							{
			  								HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
			  							}
			  						}
			  						if(errorCounter2[i]>=9)
			  						{
			  							acuState = 0b11; //too high voltage
			  							if(errorCounter2[i]==10)
			  							{
			  								HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
			  							}
			  						}
			  						if(errorCounter3[i]>=9)
			  						{
			  							acuState = 0b111; //too high temp
			  							if(errorCounter3[i]==10)
			  							{
			  								HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
			  							}
			  						}
			  						if(errorCounter4[i]>=1)
			  						{
			  							acuState = 0b1111; //not balanced
			  							if(errorCounter4[i]==2)
			  							{
			  								HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
			  								HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
			  							}
			  						}
			  						//if(cellValues[i] > 36000 && cellValues[i] <= 42300 && tempValues[i] < 40)
			  						//{
			  						//	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, SET);
			  						//}
			  					}

		//add temp error here

		/*for(int i = 0; i < 6; i++)
		{
			if(cell_temperatures[i] > TEMPERATURE_ERROR_LIMIT)
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
			if(cell_temperatures[i] > TEMPERATURE_WARNING_LIMIT)
				caution_high_temp = 1;
		}*/

}

void CAN_Data1()
{
	CAN_TxHeaderTypeDef tx_header;
	tx_header.DLC = 8;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = 0x0C;
	uint8_t data[8];

	for(int i = 0; i < 6; i++)
	{
		cellValuesCAN[i] = cellValues[i] / 1000;
	}

	data[0] = acuState;
	data[1] = cellValuesSumCAN;
	data[2] = cellValuesCAN[0];
	data[3] = cellValuesCAN[1];
	data[4] = cellValuesCAN[2];
	data[5] = cellValuesCAN[3];
	data[6] = cellValuesCAN[4];
	data[7] = cellValuesCAN[5];

	uint32_t mailbox = 0;
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}

void CAN_Data2()
{
	CAN_TxHeaderTypeDef tx_header;
	tx_header.DLC = 8;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = 0x5F;
	uint8_t data[7];
	data[0] = tempValuesAvr;
	data[1] = tempValues[0];
	data[2] = tempValues[1];
	data[3] = tempValues[2];
	data[4] = tempValues[3];
	data[5] = tempValues[4];
	data[6] = tempValues[5];

	uint32_t mailbox = 0;
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}

void balance()
{
	/*if(balance_activation == 0)
	{
		// turn off discharge only when charging is also off
		if(charging_on == 0) discharge_activation = 0;
		return;
	}*/

	if(balance_test == 0)
	{
			uint8_t balance_at_once = 0;
			for(int i = 0; i < 6; i++)
			{
				if(balance_at_once < 3)
				{
					if(cellValues[i] > balance_cell_target) // balance target voltage
					{
						balance_at_once++;
						cellDischarge[i] = 1;
						discharge_activation = 1;
					}
				}
			}
	}
	else
	{
		static int pos = 0;
		memset(&cellDischarge[0], 0, 6);

			cellDischarge[pos] = 1;

		pos++;
		pos %= 10;
		discharge_activation = 1;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, SET);
  //HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, SET);
  //HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, SET);
  //HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, SET);
  HAL_Delay(3000);

  init_PEC15_Table();

  //HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
  /*while(1){
	  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	  HAL_Delay(1000);
  }*/
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, RESET);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, SET);

  can_init();

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, SET);
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 6);
  
  //LTC_TurnOnDischarge();
  
  ltc_refresh_next_tick = HAL_GetTick();
  while (1)
  {
	  

	  if(ltc_refresh_next_tick <= HAL_GetTick())
	  	  {
	  		  ltc_refresh_next_tick += 50; //50ms loop

	  		  // discharge off while adc
	  		  //LTC_TurnOffDischarge();
	  		  //HAL_Delay(1);

	  		  //ltc_start_cell_adc_2();
	  		  LTC_StartCellADC();
	  		  HAL_Delay(30);

	  		  // on discharge if needed
	  		  //if(discharge_activation == 1)
	  		  //{
	  		  //  LTC_TurnOnDischarge();
	  		  //}

	  		  LTC_GetValuesADC();

	  		  GetTemp();

	  		  ltc_new_data = 1;
	  	  }

	  	  if(ltc_new_data == 1)
	  	  {
	  		  ltc_new_data = 0;

	  		  // accumulator status
	  			CheckError();

	  		  // only balance
	  		  //balance();

	  		  // control balance
	  		  /*if(discharge_activation == 1)
	  		  {
	  			  LTC_TurnOnDischarge();
	  		  }
	  		  else
	  		  {
		  		  LTC_TurnOffDischarge();
	  		  }*/

	  		  //calculate output current
	  		  //calculate_current();
	  		  //analize_accumulator();

	  	  }
	  	  if(can_20hz_flag == 1)
	  	  {
	  		  CAN_Data1();
	  		  //HAL_Delay(1);
	  		  CAN_Data2();
	  		  can_20hz_flag = 0;
	  	  }
	  	  
	  	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, 
	  	HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin));
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_DASH_GPIO_Port, LED_DASH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RELAY_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin LED_3_Pin LED_4_Pin
                           LED_DASH_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_3_Pin|LED_4_Pin
                          |LED_DASH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
