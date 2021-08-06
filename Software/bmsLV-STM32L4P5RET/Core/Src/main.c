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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COEFF_LSB_TO_AMP			0.03222656f
#define TEMPERATURE_WARNING_LIMIT	45.0f
#define TEMPERATURE_LIMIT			50.0f
#define VOL_DOWN_LIMIT				33000 	//3.3V
#define VOL_DOWN_OK					36000 	//3.6V
#define VOL_UP_LIMIT				42400 	//4.24V
#define VOL_UP_OK					42000 	//4.2V
#define MAX_SUM_VOLTAGE 			255000 	// 25.5V
#define NUMBER_OF_CELLS				6
#define NEUTRAL_CURRENT_SENSOR 		485
#define NEUTRAL_CURRENT_CAR_POS 	0.25
#define NEUTRAL_CURRENT_CAR_NEG 	-0.25
#define COUNTER_TO_SLEEP			18000 	//15min -> 15*60*1000/50 where 50 is the checkError loop time
#define UNBALANCE_LIMIT				1000 	// 0.1V
#define BALANCE_VALUE				100 	//0.01V
#define TOO_HIGH_CURRENT			25 		//25A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

uint8_t ltcConfig[6] = {0xFC, (uint8_t)(1874 & 0xff), (uint8_t)((1874>>4)|(2625<<4)), (uint8_t)(2625>>4), 0, 0};
uint8_t acuState;

//voltage
uint16_t cellValues[NUMBER_OF_CELLS];
uint8_t cellValuesCAN[NUMBER_OF_CELLS];
uint32_t cellValuesSum;
uint8_t cellValuesSumCAN;

//temperatures
uint16_t tempValues[NUMBER_OF_CELLS];
uint16_t tempValuesCAN[NUMBER_OF_CELLS];
uint16_t tempValuesAvr;
volatile uint16_t adcValues[NUMBER_OF_CELLS];

//current
uint32_t currentSensor[100] = {0};
float currentSensorAvg;
float chargingCurrent;
float outputCurrent = 0.0;
float outputCurrent_max = 0.0;
float outputCurrent_min = 0.0;
float outputCurrent_historic_max = 0.0;

//flags
uint32_t dataRefreshNextTick = 0;
uint8_t newData = 0;
bool can100HzFlag = 0;

//charge/discharge
uint8_t chargingOn = 0;
uint8_t dischargeActivation=0;
bool cellDischarge[NUMBER_OF_CELLS];
static uint32_t dischargeTickEnd;
uint8_t dischargeAtOnce = 0;

//counters
uint8_t errorCounterLowVol[NUMBER_OF_CELLS];
uint8_t errorCounterHighVol[NUMBER_OF_CELLS];
uint8_t errorCounterHighTemp[NUMBER_OF_CELLS];
uint8_t errorCounterBalance[NUMBER_OF_CELLS];
uint8_t errorCounterHighCur;
uint16_t sleepCounter;

const int temperatureMap[26][2] = {
		//	  adc value ,  temperature *C
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM8_Init(void);
static void MX_RTC_Init(void);
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
void LTC_wakeUp()
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
void LTC_startCellAdc()
{
	uint8_t tab[12];
	uint16_t pec;

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

	LTC_wakeUp();

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 12, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);


	// adc conversion
	memset(tab, 0, 12);

	//cmd = 0b1001100000 | (0b00 << 7); // discharge not permitted
	cmd = 0b1001110000 | (0b00 << 7); // discharge permitted
	tab[0] = cmd>>8;
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 4, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);
}



/**
 * Brief:	Receveing adc data from ltc, BLOCKING MODE
 * Param:	None
 * Retval:	None
 */
void LTC_getValuesAdc()
{
	uint8_t tab[100], rx_tab[100];
	uint16_t pec;

	// read cell voltage group A
	uint16_t cmd = (1<<15) | 0b100;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	LTC_wakeUp();

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

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_TransmitReceive(&hspi1, tab, rx_tab, 12, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);

	cellValues[3] = (uint16_t)rx_tab[4] | (((uint16_t)rx_tab[5])<<8);
	cellValues[4] = (uint16_t)rx_tab[6] | (((uint16_t)rx_tab[7])<<8);
	cellValues[5] = (uint16_t)rx_tab[8] | (((uint16_t)rx_tab[9])<<8);

	cellValuesSum  = cellValues[0];

	//cells sum calculations for CAN
	for(unsigned int i = 1; i < NUMBER_OF_CELLS; i++)
	{
		cellValuesSum += cellValues[i];
	}
	if(cellValuesSum > MAX_SUM_VOLTAGE){
		cellValuesSum = MAX_SUM_VOLTAGE;
	}
	cellValuesSumCAN = cellValuesSum / 1000;

}


/**
 * Brief:	Calculation of temperature, from value of measured voltage
 * Param:	value:	Value of voltage, LSB -> 0.1 mV
 * Retval:	Temperature in *C
 */
float tempCalculate(uint16_t value)
{
	float retval = 0.0;
	for(unsigned int i = 1; i < 28; i++)
	{
		if(value >= (uint16_t)temperatureMap[i][0])
		{
			// approximation
			retval = (float)temperatureMap[i][1] - 5.0 * ((float)value-(float)temperatureMap[i][0]) / ((float)temperatureMap[i-1][0] - (float)temperatureMap[i][0]);
			break;
		}
	}
	return retval;
}

/**
 * Brief:	Getting proper values of temperatures on every cell in *C
 * Param:	None
 * Retval:	None
 */
void getTemp()
{
	tempValuesAvr = 0;
	for(unsigned int i = 0; i < NUMBER_OF_CELLS; i++)
	{
		tempValues[i] = (uint16_t)tempCalculate(adcValues[i]);
		tempValuesAvr += tempValues[i];
	}
	tempValuesAvr = tempValuesAvr / 6;
}
/**
 * Brief:	Muting discharge
 * Param:	None
 * Retval:	None
 */
void muteDis()
{
	uint8_t tab[4];
	uint16_t pec;


	uint16_t cmd = (1<<15) | 0b101000;
	memset(tab, 0, 4);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	LTC_wakeUp();

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 4, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);
}
/**
 * Brief:	Unmuting discharge
 * Param:	None
 * Retval:	None
 */
void unmuteDis()
{
	uint8_t tab[4];
	uint16_t pec;


	uint16_t cmd = (1<<15) | 0b101001;
	memset(tab, 0, 4);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	LTC_wakeUp();

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 4, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);
}

/**
 * Brief:	Send discharge configuration and start the discharge, BLOCKING MODE
 * Param:	cell: number of cell wanted to be discharged
 * Retval:	None
 */
void LTC_turnOnDischarge(int cell)
{
	uint8_t tab[12];
	uint16_t pec;

	unmuteDis();

	uint16_t cmd = (1<<15) | 0b10100;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;


	tab[4] = (cellDischarge[0]) | (cellDischarge[1] << 4); // 1, 2
	tab[5] = (cellDischarge[2]) | (cellDischarge[3] << 4); // 3, 4
	tab[6] = (cellDischarge[4]) | (cellDischarge[5] << 4); // 5, 6
	tab[7] = 0;
	tab[8] = 0;
	tab[9] = 0;
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	LTC_wakeUp();

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 12, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);

	cmd = (1<<15) | 0x01;
	memset(tab, 0, 12);
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
	if(cell<7){
		ltcConfig[4] = ltcConfig[4] | (1 << (cell));
	}
	tab[8] = ltcConfig[4];
	tab[9] = ltcConfig[5];
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;


	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 12, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);


}

/**
 * Brief:	Turn off discharge, BLOCKING MODE
 * Param:	None
 * Retval:	None
 */
void LTC_turnOffDischarge()
{

	uint8_t tab[100], rx_tab[100];
	uint16_t pec;


	uint16_t cmd = (1<<15) | 0b10100;
	memset(tab, 0, 12);
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;


	tab[4] = 0;
	tab[5] = 0;
	tab[6] = 0;
	tab[7] = 0;
	tab[8] = 0;
	tab[9] = 0;
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	LTC_wakeUp();

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 12, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);


	cmd = (1<<15) | 0x01;
	memset(tab, 0, 12);
	// configuration
	tab[0] = (cmd>>8);
	tab[1] = cmd;
	pec = pec15((char*)tab, 2);
	tab[2] = pec >> 8;
	tab[3] = pec;

	ltcConfig[4] = 0;

	tab[4] = ltcConfig[0];
	tab[5] = ltcConfig[1];
	tab[6] = ltcConfig[2];
	tab[7] = ltcConfig[3];
	tab[8] = ltcConfig[4];
	tab[9] = ltcConfig[5];
	pec = pec15((char*)&tab[4], 6);
	tab[10] = pec >> 8;
	tab[11] = pec;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, tab, 12, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, SET);

	muteDis();
}

/**
 * Brief:	CAN initialization
 * Param:	None
 * Retval:	None
 */
void canInit()
{
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
/**
 * Brief:	Control of cell discharges
 * Param:	None
 * Retval:	None
 */
void balanceControl()
{
	uint8_t charged_cells = 0;
	if(chargingOn == 1)
	{
		uint16_t lowestValue;
		lowestValue=cellValues[0];
		for(unsigned int i = 1; i < NUMBER_OF_CELLS; i++)
		{
			if(lowestValue > cellValues[i])
			{
				lowestValue = cellValues[i];
			}
		}
		if(dischargeActivation == 0)
		{
			for(int i = 0; i < NUMBER_OF_CELLS; i++)
			{
				if((cellValues[i] - lowestValue) > BALANCE_VALUE)
				{
					if(dischargeAtOnce < 2)
					{
						cellDischarge[i] = 1;
						dischargeAtOnce++;
						i++;
					}
					dischargeActivation = 1;
					dischargeTickEnd = HAL_GetTick()+20000;
				}
				else if(cellValues[i] > VOL_UP_OK)
				{
					charged_cells++;
					if(dischargeAtOnce < 2)
					{
						cellDischarge[i] = 1;
						dischargeAtOnce++;
						i++;
					}
					dischargeActivation = 1;
					dischargeTickEnd = HAL_GetTick()+20000;
				}
				else
					cellDischarge[i] = 0;

				if(cellValues[i] >= VOL_UP_OK) chargingOn = 2;
			}
		}
		if(charged_cells >= 4)
		{
			chargingOn = 2;
			chargingCurrent = abs(outputCurrent);
		}
	}
	else
	{
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, SET);
		dischargeActivation = 0;

		for(int i = 0; i < NUMBER_OF_CELLS; i++)
		{
			cellDischarge[i] = 0;
		}
	}

	if(chargingOn == 2 && abs(outputCurrent) < 0.1 * chargingCurrent)
	{
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, SET);
	}

	if(dischargeActivation == 0)
	{
		for(int i = 0; i < NUMBER_OF_CELLS; i++)
		{
			if(cellValues[i] > VOL_UP_OK)
			{
				charged_cells++;
				if(dischargeAtOnce < 2)
				{
					cellDischarge[i] = 1;
					dischargeAtOnce++;
					i++;
				}
				dischargeActivation = 1;
				dischargeTickEnd = HAL_GetTick()+20000;
			}
			else
				cellDischarge[i] = 0;
		}
	}


}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		can100HzFlag = 1;
	}

}
/**
 * Brief:	Calculations of output current
 * Param:	None
 * Retval:	None
 */
void calculateCurrent()
{
	float raw_sum = 0, raw_max = INT32_MIN, raw_min = INT32_MAX;

	for(int i = 0; i < 100; i++)
	{
		int16_t raw_value = currentSensor[i];
		raw_sum += raw_value;
		if(raw_value > raw_max) raw_max = raw_value;
		if(raw_value < raw_min) raw_min = raw_value;
	}
	currentSensorAvg = raw_sum / 100;

	currentSensorAvg -= NEUTRAL_CURRENT_SENSOR;
	raw_max -= NEUTRAL_CURRENT_SENSOR;
	raw_min -= NEUTRAL_CURRENT_SENSOR;

	outputCurrent = currentSensorAvg * COEFF_LSB_TO_AMP;
	outputCurrent_max = (float)raw_max * COEFF_LSB_TO_AMP;
	outputCurrent_min = (float)raw_min * COEFF_LSB_TO_AMP;

	if(outputCurrent_historic_max < outputCurrent_max)
		outputCurrent_historic_max = outputCurrent_max;

	if(outputCurrent < -1)
	{
		chargingOn = 1;
	}else{
		chargingOn = 0;
	}

}

/**
 * Brief:	Checking values and switching off safety
 * Param:	None
 * Retval:	None
 */
void checkError()
{
	uint16_t lowestValue;
	lowestValue=cellValues[0];
	for(int i = 1; i < NUMBER_OF_CELLS; i++)
	{
		if(lowestValue > cellValues[i])
		{
			lowestValue = cellValues[i];
		}
	}

	/*if(HAL_GPIO_ReadPin(LED_2_GPIO_Port, LED_2_Pin) == 1)
	{
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, SET);
	}*/
	for(int i = 0; i < NUMBER_OF_CELLS; i++)
	{
		//if (HAL_GPIO_ReadPin(LED_2_GPIO_Port, LED_2_Pin) != 1){
		if((cellValues[i] - lowestValue) > UNBALANCE_LIMIT)
		{
			errorCounterBalance[i]++;
		}else{
			errorCounterBalance[i]=0;
			acuState = 0;
		}
		if(cellValues[i] < VOL_DOWN_LIMIT)
		{
			errorCounterLowVol[i]++;
		}else{
			errorCounterLowVol[i]=0;
			acuState=0;
		}
		//}
		if(cellValues[i] > VOL_UP_LIMIT)
		{
			errorCounterHighVol[i]++;
		}else{
			errorCounterHighVol[i]=0;
			acuState=0;
		}
		if(tempValues[i] > TEMPERATURE_LIMIT)
		{
			errorCounterHighTemp[i]++;
		}else{
			errorCounterHighTemp[i]=0;
			acuState = 0;
		}
		/*if(cellValues[i] < 30000)
		{
			sleep mode
		}*/
	}

	if(outputCurrent <= NEUTRAL_CURRENT_CAR_POS && outputCurrent >= NEUTRAL_CURRENT_CAR_NEG)
	{
		sleepCounter++;
	}else{
		sleepCounter = 0;
		acuState = 0;
	}
	if(outputCurrent > TOO_HIGH_CURRENT)
	{
		errorCounterHighCur++;
	}else{
		errorCounterHighCur = 0;
		acuState = 0;
	}

	//EVERY COUNTER IS MULTIPLIED BY 50MS. IT IS A TIME!

	for(int i = 0; i < NUMBER_OF_CELLS; i++)
	{
		if(errorCounterLowVol[i] >= 9)
		{
			acuState = 0b1; //too low voltage
			if(errorCounterLowVol[i] == 10)
			{
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);

				HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
			}
		}
		if(errorCounterHighVol[i] >= 9)
		{
			acuState = 0b10; //too high voltage
			if(errorCounterHighVol[i] == 10)
			{
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
				//HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
			}
		}
		if(errorCounterHighTemp[i] >= 19)
		{
			acuState = 0b11; //too high temp
			if(errorCounterHighTemp[i] == 20)
			{
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);

				HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, RESET);
			}
		}
		if(errorCounterBalance[i] >= 9)
		{
			acuState = 0b100; //not balanced
			if(errorCounterBalance[i] == 10)
			{
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
				//HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, RESET);
			}
		}
		//if(cellValues[i] > 37000 && cellValues[i] <= 42200 && tempValues[i] < 40 && sleepCounter < COUNTER_TO_SLEEP)
		//{
		//	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, SET);
		//}
	}


	if(errorCounterHighCur >= 9)
	{
		acuState = 0b101; //too high current
		if(errorCounterHighCur == 10)
		{
			HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);

			HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, RESET);
			HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
		}
	}

	if(sleepCounter == COUNTER_TO_SLEEP){
		acuState = 0b110; //Relay permanently switched off- need to reset
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, RESET);
	}


}
/**
 * Brief:	First CAN message
 * Param:	None
 * Retval:	None
 */
void canData_1()
{
	CAN_TxHeaderTypeDef tx_header;
	tx_header.DLC = 8;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = 0x0C;
	uint8_t data[8];

	for(int i = 0; i < NUMBER_OF_CELLS; i++)
	{
		cellValuesCAN[i] = cellValues[i] / 1000;
	}

	/*
	 * acuState:
	 * 0- all good
	 * 1- too low voltage
	 * 2- too high voltage
	 * 3- too high temperature
	 * 4- cells are not balanced (difference from lowest to any >0.2V)
	 * 5- too high current
	 * 6- relay permanently switched off- need to reset
	 */

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
/**
 * Brief:	Second CAN message
 * Param:	None
 * Retval:	None
 */
void canData_2()
{
	CAN_TxHeaderTypeDef tx_header;
	tx_header.DLC = 8;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = 0x5F;
	uint8_t data[7];

	tempValuesCAN[0] = tempValues[5];
	tempValuesCAN[1] = tempValues[2];
	tempValuesCAN[2] = tempValues[4];
	tempValuesCAN[3] = tempValues[1];
	tempValuesCAN[4] = tempValues[3];
	tempValuesCAN[5] = tempValues[0];
	//mixed because of thermistor placement

	data[0] = tempValuesAvr;
	data[1] = tempValuesCAN[0];
	data[2] = tempValuesCAN[1];
	data[3] = tempValuesCAN[2];
	data[4] = tempValuesCAN[3];
	data[5] = tempValuesCAN[4];
	data[6] = tempValuesCAN[5];

	uint32_t mailbox = 0;
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}

/**
 * Brief:	Activation of balance
 * Param:	None
 * Retval:	None
 */
void balanceActivation()
{
	if(dischargeActivation == 1)
	{
		for(int i = 0; i < NUMBER_OF_CELLS ; i++)
		{
			if(cellDischarge[i] == 1)
			{
				LTC_turnOnDischarge(i);
			}
		}

	}
	else
	{
		LTC_turnOffDischarge();
	}
}
/**
 * Brief:	Serial print through USB- all voltages, temperatures and output current (min, max, historical max and active)
 * Param:	None
 * Retval:	None
 */
void serialPrint()
{
	static char tab[3000];
	uint16_t n=0;

	RTC_DateTypeDef rtc_date;
	RTC_TimeTypeDef rtc_time;
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

	float cellValuesSumF = (float)cellValuesSum / 10000;
	n += sprintf(&tab[n], "%02d:%02d:%02d\r\n", rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds);
	n += sprintf(&tab[n], "*** Battery state: %d ***", acuState);
	n += sprintf(&tab[n], "\n");
	n += sprintf(&tab[n], "LEGEND FOR BATTERY STATE:");
	n += sprintf(&tab[n], "\n");
	n += sprintf(&tab[n], "0- all good");
	n += sprintf(&tab[n], "\n");
	n += sprintf(&tab[n], "1- too low voltage");
	n += sprintf(&tab[n], "\n");
	n += sprintf(&tab[n], "2- too high voltage");
	n += sprintf(&tab[n], "\n");
	n += sprintf(&tab[n], "3- too high temperature");
	n += sprintf(&tab[n], "\n");
	n += sprintf(&tab[n], "4- cells are not balanced (difference from lowest to any >0.2V)");
	n += sprintf(&tab[n], "\n");
	n += sprintf(&tab[n], "5- too high current");
	n += sprintf(&tab[n], "\n");
	n += sprintf(&tab[n], "6- relay permanently switched off- need to reset");
	n += sprintf(&tab[n], "\r\n\n");
	n += sprintf(&tab[n], "*** Stack voltage:\t%3.2f V ***", cellValuesSumF);
	n += sprintf(&tab[n], "\r\n");

	for(int i = 0; i < NUMBER_OF_CELLS; i++)
	{
		float cellValue = (float)cellValues[i] / 10000;
		n += sprintf(&tab[n], "-V.%d-\t", i+1);
		n += sprintf(&tab[n], "%1.2f%c\t", cellValue, cellDischarge[i] == 0 ? ' ' : '*');
		n += sprintf(&tab[n], " -T.%d-\t", i+1);
		n += sprintf(&tab[n], "%d\t", tempValuesCAN[i]);
		n += sprintf(&tab[n], "\r\n");
	}

	n += sprintf(&tab[n], "\r\n");
	n += sprintf(&tab[n], "Output current:\t%3.2f\r\n", outputCurrent);
	n += sprintf(&tab[n], "Max current:\t%3.2f\r\n", outputCurrent_max);
	n += sprintf(&tab[n], "Min current:\t%3.2f\r\n", outputCurrent_min);
	n += sprintf(&tab[n], "Historic max:\t%3.2f\r\n", outputCurrent_historic_max);
	n += sprintf(&tab[n], "\r\n");

	n += sprintf(&tab[n], "Relay state:\t%d\r\n", HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin));
	n += sprintf(&tab[n], "\r\n");

	CDC_Transmit_FS(tab, n);
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
	MX_TIM3_Init();
	MX_TIM6_Init();
	MX_ADC2_Init();
	MX_TIM8_Init();
	MX_RTC_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(2000);

	init_PEC15_Table();

	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, SET);

	canInit();

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start(&htim8);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, NUMBER_OF_CELLS);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)currentSensor, 100);


	dataRefreshNextTick = HAL_GetTick();
	uint8_t chargingLED = 0;
	uint8_t skip = 0;

	while (1)
	{
		if(dataRefreshNextTick <= HAL_GetTick())
		{
			dataRefreshNextTick += 50; //50ms loop


			// discharge off while adc
			//LTC_turnOffDischarge();

			HAL_Delay(1);
			LTC_startCellAdc();
			HAL_Delay(30);

			// on discharge if needed
			//if(dischargeActivation == 1)
			//	  {
			//		  LTC_turnOnDischarge();
			//	  }

			LTC_getValuesAdc();

			getTemp();

			chargingLED++;

			newData = 1;

			if(dischargeTickEnd < HAL_GetTick())
			{
				dischargeActivation = 0;
				dischargeTickEnd = 0;
				dischargeAtOnce = 0;
				for(int i = 0; i < NUMBER_OF_CELLS; i++)
				{
					cellDischarge[i] = 0;
				}
				skip++;
			}
		}

		if(newData == 1)
		{
			newData = 0;

			calculateCurrent();

			// accumulator status
			checkError();

			if(skip == 10)
			{
				balanceControl();
				skip = 0;
			}else if(dischargeTickEnd > HAL_GetTick()){
				balanceControl();
			}

			if(chargingOn == 1 && chargingLED == 10)
			{
				HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
				chargingLED = 0;
			}


			// balance activation
			balanceActivation();

			//Serial Print through USB
			serialPrint();

		}
		if(can100HzFlag == 1)
		{
			canData_1();
			canData_2();
			can100HzFlag = 0;
		}
		//LTC_turnOnDischarge(1); //balance test

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB
			|RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
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
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_16;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

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
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	hrtc.Init.BinMode = RTC_BINARY_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

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
	htim3.Init.Period = 49;
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
	htim6.Init.Prescaler = 2399;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 9;
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
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 95;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 49;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA2_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
	HAL_GPIO_WritePin(GPIOB, LED_2_Pin|LED_3_Pin|LED_4_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_DASH_GPIO_Port, LED_DASH_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : RELAY_Pin */
	GPIO_InitStruct.Pin = RELAY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_2_Pin LED_3_Pin LED_4_Pin LED_DASH_Pin */
	GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin|LED_4_Pin|LED_DASH_Pin;
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
