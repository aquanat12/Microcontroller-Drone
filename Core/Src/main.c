/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

// NUR SARAH BINTE ASMADI A0254664X
// VAN DER HORST NIGEL SEBESTIAAN A0253455A


#include <main.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32l475xx.h>
#include <stm32l4xx_hal_cortex.h>
#include <stm32l4xx_hal_def.h>
#include <stm32l4xx_hal_gpio.h>
#include <stm32l4xx_hal_gpio_ex.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_hal_uart.h>
#include <stm32l4xx_hal_uart_ex.h>
#include <string.h>
#include <sys/_stdint.h>

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/Components/ssd1306/ssd1306.h"


#define Buzzer_Pin GPIO_PIN_1
#define Buzzer_GPIO_Port GPIOB

UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

static void UART1_Init(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void Buzzer_Init(void);
static float gyro(void);
static float humid(void);
static float accel(void);
static float temp(void);
static float pres(void);
static float mag(void);
static int Last_of_EE2028(void);
static void Standby_Mode(void);
static void Battle_Mode(void);
// extern void initialise_monitor_handles(void);

//bitmap for final picture
const unsigned char zombie[] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x79, 0x78, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3c, 0x3c, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0e, 0x3c, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0x8e, 0x1c, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x02, 0x18, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9e, 0x80, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1e, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x08, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x5f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xfe, 0x0c, 0x1f, 0xff, 0xc0, 0x00, 0x7f, 0xc0, 0x60, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xfc, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff,
		0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xfe, 0x01, 0xf0, 0x7f, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};


int lasergun_energy = 0;
int mode = 0; // mode 0 = standby, mode 1 = battle, mode 2 = last_of_ee2028
int press = 0; // 0 = no press, 1 = single press, 2 = double press
int end = 0;// End the program
char OLed_Val; // Return value for OLEDdisplay
int counter = 0; //Last of 2028 counter
uint32_t T3 = 0;
uint32_t dp = 500;



const float T_Max = 40.00; //Too hot for humans outside 40
const float P_Min = 1000.00; //Too high off the ground or storm is approaching 1000
const float H_Min = 50.00; //Not enough humidity for humans 50
const float A_Min = -6.00; //Board is upside down -6
const float G_Max = 5.00; //Crash incoming! 5
const float M_Max = 500.00;//Magnetic field indicates a chance of bomb! 500

int T_Max_int = 40;
int T_Max_frac = 00;
int P_Min_int = 1000;
int P_Min_frac = 00;
int H_Min_int = 50;
int H_Min_frac = 00;
int A_Min_int = -6;
int A_Min_frac = 00;
int G_Max_int = 5;
int G_Max_frac = 00;
int M_Max_int = 500;
int M_Max_frac = 00;



void LED_DISPLAY(int mode){
	ssd1306_Fill(Black);
	ssd1306_SetCursor(5,5);
	if (mode == 0){
		char message[] = "Standby Mode";
		OLed_Val = ssd1306_WriteString(message, Font_7x10, White);
		ssd1306_SetCursor(5,25);
		char message2[15];
		sprintf(message2, "Lasergun: %d/10", lasergun_energy);
		OLed_Val = ssd1306_WriteString(message2, Font_11x18, White);
	}
	else if (mode == 1){
		char message[] = "Battle Mode";
		OLed_Val = ssd1306_WriteString(message, Font_7x10, White);
		ssd1306_SetCursor(5,25);
		char message2[20];
		sprintf(message2, "Lasergun: %d/10", lasergun_energy);
		OLed_Val = ssd1306_WriteString(message2, Font_11x18, White);
	}
	else if (mode == 2){
		char message[] = "Last_of_EE2028";
		OLed_Val = ssd1306_WriteString(message, Font_7x10, White);
	}
	else if (mode == 4){
		char message[] = "Battle Mode";
		OLed_Val = ssd1306_WriteString(message, Font_7x10, White);
		ssd1306_SetCursor(5,25);
		char message2[20];
		sprintf(message2, "Lasergun: %d/10", lasergun_energy);
		OLed_Val = ssd1306_WriteString(message2, Font_11x18, White);
		ssd1306_SetCursor(20,45);
		char message3[] = "PEW PEW";
		OLed_Val = ssd1306_WriteString(message3, Font_11x18, White);
	}
	else if (mode == 5){
		char message[] = "Last_of_EE2028";
		OLed_Val = ssd1306_WriteString(message, Font_7x10, White);
		ssd1306_SetCursor(20,25);
		char message2[15];
		int i = 11 - (counter / 2);
		sprintf(message2, "%d", (i));
		OLed_Val = ssd1306_WriteString(message2, Font_16x26, White);
	}
	ssd1306_UpdateScreen();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t T4 = HAL_GetTick();
	if ((press > 0) && ((T4 - T3) < dp)){
		//char message_print[15];
		//sprintf(message_print, "double\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		press = 2;
	}
	else{
		//char message_print[15];
		//sprintf(message_print, "single\r\n");
		//HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		press = 1;
		T3 = HAL_GetTick();
	}
}



int main(void)
{
	//initialise_monitor_handles();
	HAL_Init();
	MX_GPIO_Init();
	MX_I2C1_Init();
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	UART1_Init();
	ssd1306_Init();
	Buzzer_Init();

	while (end == 0)
	{
		if (mode == 0){
			Standby_Mode();}
		else if (mode == 1){
			Battle_Mode();}
		else{
			Last_of_EE2028();}
	}
	exit(0);

}




void Telemetry_Standby(void){
	/*
					  float G = gyro();
					  float M = mag();
					  float P = pres();
					  float H = humid();
					  int max = 0;
					  //int G_1 = (int)G;
					  //int G_2 = (int)((G - G_1)*100) ;
					  if (G > G_Max){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "G:%.2f(°/s), exceeds threshold of %.2f(°/s).\r\n", G, G_Max);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (M > M_Max){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "M:%.2f(°/s), exceeds threshold of %.2f(°/s).\r\n", M, M_Max);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (P < P_Min){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "P:%.2f(°/s), exceeds threshold of %.2f(°/s).\r\n", P, P_Min);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (H > H_Max){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "H:%.2f(°/s), exceeds threshold of %.2f(°/s)\r\n", H, H_Max);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (max == 0){
						  char message_print[70];
						  sprintf(message_print, "G:%.2f(°/s), M:%.2f(M), P:%.2f(P), H:%.2f(g/kg)\r\n", G, M, P, H);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
	 */

	float G = gyro();
	int G_int = (int)G;
	int G_frac = abs((int)((G - G_int) * 100));
	float M = mag();
	int M_int = (int)M;
	int M_frac = abs((int)((M - M_int) * 100));
	float P = pres();
	int P_int = (int)P;
	int P_frac = abs((int)((P - P_int) * 100));
	float H = humid();
	int H_int = (int)H;
	int H_frac = abs((int)((H - H_int) * 100));
	int max = 0;

	if (G > G_Max){
		max = 1;
		char message_print[60];
		sprintf(message_print, "G:%d.%02d(rad/s), exceeds threshold of %d.%02d(rad/s).\r\n", G_int, G_frac, G_Max_int, G_Max_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (M > M_Max){
		max = 1;
		char message_print[60];
		sprintf(message_print, "M:%d.%02d(uT), exceeds threshold of %d.%02d(uT).\r\n", M_int, M_frac, M_Max_int, M_Max_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (P < P_Min){
		max = 1;
		char message_print[60];
		sprintf(message_print, "P:%d.%02d(mb), exceeds threshold of %d.%02d(mb).\r\n", P_int, P_frac, P_Min_int, P_Min_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (H < H_Min){
		max = 1;
		char message_print[60];
		sprintf(message_print, "H:%d.%02d(%%rH), exceeds threshold of %d.%02d(%%rH)\r\n", H_int, H_frac, H_Min_int, H_Min_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (max == 0){
		char message_print[100];
		sprintf(message_print, "G:%d.%02d(rad/s), M:%d.%02d(uT), P:%d.%02d(mb), H:%d.%02d(%%rH)\r\n", G_int, G_frac, M_int, M_frac, P_int, P_frac, H_int, H_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
}


void Telemetry_Battle(void){

	/*
					  float T = temp();
					  float P = pres();
					  float H = humid();
					  float A = accel();
					  float G = gyro();
					  float M = mag();
					  int max = 0;
					  if (T > T_Max){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "G:%.2f(°C), exceeds threshold of %.2f(°C)\r\n", T, T_Max);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (P < P_Min){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "G:%.2f(°/s), exceeds threshold of %.2f(°/s)\r\n", P, P_Min);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (H > H_Max){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "G:%.2f(°/s), exceeds threshold of %.2f(°/s)\r\n", H, H_Max);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (A > A_Max){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "G:%.2f(°/s), exceeds threshold of %.2f(°/s)\r\n", A, A_Max);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (G > G_Max){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "G:%.2f(°/s), exceeds threshold of %.2f(°/s)\r\n", G, G_Max);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (M > M_Max){
						  max = 1;
						  char message_print[60];
						  sprintf(message_print, "G:%.2f(°/s), exceeds threshold of %.2f(°/s)\r\n", M, M_Max);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
					  if (max == 0){
						  char message_print[100];
						  sprintf(message_print, "T:%.2f(°C), P:%.2f(P), H:%.2f(g/kg), A:%.2f(n/M), G:%.2f(°/s), M:%.2f(M)\r\n", T, P, H, A, G, M);
						  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					  }
	 */
	float G = gyro();
	int G_int = (int)G;
	int G_frac = abs((int)((G - G_int) * 100));
	float M = mag();
	int M_int = (int)M;
	int M_frac = abs((int)((M - M_int) * 100));
	float P = pres();
	int P_int = (int)P;
	int P_frac = abs((int)((P - P_int) * 100));
	float H = humid();
	int H_int = (int)H;
	int H_frac = abs((int)((H - H_int) * 100));
	float T = temp();
	int T_int = (int)T;
	int T_frac = abs((int)((T- T_int) * 100));
	float A = accel();
	int A_int = (int)A;
	int A_frac = abs((int)((A- A_int) * 100));
	int max = 0;
	if (T > T_Max){
		max = 1;
		char message_print[60];
		sprintf(message_print, "T:%d.%02d(C), exceeds threshold of %d.%02d(C)\r\n", T_int, T_frac, T_Max_int, T_Max_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (A < A_Min){
		max = 1;
		char message_print[60];
		sprintf(message_print, "A:%d.%02d(m/s^2), exceeds threshold of %d.%02d(m/s^2)\r\n", A_int, A_frac, A_Min_int, A_Min_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (G > G_Max){
		max = 1;
		char message_print[60];
		sprintf(message_print, "G:%d.%02d(rad/s), exceeds threshold of %d.%02d(rad/s).\r\n", G_int, G_frac, G_Max_int, G_Max_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (M > M_Max){
		max = 1;
		char message_print[60];
		sprintf(message_print, "M:%d.%02d(uT), exceeds threshold of %d.%02d(uT).\r\n", M_int, M_frac, M_Max_int, M_Max_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (P < P_Min){
		max = 1;
		char message_print[60];
		sprintf(message_print, "P:%d.%02d(mb), exceeds threshold of %d.%02d(mb).\r\n", P_int, P_frac, P_Min_int, P_Min_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (H < H_Min){
		max = 1;
		char message_print[60];
		sprintf(message_print, "H:%d.%02d(%%rH), exceeds threshold of %d.%02d(%%rH)\r\n", H_int, H_frac, H_Min_int, H_Min_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
	if (max == 0){
		char message_print[130];
		sprintf(message_print, "T:%d.%02d(C), A:%d.%02d(m/s^2), G:%d.%02d(rad/s), M:%d.%02d(uT), P:%d.%02d(mb), H:%d.%02d(%%rH)\r\n", T_int, T_frac, A_int, A_frac, G_int, G_frac, M_int, M_frac, P_int, P_frac, H_int, H_frac);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
}



int Activate_EE2028(void){
	float A = accel();
	if (A < A_Min){
		return 1;
	}
	return 0;
}



void Lasergun(void){
	lasergun_energy += 3;
	LED_DISPLAY(mode);
	char message_print[40];
	sprintf(message_print, "Lasergun charging %d/10\r\n", lasergun_energy);
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	if (lasergun_energy >= 5){
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_SET);
		char message_print[25];
		sprintf(message_print, "Lasergun goes PEW PEW\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		LED_DISPLAY(4);
		lasergun_energy -= 5;
		LED_DISPLAY(mode);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_RESET);
	}

}

static void Standby_Mode(void)
{
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_RESET);
	mode = 0;
	HAL_GPIO_WritePin(GPIOB, LED2_Pin,GPIO_PIN_SET);
	uint32_t wait = 1000;
	uint32_t T1 = HAL_GetTick();
	char message_print[30];
	sprintf(message_print, "Initiating Standby Mode\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	LED_DISPLAY(mode);
	while (1)
	{
		uint32_t T2 = HAL_GetTick();
		while ((T2 - T1) > wait){
			T1 = HAL_GetTick();
			Telemetry_Standby();
		}
		if (press == 2){
			mode = 1;
			press = 0;
			return;
		}
	}

}

static void Battle_Mode(void)
{
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_RESET);
	mode = 1;
	uint32_t wait = 1000;
	uint32_t T1 = HAL_GetTick();
	char message_print[30];
	sprintf(message_print, "Initiating Battle Mode\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	LED_DISPLAY(mode);
	while (1)
	{
		uint32_t T2 = HAL_GetTick();
		while ((T2 - T1) > wait){
			HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
			T1 = HAL_GetTick();
			Telemetry_Battle();
		}
		if (press > 0 && (press %2 == 0)){
			mode = 0;
			press = 0;
			return;
		}
		else if (press == 1){
			Lasergun();
			press += 2;
		}
		if (Activate_EE2028()){
			mode = 2;
			return;
		}
	}
}

void Command_Center(void){
	char message_print[25];
	sprintf(message_print, "Drone Was Attacked! \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
}

static int Last_of_EE2028(void)
{
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_RESET);
	counter = 0;
	mode = 2;
	HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
	uint32_t wait = 500;
	uint32_t T1 = HAL_GetTick();
	char message_print[30];
	sprintf(message_print, "Initiating Last_of_EE2028\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	LED_DISPLAY(mode);
	while (counter <= 20)
	{
		uint32_t T2 = HAL_GetTick();
		while ((T2 - T1) > wait){
			HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
			T1 = HAL_GetTick();
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_SET);
			counter += 1;
			if(counter % 2 == 0){
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_RESET);
				Command_Center();
				LED_DISPLAY(5);
			}
		}
		if (press == 2){
			mode = 1;
			press = 0;
			return end = 0;
		}
	}
	char final_message[60];
	sprintf(final_message, "When there is no more room in hell, the dead will walk\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)final_message, strlen(final_message),0xFFFF);
	ssd1306_Fill(Black);
	ssd1306_DrawBitmap(0, 0, zombie, 128, 64, White);
	ssd1306_UpdateScreen();
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_RESET);
	return end = 1;
}


static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();


	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin LED2_Pin */
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


}

static void Buzzer_Init(void)
{
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,GPIO_PIN_RESET);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Configuration of Buzzer (GPIO-B Pin-1) as Arduino Pin D6
	GPIO_InitStruct.Pin = Buzzer_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	__GPIOB_CLK_ENABLE();
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00000E14;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	__I2C1_CLK_ENABLE();
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}

}

static float accel(void){
	// sensitivty = mg/LSB, convert to m/s^2
	float accel_data[3];
	int16_t accel_data_i16[3] = { 0 };
	BSP_ACCELERO_AccGetXYZ(accel_data_i16);
	//accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
	//accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
	accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
	return accel_data[2];
}

static float temp(void){
	// sensitivity = °C/LSB
	// unit = degree celsius
	return BSP_TSENSOR_ReadTemp();	// read temperature sensor
}


static float gyro(void){
	//X = roll, Y = Pitch, Z = yaw, sensitivity is mdeg/LSB, convert to rad/s
	//unit = rad/s
	float gyro_data[3] = { 0 };
	BSP_GYRO_GetXYZ(gyro_data);
	gyro_data[0] = (float)gyro_data[0] * (1/1000.0f)* (M_PI/180);
	gyro_data[1] = (float)gyro_data[1] * (1/1000.0f)* (M_PI/180);
	gyro_data[2] = (float)gyro_data[2] * (1/1000.0f)* (M_PI/180);
	return sqrt(pow(gyro_data[0],2) + pow(gyro_data[1],2) + pow(gyro_data[2],2)); // return sqrt(x^2+y^2+z^2) for total magnitude
}

static float mag(void){
	// Sensitivity = mgauss/LSB
	// unit = uT, 1 gauss = 0.1 mT
	float mag_data[3];
	int16_t mag_data_i16[3] = { 0 };
	BSP_MAGNETO_GetXYZ(mag_data_i16);
	mag_data[0] = (float)mag_data_i16[0] * (1/10.0f);
	mag_data[1] = (float)mag_data_i16[1] * (1/10.0f);
	mag_data[2] = (float)mag_data_i16[2] * (1/10.0f);
	return sqrt(pow(mag_data[0],2) + pow(mag_data[1],2) + pow(mag_data[2],2)); // return sqrt(x^2+y^2+z^2) for magnitude
}

static float pres(void){
	// Sensitivity = LSB/hPa
	//unit = mb, 1hpa = 1mb, 260 - 1260hPa, max is 1000hPa
	return BSP_PSENSOR_ReadPressure();
}

static float humid(void){
	// sensitivity = %rH/LSB
	//unit = rH, 20 to 80%rH, min is 50%rH
	return BSP_HSENSOR_ReadHumidity();
}

static void UART1_Init(void)
{
	/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while(1);
	}

}


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
