/**
*@file: hcsr04_sensor.h
*@brief: HC-SR04 ultrasonic distance sensor stm32 hal driver
*@author: Veysel G�kdemir, � 2020
*/

#ifndef HCSR04_SENSOR_H
#define HCSR04_SENSOR_H 

#ifdef __cplusplus
extern "C" {
#endif

/*Includes-------------------------------------------------------*/
#include "stm32f4xx_hal.h"
	
/*---------------------------------------------------------------*/	
	
/*Defines and variables------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

/** 
  * @brief  The sensor data structures definition  
  */
typedef struct hcsr04_data {
	
	float duration_ms;
	float duration_us;
	
	float distance_mm;
	float distance_cm;
	float distance_m;
	float distance_inch;
	
} hcsr04_data_t;

/*---------------------------------------------------------------*/	

/*Private functions----------------------------------------------*/
/**
*@brief: The sensor initiliazing settings.
*@retval: None
*/
void HCSR04_Init(void);

/**
*@brief: HC-SR04 HAL_TIM_PeriodElapsedCallback
*@param: htim pointer to a TIM_HandleTypeDef structure that contains the configurations information for TIM module.
*@retval: None
*/
void HCSR04_TIM_PEC(TIM_HandleTypeDef *htim);

/**
*@brief: The sensor data read function
*@param: HCSR04_DATA pointer to hcsr04_data_t structure that contains data for the sensor.
         HCSR04_DATA -> duration: ms, us, distance: mm, cm, m, inch.
*@retval: None
*/
void HCSR04_GetInfo(hcsr04_data_t *HCSR04_DATA);
uint32_t hcsr04_read ();
void delay (uint16_t delay);

#ifdef __cplusplus
}
#endif

#endif


/******************************************************END OF FILE*********************************************************************/
