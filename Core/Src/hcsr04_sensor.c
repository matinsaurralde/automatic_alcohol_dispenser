/**
*@file: hcsr04_sensor.c
*@brief: HC-SR04 ultrasonic distance sensor stm32 hal driver
*@author: Veysel G�kdemir, � 2020
*@note: This library is compatible with HAL Libs which contain GPIO and HAL_TIM_IT(HAL timer interrupt) features as to:
        HAL_GPIO_WritePin(...),  HAL_TIM_Base_Start_IT(...), Gated Mode, HAL_TIM_PeriodElapsedCallback(...),
        __HAL_TIM_GET_FLAG(...), __HAL_TIM_CLEAR_FLAG(...),  __HAL_TIM_SetCounter(...), __HAL_TIM_GetCounter(...).
       		
* The library has been tested on STM32F407 Board.
*//*


*/
/*Includes---------------------------------------------------------*/
#define ECHO_Pin GPIO_PIN_1
#define ECHO_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_2
#define TRIG_GPIO_Port GPIOA

#include "hcsr04_sensor.h"
#include "hcsr04_userConf.h"



/*-----------------------------------------------------------------*//*


*/
/*Defines and variables--------------------------------------------*//*



*/
/*-----------------------------------------------------------------*//*


*/
/*Private functions------------------------------------------------*//*

*/
/**
*@brief: The sensor initiliazing settings.
*@retval: None
*/

void HCSR04_Init(void)
{
	user_define();
	
	HAL_GPIO_WritePin(GPIO_Trigger, GPIO_PIN_Trigger, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_Echo, GPIO_PIN_Echo, GPIO_PIN_RESET);
	 
  HAL_TIM_Base_Start_IT(&htim_echo);
	HAL_TIM_Base_Start_IT(&htim_delay);
}


/**
*@brief: HC-SR04 HAL_TIM_PeriodElapsedCallback
*@param: htim pointer to a TIM_HandleTypeDef structure that contains the configuration information for TIM module.
*@retval: None
*/


void HCSR04_TIM_PEC(TIM_HandleTypeDef *htim)
{
	user_define();
	
	if(htim->Instance == TIM_Delay)
		__HAL_TIM_CLEAR_FLAG(&htim_delay, TIM_FLAG_UPDATE);            //Clear TIM_Delay UPDATE event Flag.
	
	else if(htim->Instance == TIM_Echo)
		{
			HAL_GPIO_WritePin(GPIO_Echo, GPIO_PIN_Echo, GPIO_PIN_RESET); //Reset Echo led (default:blue)  
	    __HAL_TIM_CLEAR_FLAG(&htim_echo, TIM_FLAG_UPDATE);           //Clear TIM_Echo UPDATE event Flag.
	  }
}/*


*/
/**
*@brief: The sensor data read function
*@param: HCSR04_DATA pointer to hcsr04_data_t structure that contains data for the sensor.
         HCSR04_DATA -> duration: ms, us, distance: mm, cm, m, inch.   
*@retval: None
*/

void delay (uint16_t delay)// RECUERDA QUE CADA 1 ES 10 NANOSEGUNDOS
{							// SI QUIERES 1 us se necesita 100
    delay=delay*100; // ahora es un delay de micro segundos
    __HAL_TIM_SET_COUNTER(&htim1,0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < delay);

}

void HCSR04_GetInfo(hcsr04_data_t *HCSR04_DATA)
{
	static volatile float duration = 0, distance = 0;
	
	user_define();
	
	  //Default: Timer1 Parameter Settings;
	  //APB2 timer clocks (MHz): 168, Prescaler:167, AutoReload Register:23323
		//168 MHz / (167+1) = 1 MHz equals 1 us, Txf=1
		//So if we get ARR(counter period) value, the time(echo signal duration) equals (ARR+1) x 1us (use directly counter_TIM1 value in microseconds)
		//The sensor distance range: about 2cm-400cm, distance_cm = (counter_TIM1/2)*0.0343
		//If we get max. distance_cm=400, the counter_TIM1=(400*2)/0.0343=23323.6 , thus we can get max. ARR > 23323.6 as 23324 (ARR+1).
		//The sensor trigger pulse width= 10us 
	  
		//Blinking blue led shows that the sensor is seeking an object or has not dedected it exactly (it doesn't send the echo signal well).
		//if blue led is on and not blinking, it shows that the sensor has found an object (it sends the echo signal well).
	  //Blue led also indicates that TIM_Echo(default:TIM1) works and counts.		
		HAL_GPIO_WritePin(GPIO_Echo, GPIO_PIN_Echo, GPIO_PIN_SET);        //Set blue led. 
		HAL_GPIO_WritePin(GPIO_Trigger, GPIO_PIN_Trigger, GPIO_PIN_SET);  //Set trigger pin, orange led.
		
	  //Default: Timer2 Parameter Settings;
	  //10us delay is enough and the sensor sends 8 pulses at 40kHz.
	  //APB1 timer clocks (MHz): 84, Prescaler:83, AutoReload Register(ARR):9
	  //84 MHz / (83+1) = 1 MHz equals 1 us, (ARR+1) x 1us equals 10us delay.

/*----10us delay by default:TIM2-----------------------------*/

    __HAL_TIM_SetCounter(&htim_delay, 0);
	  while(__HAL_TIM_GET_FLAG(&htim_delay, TIM_FLAG_UPDATE) != SET)
	  {	
			
	  }

/*-----------------------------------------------------------*/


    HAL_GPIO_WritePin(GPIO_Trigger, GPIO_PIN_Trigger, GPIO_PIN_RESET); //Reset trigger pin, orange led.
    
	
		while(__HAL_TIM_GET_FLAG(&htim_echo,TIM_FLAG_TRIGGER) != RESET && __HAL_TIM_GET_FLAG(&htim_echo, TIM_FLAG_UPDATE) != SET)
		{	
			
			duration = __HAL_TIM_GetCounter(&htim_echo);        //Get TIM_Echo(default:TIM1) counter value(default:us) as long as there is Echo signal(rising edge).					
			__HAL_TIM_SetCounter(&htim_echo, 0);                //Set the counter 0 , reset it.	
			__HAL_TIM_CLEAR_FLAG(&htim_echo, TIM_FLAG_TRIGGER); //Clear TIM_Echo Trigger Flag.
		
		}
		
				
		//X=V*t, X=distance_cm, V=0.0343cm/us, t=(transmission + reception time)/2 us
		distance = (duration/2)*(float)0.0343; //sound speed = 343m/s = 0.0343cm/us
		distance -= zero_point;                //subtract the zero point and then, the distance(default:cm) equals from the zero point to object.
	
	  HCSR04_DATA->distance_cm = distance;
		HCSR04_DATA->distance_mm = distance*10;
		HCSR04_DATA->distance_m = distance/100;
		HCSR04_DATA->distance_inch = distance/(float)2.54;
		
		HCSR04_DATA->duration_us = duration;
		HCSR04_DATA->duration_ms = duration/1000;
}
uint32_t local_time = 0;
uint32_t hcsr04_read () // PARA LEER SE HACE LO QUE MENCIONA LA HOJA DE DATOS DEL HC-SR04
{
    local_time=0;
    HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_RESET);  // PRIMERO SE PONE EN CERO AL TRIG
    //BSP_Buzz_off(Trig);
    delay(2);  // SE ESPERA 20 uS


    HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_SET);  // SE PONE EN UNO AL TRIG
    // BSP_Buzz_on(Trig);
    delay(10);  // SE ESPERA 100 uS
    HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_RESET);  // SE PONE EN CERO AL TRIG
    // BSP_Buzz_off(Trig);

    // SE LEE EL TIEMPO POR EL CUAL EL PIN ES ALTO

    while (!(HAL_GPIO_ReadPin(GPIOA, ECHO_Pin)));  // SE ESPERA QUE ECHO LEA
    while (HAL_GPIO_ReadPin(GPIOA, ECHO_Pin))    // MIENTRAS ECHO ES UNO
    {
        local_time++;   // MEDIDA DEL TIEMPO DURANTE FUE UNO EL ECHO
        delay (1);
    }
    return local_time*2;
}



/******************************************************END OF FILE*********************************************************************/

