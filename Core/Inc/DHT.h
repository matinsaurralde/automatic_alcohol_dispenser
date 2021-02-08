/*
 * DHT.h
 *
 *  Created on: Jun 28, 2020
 *      Author: Controllerstech.com
 */

#ifndef DHT_H_
#define DHT_H_



typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypeDef;


void DHT_GetData (DHT_DataTypeDef *DHT_Data);

#endif /* INC_DHT_H_ */
