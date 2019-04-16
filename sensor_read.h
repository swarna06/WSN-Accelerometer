/*
 * sensor_test.h
 *
 *  Created on: Aug 20, 2018
 *      Author: alvaro
 */

#ifndef SENSOR_READ_H_
#define SENSOR_READ_H_

#define SEN_SPI_CS_PIN      BRD_SEN_CS

void Sen_Init();

void Sen_Read_Acc(int32_t* abuf);



#endif /* SENSOR_READ_H_ */
