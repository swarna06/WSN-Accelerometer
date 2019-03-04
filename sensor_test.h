/*
 * sensor_test.h
 *
 *  Created on: Aug 20, 2018
 *      Author: alvaro
 */

#ifndef SENSOR_TEST_H_
#define SENSOR_TEST_H_

#define SEN_SPI_CS_PIN      BRD_SEN_CS

void Sen_Init();

void Sen_Read_Acc_Test(int16_t* abuf);



#endif /* SENSOR_TEST_H_ */
