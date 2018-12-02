/*
 * device_config.h
 *
 *  Created on: Nov 27, 2018
 *      Author: alvaro
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

// Network parameters
#define CFG_DEFAULT_DEV_ID          0
#define CFG_SENSOR_NODE_NUM         3

// Device (CC2640R2F) configuration (see ccfg.h)
#ifndef SET_CCFG_IEEE_BLE_0
#define SET_CCFG_IEEE_BLE_0         CFG_DEFAULT_DEV_ID
#endif /* SET_CCFG_IEEE_BLE_0 */

#endif /* CONFIGURATION_H_ */
