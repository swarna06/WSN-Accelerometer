/*
 * device_config.h
 *
 *  Created on: Nov 27, 2018
 *      Author: alvaro
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

// Values to enable/disable settings
#define CFG_SETTING_DISABLED            0
#define CFG_SETTING_ENABLED             1

// Network parameters
#define CFG_DEFAULT_DEV_ID              2
#define CFG_SENSOR_NODE_NUM             3
#if (CFG_DEFAULT_DEV_ID > CFG_SENSOR_NODE_NUM)
#error Device id is greater than total number of sensors
#endif // #if (CFG_DEFAULT_DEV_ID > CFG_SENSOR_NODE_NUM)

// Debug settings
#define CFG_DEBUG_DUMMY_SLEEP           CFG_SETTING_DISABLED
#define CFG_DEBUG_SLEEP_OUT             CFG_SETTING_ENABLED
#define CFG_DEBUG_START_OF_FRAME_OUT    CFG_SETTING_ENABLED
#define CFG_DEBUG_LF_OSC_OUT            CFG_SETTING_ENABLED
#define CFG_DEBUG_RADIO_OUT             CFG_SETTING_ENABLED

// Device (CC2640R2F) configuration (see ccfg.h)
#ifndef SET_CCFG_IEEE_BLE_0
#define SET_CCFG_IEEE_BLE_0             CFG_DEFAULT_DEV_ID
#endif /* SET_CCFG_IEEE_BLE_0 */

#endif /* CONFIGURATION_H_ */
