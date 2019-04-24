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
#define CFG_DEFAULT_DEV_ID              0
#define CFG_SENSOR_NODE_NUM             2
#if (CFG_DEFAULT_DEV_ID > CFG_SENSOR_NODE_NUM)
#error Device id is greater than total number of sensors
#endif // #if (CFG_DEFAULT_DEV_ID > CFG_SENSOR_NODE_NUM)
#if (CFG_SENSOR_NODE_NUM <= 0)
#error Sensor node number must be greater than 0
#endif // #if (CFG_DEFAULT_DEV_ID > CFG_SENSOR_NODE_NUM)

// Debug settings
#define CFG_DEBUG_DUMMY_SLEEP           CFG_SETTING_ENABLED
#define CFG_DEBUG_SLEEP_OUT             CFG_SETTING_DISABLED
#define CFG_DEBUG_START_OF_FRAME_OUT    CFG_SETTING_DISABLED
#define CFG_DEBUG_LF_OSC_OUT            CFG_SETTING_DISABLED
#define CFG_DEBUG_RADIO_OUT             CFG_SETTING_DISABLED
#define CFG_DEBUG_VERBOSE               CFG_SETTING_DISABLED
#define CFG_DEBUG_PROFILING             CFG_SETTING_ENABLED
#define CFG_DEBUG_FSM_STATE             CFG_SETTING_DISABLED
#define CFG_DEBUG_RFC_ERR_BUTTON        CFG_SETTING_DISABLED
#define CFG_DEBUG_EXT_SYNC              CFG_SETTING_ENABLED  // EXT_SYNC=01

// Reliability test configuration
#define CFG_RELIAB_TEST_PHY_MODES       {RFC_PHY_MODE_125KBPS}

#define CFG_RELIAB_TEST_TX_POWER        {RFC_TX_POW_0dBm}

#define CFG_RELIAB_TEST_CHANNELS        {4}

// Device (CC2640R2F) configuration (see ccfg.h)
#ifndef SET_CCFG_IEEE_BLE_0
#define SET_CCFG_IEEE_BLE_0             CFG_DEFAULT_DEV_ID
#endif /* SET_CCFG_IEEE_BLE_0 */

#endif /* CONFIGURATION_H_ */
