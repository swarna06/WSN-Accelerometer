/*
 * power_management.h
 *
 *  Created on: Nov 19, 2018
 *      Author: alvaro
 */

#ifndef POWER_MANAGEMENT_H_
#define POWER_MANAGEMENT_H_

#include "configuration.h"
extern int16_t abuf[5];
// Debug signal; indicates active and sleep states
#if (CFG_DEBUG_SLEEP_OUT == CFG_SETTING_ENABLED)
#define PMA_SLEEP_OUT
#endif // #if (CFG_DEBUG_SLEEP_OUT == CFG_SETTING_ENABLED)

// 'Dummy' sleep; busy-wait instead to go to deep sleep
#if (CFG_DEBUG_DUMMY_SLEEP == CFG_SETTING_ENABLED)
#define PMA_DUMMY_SLEEP
#endif // #if (CFG_DEBUG_SLEEP_OUT == CFG_SETTING_ENABLED)

// Wake up procedure execution time
//#define PMA_WAKEUP_TIME_USEC        400
#define PMA_WAKEUP_TIME_USEC        2048 // xxx measure this time again later and update this constant

// Minimum sleep time
#define PMA_MIN_SLEEP_RTC_TICKS     (2000/TM_RTC_USEC_PER_TICK)

// Battery voltage readings
#define PMA_BATT_VOLT_SAMP_NUM      4

// Power domain flags
#define PMA_F_DOMAIN_RF_CORE        0x8000
#define PMA_F_DOMAIN_SERIAL         0x4000
#define PMA_F_DOMAIN_PERIPH         0x2000


typedef enum
{
    PMA_PERIPH_RF_CORE = PMA_F_DOMAIN_RF_CORE,

    PMA_PERIPH_UART0 = PMA_F_DOMAIN_SERIAL,

    PMA_PERIPH_GPIO = PMA_F_DOMAIN_PERIPH,
} pma_peripherals_t;

typedef struct
{
    size_t idx;
    uint16_t batt_volt[PMA_BATT_VOLT_SAMP_NUM]; // used to implement a moving average
} pma_control_t;

void Pma_Init();

bool Pma_Batt_Volt_Meas_Ready();

void Pma_Process();

void Pma_Power_On_Peripheral(uint16_t peripheral);

void Pma_CPU_Sleep(uint32_t tout_ms);

void Pma_MCU_Sleep(uint32_t rtc_wakeup_time);

void Pma_MCU_Wakeup();

uint8_t Pma_Get_Batt_Volt_Fixed_Point();

void Pma_Get_Batt_Volt_Parts(uint8_t batt_volt,
                             uint8_t* int_part,
                             uint16_t* frac_part);

void Pma_Get_Batt_Volt(uint8_t* int_part, uint16_t* frac_part);

#endif /* POWER_MANAGEMENT_H_ */
