/*
 * power_management.h
 *
 *  Created on: Nov 19, 2018
 *      Author: alvaro
 */

#ifndef POWER_MANAGEMENT_H_
#define POWER_MANAGEMENT_H_

// Wake up procedure execution time
#define PMA_WAKEUP_TIME_USEC        400

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

void Pma_Init();

void Pma_Power_On_Peripheral(uint16_t peripheral);

void Pma_CPU_Sleep(uint32_t tout_ms);

void Pma_MCU_Sleep(uint32_t rtc_wakeup_time);

void Pma_MCU_Wakeup();

void Pma_Dummy_MCU_Sleep(uint32_t rtc_wakeup_time);

#endif /* POWER_MANAGEMENT_H_ */
