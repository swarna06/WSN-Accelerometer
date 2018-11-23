/*
 * power_management.h
 *
 *  Created on: Nov 19, 2018
 *      Author: alvaro
 */

#ifndef POWER_MANAGEMENT_H_
#define POWER_MANAGEMENT_H_

// Power domain flags
#define PMA_POWER_DOMAIN_RF_CORE    0x1000
#define PMA_POWER_DOMAIN_SERIAL     0x2000
#define PMA_POWER_DOMAIN_PERIPH     0x4000

typedef enum
{
    PMA_PERIPH_RF_CORE = PMA_POWER_DOMAIN_RF_CORE,

    PMA_PERIPH_UART0 = PMA_POWER_DOMAIN_SERIAL | 0x01,

    PMA_PERIPH_GPIO = PMA_POWER_DOMAIN_PERIPH | 0x10,
} pma_periph_id_t;

typedef struct
{
    pma_periph_id_t powered_peripherals;
} pma_control_t;

void Pma_Init();

void Pma_Power_On_Peripheral(uint16_t periph_id);

void Pma_CPU_Sleep(uint32_t tout_ms);

void Pma_MCU_Sleep(uint32_t rtc_wakeup_time);

#endif /* POWER_MANAGEMENT_H_ */
