/*
 * radio.h
 *
 *  Created on: 21 oct. 2018
 *      Author: Alvaro
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <driverlib/rf_common_cmd.h>

// Macros for printing error information
#define Rad_Print_RFCPEIFG_Err() \
        PRINTF("ERR: func: %s, RFCPEIFG: %p\r\n", \
               (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))

#define Rad_Print_Radio_Op_Err(op_p) \
        PRINTF("ERR: func: %s, commandNo: %04X, CMDSTA: %p, status: %04X, RFCPEIFG: %p\r\n", \
               __func__, (op_p)->commandNo, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA), \
               (op_p)->status, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))

#define Rad_Print_Direct_Cmd_Err(id) \
        PRINTF("ERR: func: %s, commandNo: %04X, CMDSTA: %p, RFCPEIFG: %p\r\n", \
               __func__, id, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA), \
               (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))

// CPE error interrupt flags mask
#define RAD_M_RFCPEIFG_ERROR    (RFC_DBELL_RFCPEIFG_INTERNAL_ERROR | \
                                 RFC_DBELL_RFCPEIFG_SYNTH_NO_LOCK)

// Error flag for CMDSTA register and status field in
#define RAD_F_STATUS_ERR        0x0800

// Return values
#define RAD_OK                  0
#define RAD_E_CPE_CMDSTA        -1
#define RAD_E_RFCPEIFG          -2
#define RAD_E_STATUS_FIELD      -3

void Rad_Init();

int Rad_Execute_Radio_Op(volatile rfc_radioOp_t* radio_op_p);

int Rad_Execute_Direct_Cmd(uint16_t cmd_id);

#endif /* RADIO_H_ */
