/*
 * host_interface.h
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#ifndef HOST_INTERFACE_H_
#define HOST_INTERFACE_H_

// Operation modes
#define HIF_OP_MODE_UNDEFINED       0
#define HIF_OP_MODE_MSTR            'M'
#define HIF_OP_MODE_SLV             'S'

// Supported commands
typedef enum
{
    HFI_CMD_NULL = 0,
    HFI_CMD_MSTR_MODE = 'M',
    HFI_CMD_SLV_MODE = 'S',
    HFI_CMD_START_SYNC = 'Y',
    HFI_CMD_ENABLE_GPIO_INT = 'X',
} hfi_command_t;

void Hif_Init();

void Hif_Process();

bool Hif_Data_Received();

bool Hif_Command_Received();

hfi_command_t Hif_Get_Command();

#endif /* HOST_INTERFACE_H_ */
