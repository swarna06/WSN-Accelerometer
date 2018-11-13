/*
 * host_interface.c
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#include <driverlib/uart.h>

#include "host_interface.h"
//#include "printf.h"
#include "log.h"

static int32_t hif_uart_datum;
static hfi_command_t hif_command;

void Hif_Init()
{
    hif_command = HFI_CMD_NULL;
}

void Hif_Process()
{
    // Check if the command is valid
    switch (hif_uart_datum)
    {
    case HFI_CMD_NULL: break;
    case HFI_CMD_MSTR_MODE: break;
    case HFI_CMD_SLV_MODE: break;
    case HFI_CMD_START_SYNC: break;
    case HFI_CMD_ENABLE_GPIO_INT: break;
    default:
        if (hif_uart_datum >= '0' && hif_uart_datum <= '9')
            break;
        else
        {
            Log_Line("Unknown cmd");
            hif_command = HFI_CMD_NULL;
            return;
        }
    }

    hif_command = hif_uart_datum;
    Log_Char("cmd: ", hif_command);
}

bool Hif_Data_Received()
{
    hif_uart_datum = UARTCharGetNonBlocking(UART0_BASE);
    return (hif_uart_datum < 0 ? false : true);
}

bool Hif_Command_Received()
{
    return (hif_command == HFI_CMD_NULL ? false : true);
}

hfi_command_t Hif_Get_Command()
{
    uint32_t result = hif_command;
    hif_command = HFI_CMD_NULL;
    return result;
}


