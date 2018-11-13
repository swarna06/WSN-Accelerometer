/*
 * coordinator.c
 *
 *  Created on: Nov 2, 2018
 *      Author: alvaro
 */

#include "driverlib/gpio.h"

#include "coordinator.h"
#include "host_interface.h"
#include "timing.h"
#include "log.h"
#include "protocol.h"

#include "board.h"

static crd_control_t crc;
static uint16_t sync_periods_ms[10] = {100, 200, 500,
                                       1000, 2000, 50000,
                                       10000, 20000,30000, 60000};

void Crd_Init()
{
    crc.state = CRD_S_WAIT_COMMAND;
    crc.mode = PRO_MODE_SLV;
    crc.sync = false;
    crc.per_sync_idx = 4;
}

void Crd_Process()
{
    switch (crc.state)
    {
    case CRD_S_WAIT_COMMAND:

        if (Hif_Command_Received())
        {
            uint32_t cmd = Hif_Get_Command();
            switch (cmd)
            {
            case HFI_CMD_MSTR_MODE:
                Pro_Set_Mode(PRO_MODE_MSTR);
                crc.mode = PRO_MODE_MSTR;
                break;
            case HFI_CMD_SLV_MODE:
                Pro_Set_Mode(PRO_MODE_SLV);
                crc.mode = PRO_MODE_SLV;
                break;
            case HFI_CMD_START_SYNC:
                crc.sync = !crc.sync;
                if (crc.sync)
                {
                    Pro_Start_Synch();
                    crc.state = CRD_S_WAIT_SYNC;
                    Tm_Start_Period(TM_PER_COORD_ID, sync_periods_ms[crc.per_sync_idx]);
                }
                break;
            case HFI_CMD_ENABLE_GPIO_INT:
                // TODO
                break;
            default:
                if (cmd >= '0' && cmd <= '9')
                    crc.per_sync_idx = cmd - '0';
                break;
            }
        }
        else if (crc.sync)
        {
            if (crc.mode == PRO_MODE_MSTR && !Tm_Period_Completed(TM_PER_COORD_ID))
                break;

            Pro_Start_Synch();
            crc.state = CRD_S_WAIT_SYNC;
        }
        break;

    case CRD_S_WAIT_SYNC:

        if (Pro_Ready())
        {
            if (Pro_Sync_Done())
            {
                GPIO_toggleDio(BRD_LED1);  // heart beat
                Log_Line("Sync ok");
            }
            else
            {
                if (crc.mode == PRO_MODE_SLV)
                    Log_String_Literal(".");
                else
                    Log_Line("Sync NOT ok");
            }

            crc.state = CRD_S_WAIT_COMMAND;
        }
        break;
    }
}



