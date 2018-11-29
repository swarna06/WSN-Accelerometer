/*
 * sniffer.c
 *
 *  Created on: 29 Nov 2018
 *      Author: swarna
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>

#include <driverlib/rfc.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_data_entry.h>
#include <inc/hw_rfc_rat.h>
#include <driverlib/rf_ble_mailbox.h>
#include <driverlib/rf_ble_cmd.h>

#include <driverlib/aon_rtc.h>

#include "rf_core.h"
#include "timing.h"
#include "smartrf_settings.h"
#include "log.h"
#include "misc.h"
#include "power_management.h"
#include "printf.h"

void BLE_sniffer(bool listen)
{
    rfc_rx_result_t rx_result;
    uint8_t buf[256];
    rx_result.buf = buf;
    rx_result.buf_len = sizeof(buf);


    if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
            {
               // Brd_Led_Toggle(BRD_LED0);
                //Log_Value("count: ", count++);
                if(!listen && Rfc_Ready())
                {
                    Rfc_BLE5_Scanner(800000); //1 sec
                    listen = true;
                }
            }

    if(rx_result.buf_len) (void) 0;
    if(listen && Rfc_Ready())
    {
        listen = false;
        Rfc_BLE5_Get_Scanner_Result(&rx_result);
        if(!rx_result.err_flags)
        {
            for(int i=0; i< rx_result.payload_len; i++)
                Log_Value_Hex(buf[i]);
            Log_Line("");

        }
        else Log_Line("no packet");

    }
}
