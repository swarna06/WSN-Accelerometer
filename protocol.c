/*
 * protocol.c
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#include <stdint.h>

#include <driverlib/aon_event.h>
#include <driverlib/sys_ctrl.h>

#include "protocol.h"
#include "rf_core.h"
#include "timing.h"
#include "log.h"

// Module variables
pro_control_t prc;
static uint8_t pro_comm_buf[PRO_COMM_BUF_LEN];

void Pro_Init()
{
    // Initialize control structure
    prc.state = PRO_S_WAIT_RF_CORE_INIT; // first wait RF core initialization
    prc.flags = 0;
    prc.mode = PRO_MODE_SLV; // default mode
    prc.comm_buf_p = pro_comm_buf;
    prc.tx_param.buf = pro_comm_buf;
    prc.rx_result.buf = pro_comm_buf;
    prc.rx_result.buf_len = PRO_COMM_BUF_LEN;
    prc.error_count = PRO_MAX_ERROR_COUNT;
}

void Pro_Process()
{
    if (Rfc_Error())
        exit(0); // TODO error handling

    if (!Rfc_Ready() || prc.state == PRO_S_IDLE)
        return;

    switch (prc.state)
    {
    case PRO_S_IDLE:
        break;

    case PRO_S_WAIT_RF_CORE_INIT:
        prc.state = PRO_S_IDLE;
        break;

    case PRO_S_WAIT_RAT_SYNC:
        prc.error_count = PRO_MAX_ERROR_COUNT;
        if (prc.mode == PRO_MODE_MSTR)
            prc.state = PRO_S_MSTR_SEND_SYNC_PKT;
        else // slave mode
            prc.state = PRO_S_SLV_START_RX;

        break;

    // ********************************
    // Master mode states
    // ********************************
    case PRO_S_MSTR_SEND_SYNC_PKT:

        // Wait for the start of the next RTC period (up to ~30 us)
        Tm_Synch_With_RTC();

        // Get time stamps from RAT and RTC right after synchronization
        prc.ts.rat_t0 = Rfc_Get_RAT_Time();
        prc.ts.rtc_t0 = Tm_Get_RTC_Time();

        // Schedule TX of synchronization packet
        prc.ts.rat_t1 = prc.ts.rat_t0 + PRO_DELAY_RAT_SYNC_PKT_TX;
        prc.tx_param.rat_start_time = prc.ts.rat_t1;
        Rfc_BLE5_Adv_Aux(&prc.tx_param);

        prc.state = PRO_S_MSTR_WAIT_SYNC_PKT_SENT;

        break;

    case PRO_S_MSTR_WAIT_SYNC_PKT_SENT:
        // Start reception
        Rfc_BLE5_Scanner(PRO_TOUT_SYNC_RESP_USEC);
        prc.state = PRO_S_MSTR_WAIT_SYNC_RESP;
        break;

    case PRO_S_MSTR_WAIT_SYNC_RESP:

        Rfc_BLE5_Get_Scanner_Result(&prc.rx_result);
        if (prc.rx_result.err_flags == 0) // no errors ?
        {
            // Calculate round-trip time (RTT)
            prc.ts.rat_t4 = prc.rx_result.rat_timestamp;
            uint32_t rtt = Tm_Delta_Time32(prc.ts.rat_t1, prc.ts.rat_t4);

            // Prepare packet with time stamps (sync start time (RTC), TX delay and RTT (both RAT))
            uint32_t* buf_int32_p = (uint32_t*)prc.tx_param.buf;
            buf_int32_p[0] = prc.ts.rtc_t0;
            buf_int32_p[1] = PRO_DELAY_RAT_SYNC_PKT_TX;
            buf_int32_p[2] = rtt;
            prc.tx_param.len = PRO_SYNC_RESULT_LEN;

            // Send synchronization results
            prc.tx_param.rat_start_time = 0; // transmit ASAP
            Rfc_BLE5_Adv_Aux(&prc.tx_param);
            prc.state = PRO_S_MSTR_WAIT_SYNC_RESULT_SENT;
        }
        else
        {
            prc.error_count--;
            if (!prc.error_count)
            {
                // TODO hanle error
                prc.flags &= ~PRO_F_SYNC_DONE;
                prc.state = PRO_S_IDLE;
            }
            else
                prc.state = PRO_S_MSTR_SEND_SYNC_PKT; // retry
        }

        break;

    case PRO_S_MSTR_WAIT_SYNC_RESULT_SENT:
        prc.flags |= PRO_F_SYNC_DONE;
        prc.state = PRO_S_IDLE;
        break;

    // ********************************
    // Slave mode states
    // ********************************
    case PRO_S_SLV_START_RX:

        // Start reception
        Rfc_BLE5_Scanner(PRO_TOUT_SYNC_START_USEC);
        prc.state = PRO_S_SLV_WAIT_SYNC_PKT;
        break;

    case PRO_S_SLV_WAIT_SYNC_PKT:

        Rfc_BLE5_Get_Scanner_Result(&prc.rx_result);
        if (prc.rx_result.err_flags == 0 &&
            prc.rx_result.payload_len == 1) // no errors ?
        {
            // Save reception time stamp and calculate TX time of response
            prc.ts.rat_t2 = prc.rx_result.rat_timestamp;
            prc.ts.rat_t3 = prc.rx_result.rat_timestamp + PRO_DELAY_RAT_SYNC_RESP_TX;

            // Schedule transmission of response
            prc.tx_param.rat_start_time = prc.ts.rat_t3;
            Rfc_BLE5_Adv_Aux(&prc.tx_param);

            prc.state = PRO_S_SLV_WAIT_RESP_SENT;
        }
        else
        {
            prc.error_count--;
            if (!prc.error_count)
            {
                // TODO hanle error
                prc.flags &= ~PRO_F_SYNC_DONE;
                prc.state = PRO_S_IDLE;
            }
            else
                prc.state = PRO_S_SLV_START_RX; // retry
        }
        break;

    case PRO_S_SLV_WAIT_RESP_SENT:

        // Start reception
        Rfc_BLE5_Scanner(PRO_TOUT_SYNC_START_USEC);
        prc.state = PRO_S_SLV_WAIT_SYNC_RESULT;
        break;

    case PRO_S_SLV_WAIT_SYNC_RESULT:

        Rfc_BLE5_Get_Scanner_Result(&prc.rx_result);
        if (prc.rx_result.err_flags == 0 &&
            prc.rx_result.payload_len == PRO_SYNC_RESULT_LEN) // no errors ?
        {
            // Extract results
            uint32_t* buf_int32_p = (uint32_t*)prc.tx_param.buf;
            uint32_t mstr_rtc_t0 = buf_int32_p[0];
            uint32_t mstr_delay = buf_int32_p[1];
//            uint32_t rtt = buf_int32_p[2];
            uint32_t rtt = PRO_DELAY_RAT_SYNC_RESP_TX; // FIXME this value should be measured not hardcoded

            // Calculate clock offset
            Tm_Synch_With_RTC(); // wait for next RTC cycle
            uint32_t rat_curr_time = Rfc_Get_RAT_Time();
            uint32_t rtc_curr_time = Tm_Get_RTC_Time();

            uint32_t rat_time_since_t0 = rat_curr_time - prc.ts.rat_t3
                                         + (rtt + PRO_DELAY_RAT_SYNC_RESP_TX)/2
                                         + mstr_delay; // TODO this can be improved

//            uint32_t slv_rtc_t0 = rtc_curr_time - (rat_time_since_t0 / RAD_RAT_TICKS_PER_RTC_TICK);
            uint32_t slv_rtc_t0 = rtc_curr_time - (rat_time_since_t0 / RAD_RAT_TICKS_PER_RTC_TICK) - 190;
                                  // FIXME trimming of slv_rtc_t0 to compensate for 4 ms offset

            int32_t clk_offset = mstr_rtc_t0 - slv_rtc_t0;

            // Adjust local clock
            Tm_Synch_With_RTC(); // wait for next RTC cycle
            rtc_curr_time = Tm_Get_RTC_Time();
            uint32_t rtc_new_time = rtc_curr_time + clk_offset;
            uint32_t rtc_new_time_sec = rtc_new_time >> 16;
            uint32_t rtc_new_time_subsec = (rtc_new_time & 0x0000FFFF) << 16;
            HWREG(AON_RTC_BASE + AON_RTC_O_SEC) = rtc_new_time_sec;
            HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC) = rtc_new_time_subsec;

            // Adjust system tick RTC channel
            uint32_t new_cmp_val;
            new_cmp_val = rtc_new_time + (TM_RTC_TICKS_PER_MSEC - rtc_new_time%TM_RTC_TICKS_PER_MSEC);
            AONRTCCompareValueSet(AON_RTC_CH2, new_cmp_val);

            // Adjust RTC CH1 compare value
            new_cmp_val = rtc_new_time + (TM_ABS_TIME_PER_TICKS - rtc_new_time%TM_ABS_TIME_PER_TICKS);
            IntDisable(INT_AON_PROG0); // critical section !!!
            AONRTCCompareValueSet(AON_RTC_CH1, new_cmp_val);
            IntEnable(INT_AON_PROG0);
//            Tm_Adjust_Counters();

            PRINTF("mstr_rtc_t0: %lu, mstr_delay: %lu, rtt: %lu, slv_rat_t0: %lu, clk_offset: %d\r\n",
                   mstr_rtc_t0, mstr_delay, rtt, slv_rtc_t0, clk_offset);

            prc.flags |= PRO_F_SYNC_DONE;
            prc.state = PRO_S_IDLE;
        }
        else
        {
            prc.error_count--;
            if (!prc.error_count)
            {
                // TODO hanle error
                prc.flags &= ~PRO_F_SYNC_DONE;
                prc.state = PRO_S_IDLE;
            }
            else
                prc.state = PRO_S_SLV_START_RX; // retry
        }
        break;
    }
}

bool Pro_Set_Mode(bool mode)
{
    if (!Pro_Ready())
        return false;

    prc.mode = mode;
    return true;
}

bool Pro_Start_Synch()
{
    if (!Pro_Ready())
        return false;

    // Synchronize RAT and RTC
    if (!Rfc_Synchronize_RAT())
        return false;

    // Minimum packet length
    prc.tx_param.buf[0] = 0;
    prc.tx_param.len = 1; // minimum length

    prc.flags &= ~PRO_F_SYNC_DONE;
    prc.state = PRO_S_WAIT_RAT_SYNC;

    return true;
}

inline bool Pro_Ready()
{
    return (prc.state == PRO_S_IDLE);
}

bool Pro_Sync_Done()
{
    if (prc.flags & PRO_F_SYNC_DONE)
    {
        prc.flags &= ~PRO_F_SYNC_DONE;
        return true;
    }
    else
        return false;
}
