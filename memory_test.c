/*
 * memory_test.c
 *
 *  Created on: Aug 31, 2018
 *      Author: alvaro
 */

#include <driverlib/ioc.h>
#include <driverlib/gpio.h>
//#include <stdio.h>

#include "memory_test.h"
#include "spi_bus.h"
#include "board.h"
#include "timing.h"
#include "printf.h"
//#include "myassert.h"
#include "log.h"

#if (MEM_PART_NUM == MEM_23LCV1024)
#define ARRAY_LEN 4
static uint16_t tx_buf[ARRAY_LEN];

#elif (MEM_PART_NUM == MEM_CY15B104Q)
#define ARRAY_LEN 4
static uint16_t tx_buf[ARRAY_LEN];

#endif

#if (MEM_PART_NUM == MEM_23LCV1024)
static uint8_t Mem_Get_Op_Mode()
{
    uint8_t instruction = MEM_I_RDMR;
    uint8_t op_mode = 0;

    Spi_Transaction(&instruction, sizeof(instruction),
                    &op_mode, sizeof(op_mode),
                    MEM_SPI_CS_PIN);

    return op_mode;
}
#elif (MEM_PART_NUM == MEM_CY15B104Q)
static uint8_t Mem_Get_Op_Mode()
{
    uint8_t instruction = MEM_I_RDMR;
    uint8_t op_mode = 0;

    Spi_Transaction(&instruction, sizeof(instruction),
                    &op_mode, sizeof(op_mode),
                    MEM_SPI_CS_PIN);

    return op_mode;
}

#endif

#if (MEM_PART_NUM == MEM_23LCV1024)
static void Mem_Set_Op_Mode(uint8_t op_mode)
{
    const uint8_t frame[] = {MEM_I_WRMR, op_mode};

    Spi_Transaction(frame, sizeof(frame),
                    NULL, 0,
                    MEM_SPI_CS_PIN);
}
#elif (MEM_PART_NUM == MEM_CY15B104Q)
static void Mem_Set_Op_Mode(uint8_t op_mode)
{
    const uint8_t frame[] = {MEM_I_WRMR, op_mode};

    Spi_Transaction(frame, sizeof(frame),
                    NULL, 0,
                    MEM_SPI_CS_PIN);
}
#endif

void Mem_Init()
{
    // Configure chip select (CS) pin
    Spi_Init_CS_Pin(MEM_SPI_CS_PIN);

#if (MEM_PART_NUM == MEM_23LCV1024)
    // Set operation mode: sequential
    Mem_Set_Op_Mode(MEM_OP_MODE_SEQ);
    Tm_Delay(TM_USEC_TO_TICKS(1));

    uint8_t op_mode = Mem_Get_Op_Mode();
    Assert(op_mode == MEM_OP_MODE_SEQ);
    myprintf("Mem op_mode %02x\r\n", op_mode);

    for (size_t n = 0; n < ARRAY_LEN; n++)
        tx_buf[n] = n;
#elif (MEM_PART_NUM == MEM_CY15B104Q)


#endif
}

void Mem_Read_Array(const uint32_t addr,
                    uint8_t* dest,
                    const size_t dest_count)
{
#if (MEM_PART_NUM == MEM_23LCV1024)
    const uint8_t frame[] = {MEM_I_READ, ((uint8_t*)&addr)[2], ((uint8_t*)&addr)[1], ((uint8_t*)&addr)[0]};

    Spi_Transaction(frame, sizeof(frame),
                    dest, dest_count,
                    MEM_SPI_CS_PIN);
#elif (MEM_PART_NUM == MEM_CY15B104Q)


#endif

}

void Mem_Write_Array(const uint32_t addr,
                     const uint8_t* src,
                     const size_t src_count)
{
#if (MEM_PART_NUM == MEM_23LCV1024)
    const uint8_t frame[] = {MEM_I_WRITE, ((uint8_t*)&addr)[2], ((uint8_t*)&addr)[1], ((uint8_t*)&addr)[0]};

    Spi_Assert_CS(MEM_SPI_CS_PIN);
    Spi_Send(frame, sizeof(frame));
    Spi_Send(src, src_count);
    Spi_Flush_Fifo(); // wait end of TX
    Spi_Deassert_CS(MEM_SPI_CS_PIN);
#elif (MEM_PART_NUM == MEM_CY15B104Q)


#endif
}

void Mem_FRAM_Get_Dev_Id(uint8_t *dest)
{
    if (dest == NULL) return;

    uint8_t opcode = MEM_OC_RDID;

    Spi_Transaction(&opcode, sizeof(opcode),
                    dest, MEM_DEV_ID_LEN,
                    MEM_SPI_CS_PIN);
}

void Mem_ReadStatus()
{
    uint8_t opcode = MEM_OC_RDSR;
    uint8_t status = 0;
    Spi_Transaction(&opcode, sizeof(opcode),
                            &status, sizeof(status),
                            MEM_SPI_CS_PIN);
    Log_Line("status:");Log_Value_Int(status);Log_String_Literal("\r\n"); //01000000 = 64

}
void Mem_Read(uint32_t addr, uint8_t *data_ptr,uint32_t total_count)
{
    uint8_t opcode_read = MEM_OC_READ;
    const uint8_t frame[] = {MEM_OC_READ, ((uint8_t)(addr>>16)), ((uint8_t)(addr>>8)), ((uint8_t)(addr))};
    Spi_Transaction(&frame, sizeof(frame),
                                    data_ptr, total_count,
                                    MEM_SPI_CS_PIN);
}

void Mem_Write(uint32_t addr, uint8_t* data_ptr, uint32_t total_count)
{
    //send Write enable command opcode
    uint8_t opcode_wren = MEM_OC_WREN;
    Spi_Transaction(&opcode_wren, sizeof(opcode_wren),
                        NULL, 0,
                        MEM_SPI_CS_PIN);


    //send Write command opcode
    uint8_t opcode_write = MEM_OC_WRITE;
    const uint8_t frame[] = {MEM_OC_WRITE, ((uint8_t)(addr>>16)), ((uint8_t)(addr>>8)), ((uint8_t)(addr)), data_ptr[0], data_ptr[1], data_ptr[2], data_ptr[3], data_ptr[4],data_ptr[5],data_ptr[6],data_ptr[7],data_ptr[8],data_ptr[9],data_ptr[10],data_ptr[11],data_ptr[12],data_ptr[13],data_ptr[14],data_ptr[15]};
   //const uint8_t frame[] = {MEM_OC_WRITE, ((uint8_t)(addr>>16)), ((uint8_t)(addr>>8)), ((uint8_t)(addr)), data_ptr[0], data_ptr[1], data_ptr[2], data_ptr[3]};

    Spi_Transaction(&frame, sizeof(frame),
                            NULL, 0,
                            MEM_SPI_CS_PIN);
}
void Mem_Flag_Set()
{
    uint32_t addr=0x00000;
    //send Write enable command opcode
    uint8_t opcode_wren = MEM_OC_WREN;
    Spi_Transaction(&opcode_wren, sizeof(opcode_wren),
                        NULL, 0,
                        MEM_SPI_CS_PIN);


    //send Write command opcode
    uint8_t opcode_write = MEM_OC_WRITE;
    const uint8_t frame[] = {MEM_OC_WRITE, ((uint8_t)(addr>>16)), ((uint8_t)(addr>>8)), ((uint8_t)(addr)), 1};

    Spi_Transaction(&frame, sizeof(frame),
                            NULL, 0,
                            MEM_SPI_CS_PIN);
}
void Mem_Flag_Reset()
{
    uint32_t addr=0x00000;
    //send Write enable command opcode
    uint8_t opcode_wren = MEM_OC_WREN;
    Spi_Transaction(&opcode_wren, sizeof(opcode_wren),
                        NULL, 0,
                        MEM_SPI_CS_PIN);


    //send Write command opcode
    uint8_t opcode_write = MEM_OC_WRITE;
    const uint8_t frame[] = {MEM_OC_WRITE, ((uint8_t)(addr>>16)), ((uint8_t)(addr>>8)), ((uint8_t)(addr)), 0};

    Spi_Transaction(&frame, sizeof(frame),
                            NULL, 0,
                            MEM_SPI_CS_PIN);
}
bool Mem_Flag_Read()
{
    uint32_t addr=0x00000;
    uint8_t data_ptr;
    uint8_t opcode_read = MEM_OC_READ;
    const uint8_t frame[] = {MEM_OC_READ, ((uint8_t)(addr>>16)), ((uint8_t)(addr>>8)), ((uint8_t)(addr))};
    Spi_Transaction(&frame, sizeof(frame),
                                    &data_ptr, sizeof(data_ptr),
                                    MEM_SPI_CS_PIN);
    return data_ptr;
}
void Mem_Test()
{
#if (MEM_PART_NUM == MEM_23LCV1024)
    uint32_t addr = 16;
    uint16_t rx_buf[ARRAY_LEN];

//    while(1)
    {
        Mem_Read_Array(addr, (uint8_t*)rx_buf, sizeof(rx_buf));
        Tm_Delay(TM_USEC_TO_TICKS(1)); // time between commands

        for (size_t n = 0; n < ARRAY_LEN; n++)
            tx_buf[n]++;
        Mem_Write_Array(addr, (uint8_t*)tx_buf, sizeof(tx_buf));

        myprintf("rx_buf:");
        for (size_t n = 0; n < ARRAY_LEN; n++)
            myprintf(" %d", rx_buf[n]);
        myprintf("\r\n");
    }
#elif (MEM_PART_NUM == MEM_CY15B104Q)

    uint8_t dev_id[MEM_DEV_ID_LEN];
    Mem_FRAM_Get_Dev_Id(dev_id);

    Log_Line("dev ID:");
    for (size_t n = 0; n < MEM_DEV_ID_LEN; n++)
        Log_Value_Hex(dev_id[n]);
    Log_String_Literal("\r\n");



#endif
}

