/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
//#include "fsl_sd.h"
//#include "fsl_debug_console.h"
//#include "ff.h"
//#include "diskio.h"
//#include "fsl_sd_disk.h"
//#include "app.h"
//#include "pin_mux.h"
//#include "clock_config.h"
//#include "board.h"
//#include "sdmmc_config.h"
//#include "fsl_common.h"
//#include "fsl_lpuart.h"
//#include "flexcanfd.h"
#include "MCXN-CANFD-LIN-Log.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* buffer size (in byte) for read/write operations */
#define BUFFER_SIZE (513U)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief wait card insert function.
 */
static status_t sdcardWaitCardInsert(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static FATFS g_fileSystem; /* File system object */
FIL g_fileObject;   /* File object */
FIL canfd_fileObject;   /* File object */

/* @brief decription about the read/write buffer
 * The size of the read/write buffer should be a multiple of 512, since SDHC/SDXC card uses 512-byte fixed
 * block length and this driver example is enabled with a SDHC/SDXC card.If you are using a SDSC card, you
 * can define the block length by yourself if the card supports partial access.
 * The address of the read/write buffer should align to the specific DMA data buffer address align value if
 * DMA transfer is used, otherwise the buffer address is not important.
 * At the same time buffer address/size should be aligned to the cache line size if cache is supported.
 */
/*! @brief Data written to the card */
SDK_ALIGN(uint8_t g_bufferWrite[BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);
/*! @brief Data read from the card */
SDK_ALIGN(uint8_t g_bufferRead[BUFFER_SIZE], BOARD_SDMMC_DATA_BUFFER_ALIGN_SIZE);

#define LIN_COM2

#ifdef LIN_COM0
#define DEMO_LIN            LPUART0
#define DEMO_LIN_CLK_FREQ   12000000U
#define DEMO_LIN_IRQn       LP_FLEXCOMM0_IRQn
#define DEMO_LIN_IRQHandler LP_FLEXCOMM0_IRQHandler
#define DisableLinBreak LPUART0->STAT &= ~(LPUART_STAT_LBKDE_MASK);
#define EnableLinBreak  LPUART0->STAT |= LPUART_STAT_LBKDE_MASK;
#endif

#ifdef LIN_COM2
#define DEMO_LIN            LPUART2
#define DEMO_LIN_CLK_FREQ   12000000U
#define DEMO_LIN_IRQn       LP_FLEXCOMM2_IRQn
#define DEMO_LIN_IRQHandler LP_FLEXCOMM2_IRQHandler
#define DisableLinBreak LPUART2->STAT &= ~(LPUART_STAT_LBKDE_MASK);
#define EnableLinBreak  LPUART2->STAT |= LPUART_STAT_LBKDE_MASK;
#endif
/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16



#define IDLE                0x00          /**< IDLE state */
#define SEND_BREAK          0x01          /**< Send break field state */
#define SEND_PID            0x02          /**< send PID state */
#define RECV_SYN            0x03          /**< receive synchronize state */
#define RECV_PID            0x04          /**< receive PID state */
#define IGNORE_DATA         0x05          /**< ignore data state */
#define RECV_DATA           0x06          /**< receive data state */
#define SEND_DATA           0x07          /**< send data state */
#define SEND_DATA_COMPLETED 0x08          /**< send data completed state */
#define PROC_CALLBACK       0x09          /**< proceduce callback state */
#define SLEEP_MODE          0x0A          /**< sleep mode state */
#define UNINIT              0xFF          /**< uninitialize state */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*
  Ring buffer for data input and output, in this example, input data are saved
  to ring buffer in IRQ handler. The main function polls the ring buffer status,
  if there are new data, then send them out.
  Ring buffer full: (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) == txIndex)
  Ring buffer empty: (rxIndex == txIndex)
*/
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */
uint8_t rxbuff[20] = {0};
uint16_t cnt=0, recdatacnt=0;
uint8_t Lin_BKflag=0;
static uint8_t          state = UNINIT;

/*******************************************************************************
 * Code
 ******************************************************************************/

void DEMO_LIN_IRQHandler(void)
{
//    uint8_t data;
//    uint16_t tmprxIndex = rxIndex;
//    uint16_t tmptxIndex = txIndex;

    if (DEMO_LIN->STAT & LPUART_STAT_LBKDIF_MASK)
    {
    	DEMO_LIN->STAT |= LPUART_STAT_LBKDIF_MASK;// clear the bit
        Lin_BKflag = 1;
        cnt = 0;
        state = RECV_SYN;
        DisableLinBreak;
    }
    if (DEMO_LIN->STAT & LPUART_STAT_RDRF_MASK)
    {
      	 rxbuff[cnt] = (uint8_t)((DEMO_LIN->DATA) & 0xff);
		switch(state)
		{
		   case RECV_SYN:
			 if(0x55 == rxbuff[cnt])
			 {
				 state = RECV_PID;
			 }
			 else
			 {
				 state = IDLE;
				 DisableLinBreak;
			 }
			 break;
		   case RECV_PID:
			 if(0xAD == rxbuff[cnt])
			 {
				 state = RECV_DATA;
			 }
			 else if(0xEC == rxbuff[cnt])
			 {
				 state = SEND_DATA;
			 }
			 else
			 {
				 state = IDLE;
				 DisableLinBreak;
			 }
			 break;
		   case RECV_DATA:
			 recdatacnt++;
			 if(recdatacnt >= 4) // 3 Bytes data + 1 Bytes checksum
			 {
				 recdatacnt=0;
				 state = IDLE;
				 EnableLinBreak;
			 }
			 break;
		   default:break;
		}
		cnt++;

    }

    SDK_ISR_EXIT_BARRIER;
}

void uart_LIN_break( LPUART_Type *base )
{
	base->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);   //Disable UART0 first
	base->STAT |= LPUART_STAT_BRK13_MASK; //13 bit times
	base->STAT |= LPUART_STAT_LBKDE_MASK;//LIN break detection enable
	base->BAUD |= LPUART_BAUD_LBKDIE_MASK;

	base->CTRL |= (LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
	base->CTRL |= LPUART_CTRL_RIE_MASK;
	EnableIRQ(DEMO_LIN_IRQn);
}
uint8_t LINCheckSum(uint8_t PID, uint8_t *buf, uint8_t lens);
uint8_t LINCalcParity(uint8_t id);
uint8_t sendbuffer[3] = {0x01, 0x02, 0x10};

/*******************************************************************************
 * Code
 ******************************************************************************/

void SD_Fatfs_test( void )
{
    FRESULT error;
    DIR directory; /* Directory object */
    FILINFO fileInformation;
    UINT bytesWritten;
    UINT bytesRead;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
    volatile bool failedFlag           = false;
    char ch                            = '0';
    BYTE work[FF_MAX_SS];

    PRINTF("\r\nFATFS example to demonstrate how to use FATFS with SD card.\r\n");
    PRINTF("\r\nPlease insert a card into board.\r\n");
#if 1
    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        return -1;
    }

    if (f_mount(&g_fileSystem, driverNumberBuffer, 0U))
    {
        PRINTF("Mount volume failed.\r\n");
        return -1;
    }

#if (FF_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        return -1;
    }
#endif

#if FF_USE_MKFS
    PRINTF("\r\nMake file system......The time may be long if the card capacity is big.\r\n");
    if (f_mkfs(driverNumberBuffer, 0, work, sizeof work))
    {
        PRINTF("Make file system failed.\r\n");
        return -1;
    }
#endif /* FF_USE_MKFS */

    PRINTF("\r\nCreate directory......\r\n");
    error = f_mkdir(_T("/dir_1"));
    if (error)
    {
        if (error == FR_EXIST)
        {
            PRINTF("Directory exists.\r\n");
        }
        else
        {
            PRINTF("Make directory failed.\r\n");
            return -1;
        }
    }

    PRINTF("\r\nCreate a file in that directory......\r\n");
    error = f_open(&g_fileObject, _T("/dir_1/f_1.dat"), (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
    if (error)
    {
        if (error == FR_EXIST)
        {
            PRINTF("File exists.\r\n");
        }
        else
        {
            PRINTF("Open file failed.\r\n");
            return -1;
        }
    }

    PRINTF("\r\nCreate a directory in that directory......\r\n");
    error = f_mkdir(_T("/dir_1/dir_2"));
    if (error)
    {
        if (error == FR_EXIST)
        {
            PRINTF("Directory exists.\r\n");
        }
        else
        {
            PRINTF("Directory creation failed.\r\n");
            return -1;
        }
    }

    PRINTF("\r\nList the file in that directory......\r\n");
    if (f_opendir(&directory, "/dir_1"))
    {
        PRINTF("Open directory failed.\r\n");
        return -1;
    }

    for (;;)
    {
        error = f_readdir(&directory, &fileInformation);

        /* To the end. */
        if ((error != FR_OK) || (fileInformation.fname[0U] == 0U))
        {
            break;
        }
        if (fileInformation.fname[0] == '.')
        {
            continue;
        }
        if (fileInformation.fattrib & AM_DIR)
        {
            PRINTF("Directory file : %s.\r\n", fileInformation.fname);
        }
        else
        {
            PRINTF("General file : %s.\r\n", fileInformation.fname);
        }
    }

    memset(g_bufferWrite, 'a', sizeof(g_bufferWrite));
    g_bufferWrite[BUFFER_SIZE - 2U] = '\r';
    g_bufferWrite[BUFFER_SIZE - 1U] = '\n';
#endif
    PRINTF("\r\nWrite/read file until encounters error......\r\n");
    while (true)
    {
        if (failedFlag || (ch == 'q'))
        {
            break;
        }

        PRINTF("\r\nWrite to above created file.\r\n");
        error = f_write(&g_fileObject, g_bufferWrite, sizeof(g_bufferWrite), &bytesWritten);
        if ((error) || (bytesWritten != sizeof(g_bufferWrite)))
        {
            PRINTF("Write file failed. \r\n");
            failedFlag = true;
            continue;
        }

        /* Move the file pointer */
        if (f_lseek(&g_fileObject, 0U))
        {
            PRINTF("Set file pointer position failed. \r\n");
            failedFlag = true;
            continue;
        }

        PRINTF("Read from above created file.\r\n");
        memset(g_bufferRead, 0U, sizeof(g_bufferRead));
        error = f_read(&g_fileObject, g_bufferRead, sizeof(g_bufferRead), &bytesRead);
        if ((error) || (bytesRead != sizeof(g_bufferRead)))
        {
            PRINTF("Read file failed. \r\n");
            failedFlag = true;
            continue;
        }

        PRINTF("Compare the read/write content......\r\n");
        if (memcmp(g_bufferWrite, g_bufferRead, sizeof(g_bufferWrite)))
        {
            PRINTF("Compare read/write content isn't consistent.\r\n");
            failedFlag = true;
            continue;
        }
        PRINTF("The read/write content is consistent.\r\n");

        PRINTF("\r\nInput 'q' to quit read/write.\r\nInput other char to read/write file again.\r\n");
        break;
//        ch = GETCHAR();
//        PUTCHAR(ch);
    }
    PRINTF("\r\nThe example will not read/write file again.\r\n");

    if (f_close(&g_fileObject))
    {
        PRINTF("\r\nClose file failed.\r\n");
        return -1;
    }
}
void write_canfd_file( uint8_t *writedata, uint8_t len );
void sd_savecsv_header( void )
{
    FRESULT error;
    DIR directory; /* Directory object */
    FILINFO fileInformation;
    UINT bytesWritten;
    UINT bytesRead;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
    volatile bool failedFlag           = false;
    char ch                            = '0';
    BYTE work[FF_MAX_SS];
    uint8_t filewritelength;

    PRINTF("\r\nFATFS example to demonstrate how to use FATFS with SD card.\r\n");
    PRINTF("\r\nPlease insert a card into board.\r\n");

    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        return -1;
    }

    if (f_mount(&g_fileSystem, driverNumberBuffer, 0U))
    {
        PRINTF("Mount volume failed.\r\n");
        return -1;
    }

#if (FF_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        return -1;
    }
#endif

#if 0//FF_USE_MKFS
    PRINTF("\r\nMake file system......The time may be long if the card capacity is big.\r\n");
    if (f_mkfs(driverNumberBuffer, 0, work, sizeof work))
    {
        PRINTF("Make file system failed.\r\n");
        return -1;
    }
#endif /* FF_USE_MKFS */


    PRINTF("\r\nCreate directory......\r\n");
    error = f_mkdir(_T("/logs"));
    if (error)
    {
        if (error == FR_EXIST)
        {
            PRINTF("Directory exists.\r\n");
        }
        else
        {
            PRINTF("Make directory failed.\r\n");
            return -1;
        }
    }
#if 0
    PRINTF("\r\nCreate a lin csv......\r\n");
    error = f_open(&g_fileObject, _T("/logs/lin_log.csv"), (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
    if (error)
    {
    	PRINTF("open lin file error.\r\n");
    }
    memset(g_bufferWrite, 0x00, sizeof(g_bufferWrite));
    strcpy(g_bufferWrite, "LIN-id,    length,    data0,    data1,    data2,    data3,    data4,    data5,    data6,    data7\r\n");


    if (f_lseek(&g_fileObject, f_size(&g_fileObject)))//0U))
    {
        PRINTF("Set file pointer position failed. \r\n");

    }
//    error = f_write(&g_fileObject, g_bufferWrite, sizeof(g_bufferWrite), &bytesWritten);
    error = f_write(&g_fileObject, g_bufferWrite, strlen(g_bufferWrite), &bytesWritten);
    if ((error) || (bytesWritten != strlen(g_bufferWrite)))
    {
        PRINTF("Write file failed. \r\n");
    }

    if (f_close(&g_fileObject))
    {
        PRINTF("\r\nClose file failed.\r\n");
        return -1;
    }
#endif

    PRINTF("\r\nCreate a canfd csv......\r\n");
    error = f_open(&canfd_fileObject, _T("/logs/canlog.csv"), (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
    if (error)
    {
    	PRINTF("open canfd file error.\r\n");
    }
    memset(g_bufferWrite, 0x00, sizeof(g_bufferWrite));
    strcpy(g_bufferWrite, "BusType, CANFD/LIN-id,    length,    brs,    dword0,    dword1,    dword2,    dword3,    dword4,    dword5,    dword6,    dword7\r\n");
    if (f_lseek(&canfd_fileObject, f_size(&canfd_fileObject)))
    {
        PRINTF("Set canfd file pointer position failed. \r\n");

    }

    error = f_write(&canfd_fileObject, g_bufferWrite, strlen(g_bufferWrite), &bytesWritten);
    if ((error) || (bytesWritten != strlen(g_bufferWrite)))
    {
        PRINTF("Write cafdfile failed. \r\n");
    }


//    if (f_close(&canfd_fileObject))
//    {
//        PRINTF("\r\nClose canfdfile failed.\r\n");
//        return -1;
//    }

}

void write_lin_file( uint8_t *writedata, uint8_t len )
{
	FRESULT error;
	UINT bytesWritten;
    error = f_write(&g_fileObject, writedata, len, &bytesWritten);
    if (error)
    {
        PRINTF("Write lin file failed. \r\n");
    }
}
#define SD_MAX_RECORD_NUM   10
uint32_t MCX_record_counter = 0;
//void write_canfd_file( uint8_t *writedata, uint8_t len )
//{
//	FRESULT error;
//	UINT bytesWritten;
//	error = f_write(&canfd_fileObject, writedata, len, &bytesWritten);
//	if (error)
//	{
//		PRINTF("Write canfd file failed. \r\n");
//	}
//}

void write_canfd_file( uint8_t *writedata, uint8_t len )
{
	FRESULT error;
	UINT bytesWritten;
	if(MCX_record_counter < SD_MAX_RECORD_NUM)
	{
		error = f_write(&canfd_fileObject, writedata, len, &bytesWritten);
		if (error)
		{
			PRINTF("Write canfd file failed. \r\n");
		}
		if(MCX_record_counter == (SD_MAX_RECORD_NUM-1))
		{
		    if (f_close(&canfd_fileObject))
		    {
		        PRINTF("\r\nClose canfd file failed.\r\n");
		    }
		    else
		    {
		    	PRINTF("bus record finished \r\n");
		    }
		}
	}
	MCX_record_counter++;
}

void close_lin_file( void )
{
    if (f_close(&g_fileObject))
    {
        PRINTF("\r\nClose file failed.\r\n");

    }
}

void close_canfd_file( void )
{
    if (f_close(&canfd_fileObject))
    {
        PRINTF("\r\nClose file failed.\r\n");
    }
}

/*!
 * @brief Main function
 */
int main(void)
{

    CLOCK_SetClkDiv(kCLOCK_DivFlexcom0Clk, 1u);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

    CLOCK_SetClkDiv(kCLOCK_DivFlexcom2Clk, 1u);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

    BOARD_InitHardware();
    SysTick_config(1000); //1ms
#ifdef LIN_COM0
#ifdef LIN_MASTER
    GPIO_PortSet(BOARD_INITPINS_LIN1_M_GPIO, BOARD_INITPINS_LIN1_M_GPIO_PIN_MASK);
#else
    GPIO_PortClear(BOARD_INITPINS_LIN1_M_GPIO, BOARD_INITPINS_LIN1_M_GPIO_PIN_MASK);
#endif
#endif

#ifdef LIN_COM2
#ifdef LIN_MASTER
    GPIO_PortSet(BOARD_INITPINS_LIN0_M_GPIO, BOARD_INITPINS_LIN0_M_GPIO_PIN_MASK);
#else
    GPIO_PortClear(BOARD_INITPINS_LIN0_M_GPIO, BOARD_INITPINS_LIN0_M_GPIO_PIN_MASK);
#endif
#endif
    lpuart_config_t config;
	uint8_t PID;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 19200;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(DEMO_LIN, &config, DEMO_LIN_CLK_FREQ);
    uart_LIN_break( DEMO_LIN );

//    SD_Fatfs_test();
    sd_savecsv_header();
//    sd_savecanfdcsv_header();
    PRINTF("\r\nStart CANFD Init\r\n");
    MCXN_flexcan_init();
    PRINTF("\r\nStart LIN slave\r\n");
    while (true)
    {
    	MCXN_flexcan_Test();
        if(state == SEND_DATA)
        {
			PID = LINCalcParity(0x2c);
			sendbuffer[2] = LINCheckSum( PID, sendbuffer, 2);
			LPUART_WriteBlocking(DEMO_LIN, sendbuffer, 3);
			sendbuffer[0]++;
			sendbuffer[1]++;
			recdatacnt=0;
			state = IDLE;
			EnableLinBreak;
        }
    }
}

static status_t sdcardWaitCardInsert(void)
{
    BOARD_SD_Config(&g_sd, NULL, BOARD_SDMMC_SD_HOST_IRQ_PRIORITY, NULL);

    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        return kStatus_Fail;
    }

    /* wait card insert */
    if (SD_PollingCardInsert(&g_sd, kSD_Inserted) == kStatus_Success)
    {
        PRINTF("\r\nCard inserted.\r\n");
        /* power off card */
        SD_SetCardPower(&g_sd, false);
        /* power on the card */
        SD_SetCardPower(&g_sd, true);
    }
    else
    {
        PRINTF("\r\nCard detect fail.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}

uint8_t LINCheckSum(uint8_t PID, uint8_t *buf, uint8_t lens)
{
    uint8_t i, ckm = 0;
    uint16_t chm1 = PID;//Enhanced
//    uint16_t chm1 = 0;//Classic
    for(i = 0; i < lens; i++)
    {
        chm1 += *(buf++);
    }
    ckm = chm1 / 256;
    ckm = ckm + chm1 % 256;
    ckm = 0xFF - ckm;

    return ckm;
}

uint8_t LINCalcParity(uint8_t id)
{
    uint8_t parity, p0,p1;
    parity=id;

    p0=((id & 0x01)^((id & 0x02) >> 1)^((id & 0x04) >> 2)^((id & 0x10) >> 4)) << 6;
    p1=(!((id & 0x02) >> 1)^((id & 0x08) >> 3)^((id & 0x10) >> 4)^((id & 0x20) >> 5)) << 7;
    parity|=(p0|p1);

    return parity;
}
