/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:28p $
 * @brief
 *           Configure USCI_SPI1 as Slave mode and demonstrate how to communicate with an off-chip SPI Master device.
 *           This sample code needs to work with USCI_SPI_MasterMode sample code.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC126.h"


#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void USCI_SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main()
{
    uint32_t u32TxDataCount, u32RxDataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    /* Init USCI_SPI1 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           USCI_SPI Slave Mode Sample Code           |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI1 as a slave.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI1:\n");
    printf("    USCI_SPI1_SS (PA.0)\n    USCI_SPI1_CLK (PA.3)\n");
    printf("    USCI_SPI1_MISO (PB.1)\n    USCI_SPI1_MOSI (PB.0)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for(u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32TxDataCount] = 0xAA00 + u32TxDataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.");
    getchar();
    printf("\n");

    /* Access TX and RX Buffer */
    while(u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if(((USPI1->BUFSTS & USPI_BUFSTS_TXFULL_Msk) == 0) && (u32TxDataCount < TEST_COUNT))
            USPI1->TXDAT = g_au32SourceData[u32TxDataCount++]; /* Write to TX Buffer */
        /* Check RX EMPTY flag */
        if((USPI1->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) == 0)
            g_au32DestinationData[u32RxDataCount++] = USPI1->RXDAT; /* Read RX Buffer */
    }

    /* Print the received data */
    printf("Received data:\n");
    for(u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32RxDataCount]);
    }
    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Disable USCI_SPI1 function mode */
    USPI1->CTL &= ~USPI_CTL_FUNMODE_Msk;
    while(1);
}

void SYS_Init(void)
{
	uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12MHz XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Select HXT as the clock source of HCLK */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HXT;

    /* Select HXT as the clock source of UART */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HXT;

    /* Enable 48MHz HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRC48EN_Msk;

    /* Waiting for 48MHz clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRC48STB_Msk));

    /* HCLK Clock source from HIRC48 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC48;

    /* Enable UART clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable USCI1 clock */
    CLK->APBCLK1 = CLK_APBCLK1_USCI1CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

    /* Set PD5 as output mode and PD6 as Input mode */
    PD->MODE = (PD->MODE & 0xFFFFC3FF) | 0x00000400;

    /* Set USCI_SPI1 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_USCI1_CTL0 | SYS_GPA_MFPL_PA3MFP_USCI1_CLK);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_USCI1_DAT0 | SYS_GPB_MFPL_PB1MFP_USCI1_DAT1);

    /* Set PC0,2,3 as output mode and PC1 as Input mode */
    PC->MODE = (PC->MODE & 0xFFFFFF00) | 0x00000051;
}

void UART_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* UART peripheral clock rate 12MHz; UART bit rate 115200 bps. */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
}

void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI1                                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select USCI_SPI1 protocol */
    USPI1->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI1->CTL = 1 << USPI_CTL_FUNMODE_Pos;
    /* Configure USCI_SPI1 as a slave, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI1->PROTCTL = USPI_SLAVE | USPI_MODE_0;
    USPI1->LINECTL = 0;
    /* Set USCI_SPI1 clock rate = f_PCLK1 */
    USPI1->BRGEN = USPI1->BRGEN & (~USPI_BRGEN_CLKDIV_Msk);
    /* Configure USCI_SPI_SS pin as low-active. */
    USPI1->CTLIN0 = (USPI1->CTLIN0 & (~USPI_CTLIN0_ININV_Msk)) | USPI_CTLIN0_ININV_Msk;
    /* Enable USCI_SPI1 protocol */
    USPI1->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
