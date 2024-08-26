/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:28p $
 * @brief
 *          Configure USCI_SPI1 as Master mode and demonstrate how to communicate with an off-chip SPI Slave device.
 *          Needs to work with USCI_SPI_SlaveMode sample code.
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
    uint32_t u32DataCount;

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
    printf("+--------------------------------------------------------+\n");
    printf("|             USCI_SPI Master Mode Sample Code           |\n");
    printf("+--------------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI1 as a master.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI1:\n");
    printf("    USCI_SPI1_SS (PA.0)\n    USCI_SPI1_CLK (PA.3)\n");
    printf("    USCI_SPI1_MISO (PB.1)\n    USCI_SPI1_MOSI (PB.0)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The USCI_SPI master configuration is ready.\n");

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x5500 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.");
    getchar();
    printf("\n");

    /* Enable TX end interrupt */
    USPI1->INTEN |= USPI_INTEN_TXENDIEN_Msk;
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(USCI_IRQn);

    /* Write to TX Buffer */
    USPI1->TXDAT = g_au32SourceData[g_u32TxDataCount++];

    /* Wait for transfer done */
    while(g_u32RxDataCount < TEST_COUNT);

    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
    /* Disable TX end interrupt */
    USPI1->INTEN &= ~USPI_INTEN_TXENDIEN_Msk;
    NVIC_DisableIRQ(USCI_IRQn);
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
    /* Configure USCI_SPI1 as a master, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI1->PROTCTL = USPI_MASTER | USPI_MODE_0;
    USPI1->LINECTL = 0;
    /* Set USCI_SPI1 clock rate = f_PCLK1 / 2*(5+1) */
    USPI1->BRGEN = (USPI1->BRGEN & (~USPI_BRGEN_CLKDIV_Msk)) | (5 << USPI_BRGEN_CLKDIV_Pos);
    /* Enable the automatic hardware slave selection function and configure USCI_SPI_SS pin as low-active. */
    USPI1->LINECTL = (USPI1->LINECTL & (~USPI_LINECTL_CTLOINV_Msk)) | USPI_SS_ACTIVE_LOW;
    USPI1->PROTCTL |= USPI_PROTCTL_AUTOSS_Msk;
    /* Enable USCI_SPI1 protocol */
    USPI1->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;
}

void USCI_IRQHandler(void)
{
    uint32_t u32RxData;

    /* Clear TX end interrupt flag */
    USPI1->PROTSTS = USPI_PROTSTS_TXENDIF_Msk;

    /* Check RX EMPTY flag */
    while((USPI1->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) == 0)
    {
        /* Read RX Buffer */
        u32RxData = USPI1->RXDAT;
        g_au32DestinationData[g_u32RxDataCount++] = u32RxData;
    }
    /* Check TX data count */
    if(g_u32TxDataCount < TEST_COUNT)
    {
        /* Write to TX Buffer */
        USPI1->TXDAT = g_au32SourceData[g_u32TxDataCount++];
    }
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
