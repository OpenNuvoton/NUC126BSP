/**************************************************************************//**
 * @file     main.c
 * @version  V2.0
 * $Revision: 2 $
 * $Date: 16/10/25 4:27p $
 * @brief    Show Smartcard UART by connecting PA.0 and PA.1 pins.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"


#define PLLCON_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


uint8_t au8TxBuf[] = "Hello World!";

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of Smart Card                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void SC01_IRQHandler(void)
{
    // Print SCUART received data to UART port
    // Data length here is short, so we're not care about UART FIFO over flow.
    UART_WRITE(UART0, SCUART_READ(SC0));

    // RDA is the only interrupt enabled in this sample, this status bit
    // automatically cleared after Rx FIFO empty. So no need to clear interrupt
    // status here.

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT and LXT-32KHz */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    /* Switch STCLK source to HCLK/2 and HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_HCLK_DIV2 | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL;

    /* Enable SC0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_SC0CKEN_Msk;

    /* Select SC0 module clock source from PLL */
    CLK->CLKSEL3 &= ~CLK_CLKSEL3_SC0SEL_Msk;
    CLK->CLKSEL3 |= CLK_CLKSEL3_SC0SEL_PLL;
    CLK->CLKDIV1 &= ~CLK_CLKDIV1_SC0DIV_Msk;
    CLK->CLKDIV1 |= CLK_CLKDIV1_SC0(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

    /* Set PA multi-function pins for SC UART mode */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK | SYS_GPA_MFPL_PA1MFP_SC0_DAT);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Enable SC UART interface                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate)
{
    uint32_t u32ClkSrc, u32Clk, u32Div;

    u32ClkSrc = (CLK->CLKSEL3 & CLK_CLKSEL3_SC0SEL_Msk) >> CLK_CLKSEL3_SC0SEL_Pos;

    if(u32ClkSrc == 0)
        u32Clk = __HXT;
    else if(u32ClkSrc == 1)
        u32Clk = CLK_GetPLLClockFreq();
    else
        u32Clk = __HIRC;

    u32Clk /= ((CLK->CLKDIV1 & CLK_CLKDIV1_SC0DIV_Msk) + 1);

    /* Calculate divider for target baudrate */
    u32Div = (u32Clk + (u32baudrate >> 1) - 1) / u32baudrate - 1;

    sc->CTL = SC_CTL_SCEN_Msk | SC_CTL_NSB_Msk;  // Enable smartcard interface and stop bit = 1
    sc->UARTCTL = SCUART_CHAR_LEN_8 | SCUART_PARITY_NONE | SC_UARTCTL_UARTEN_Msk; // Enable UART mode, disable parity and 8 bit per character
    sc->ETUCTL = u32Div;

    return(u32Clk / u32Div);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Write data to SC UART interface                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void SCUART_Write(SC_T* sc, uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;

    for(u32Count = 0; u32Count != u32WriteBytes; u32Count++)
    {
        while(SCUART_GET_TX_FULL(sc));  // Wait 'til FIFO not full
        sc->DAT = pu8TxBuf[u32Count];   // Write 1 byte to FIFO
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    Smart Card UART Mode Sample Code    |\n");
    printf("+----------------------------------------+\n\n");
    printf("# Please connect SC0 CLK pin(PA.0) with SC0 DAT pin(PA.1) first.\n");
    printf("    - PA.0 as UART Tx\n");
    printf("    - PA.1 as UART Rx\n");
    printf("# Check check UART message ... Is Hello World! ?\n\n");

    /* Open smartcard interface 0 in UART mode */
    SCUART_Open(SC0, 115200);

    /* Enable receive interrupt */
    SCUART_ENABLE_INT(SC0, SC_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(SC01_IRQn);

    /* Write data to SC UART interface */
    SCUART_Write(SC0, au8TxBuf, sizeof(au8TxBuf));

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
