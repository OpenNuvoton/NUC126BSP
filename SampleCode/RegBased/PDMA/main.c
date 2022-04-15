/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:26p $
 * @brief    Use PDMA channel 2 to transfer data from memory to memory.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

uint32_t PDMA_TEST_LENGTH = 64;
uint8_t au8SrcArray[256];
uint8_t au8DestArray[256];
uint32_t volatile g_u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_NUC126.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 2 has abort error */
        if(PDMA_GET_ABORT_STS() & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;
        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if(PDMA_GET_TD_STS() & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;

    /* IP clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|    PDMA Memory to Memory Driver Sample Code          | \n");
    printf("+------------------------------------------------------+ \n");

    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray                         au8DestArray
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      PDMA transfer configuration:

        Channel = 2
        Operation mode = basic mode
        Request source = PDMA_MEM(memory to memory)
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/

    /* Open Channel 2 */
    PDMA->CHCTL |= (1 << 2);

    /* Transfer configuration of Channel 2 */
    PDMA->DSCT[2].CTL = \
                        ((PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
                        PDMA_WIDTH_32 |  /* Transfer width is 32 bits(one word) */ \
                        PDMA_SAR_INC |   /* Source increment size is 32 bits(one word) */ \
                        PDMA_DAR_INC |   /* Destination increment size is 32 bits(one word) */ \
                        PDMA_REQ_BURST | /* Transfer type is burst transfer type */ \
                        PDMA_BURST_4 |   /* Burst size is 4. No effect in single transfer type */ \
                        PDMA_OP_BASIC;   /* Operation mode is basic mode */

    /* Configure source address */
    PDMA->DSCT[2].SA = (uint32_t)au8SrcArray;

    /* Configure destination address */
    PDMA->DSCT[2].DA = (uint32_t)au8DestArray;

    /* Configure PDMA channel 2 as memory to memory transfer */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC2_Msk) | (PDMA_MEM << PDMA_REQSEL0_3_REQSRC2_Pos);

    /* Enable interrupt */
    PDMA->INTEN |= (1 << 2);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    g_u32IsTestOver = 0;

    /* Generate a software request to trigger transfer with PDMA channel 2  */
    PDMA->SWREQ = (1 << 2);

    /* Waiting for transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            break;
        }
    }

    /* Check transfer result */
    if(g_u32IsTestOver == 1)
        printf("test done...\n");
    else if(g_u32IsTestOver == 2)
        printf("target abort...\n");

    /* Close channel 2 */
    PDMA->CHCTL = 0;

    while(1);
}
