/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:34p $
 * @brief
 *           Show how to wake up system from Power-down mode by USCI interrupt in UART mode.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NUC126.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void USCI_UART_DataWakeUp(void);
void USCI_UART_CTSWakeUp(void);
void USCI_UART_PowerDown_TestItem(void);
void USCI_UART_PowerDownWakeUpTest(void);


void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART0)
        if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART and USCI module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.0) and TXD(PD.1) */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD0MFP_Msk)) | SYS_GPD_MFPL_PD0MFP_UART0_RXD;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD1MFP_Msk)) | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

    /* Set PC multi-function pins for USCI0_DAT0(PC.0), USCI0_DAT1(PC.1) and USCI0_CTL0(PC.3) */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk)) | SYS_GPC_MFPL_PC0MFP_USCI0_DAT0;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk)) | SYS_GPC_MFPL_PC1MFP_USCI0_DAT1;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk)) | SYS_GPC_MFPL_PC3MFP_USCI0_CTL0;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(USCI0_RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 9600);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init USCI0 for test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART Power-down and Wake-up sample function */
    USCI_UART_PowerDownWakeUpTest();

    printf("USCI UART Sample Program End.\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_IRQHandler(void)
{
    uint32_t u32IntSts = UUART_GET_PROT_STATUS(UUART0);
    uint32_t u32WkSts = UUART_GET_WAKEUP_FLAG(UUART0);

    if(u32WkSts & UUART_WKSTS_WKF_Msk)  /* USCI UART wake-up flag */
    {
        UUART_CLR_WAKEUP_FLAG(UUART0);
        printf("USCI UART wake-up.\n");
    }
    else if(u32IntSts & UUART_PROTSTS_RXENDIF_Msk)  /* USCI UART receive end interrupt flag */
    {
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        while(UUART_GET_RX_EMPTY(UUART0) == 0)
        {
            printf("Data: 0x%X\n", UUART_READ(UUART0));
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_CTSWakeUp(void)
{
    /* Enable UART nCTS wake-up function */
    UUART_EnableWakeup(UUART0, UUART_PROTCTL_CTSWKEN_Msk);

    printf("System enter to Power-down mode.\n");
    printf("Toggle USCI-UART0 nCTS to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_DataWakeUp(void)
{
    /* Enable UART data wake-up function */
    UUART_EnableWakeup(UUART0, UUART_PROTCTL_DATWKEN_Msk);

    /* Set UART data wake-up counter.
       It indicates how many clock cycle does UART can get the first bit (start bit)
       when the device is wake-up from Power-down mode.
       Clock is (USCI clock source)/((CLKDIV+1)(PDSCNT+1)) */
    UUART0->PROTCTL = (UUART0->PROTCTL & (~UUART_PROTCTL_WAKECNT_Msk)) | (2 << UUART_PROTCTL_WAKECNT_Pos);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to USCI-UART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI UART Power-down and wake-up test                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                           - [Others] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    /* Enable UART receive end interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_EnableIRQ(USCI_IRQn);

    USCI_UART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);
    switch(u32Item)
    {
        case '1':
            USCI_UART_CTSWakeUp();
            break;
        case '2':
            USCI_UART_DataWakeUp();
            break;
        default:
            return;
    }

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Wait until USCI UART data transmission is finished */
    while(UUART0->PROTSTS & UUART_PROTSTS_RXBUSY_Msk);

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n\n");
    getchar();

    /* Disable UART wake-up function */
    UUART_DisableWakeup(UUART0);

    /* Disable UART receive end interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_DisableIRQ(USCI_IRQn);

}
