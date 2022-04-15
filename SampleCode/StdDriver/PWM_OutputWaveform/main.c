/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:29p $
 * @brief    Demonstrate how to use PWM counter output waveform.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK       144000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Enable PWM0 and PWM1 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_EnableModuleClock(PWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_PCLK0, NULL);
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL1_PWM1SEL_PCLK1, NULL);

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_PLL, NULL);
    //CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL1_PWM1SEL_PLL, NULL);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Reset PWM0 and PWM1 module */
    SYS_ResetModule(PWM0_RST);
    SYS_ResetModule(PWM1_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PC multi-function pins for PWM0 Channel0~5 */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk |
                       SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk |
                       SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_PWM0_CH0 | SYS_GPC_MFPL_PC1MFP_PWM0_CH1 |
                      SYS_GPC_MFPL_PC2MFP_PWM0_CH2 | SYS_GPC_MFPL_PC3MFP_PWM0_CH3 |
                      SYS_GPC_MFPL_PC4MFP_PWM0_CH4 | SYS_GPC_MFPL_PC5MFP_PWM0_CH5);

    /* Set PA and PC multi-function pins for PWM1 Channel 0~5 */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC6MFP_PWM1_CH0 | SYS_GPC_MFPL_PC7MFP_PWM1_CH1);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk |
                       SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_PWM1_CH2 | SYS_GPA_MFPL_PA2MFP_PWM1_CH3 |
                      SYS_GPA_MFPL_PA1MFP_PWM1_CH4 | SYS_GPA_MFPL_PA0MFP_PWM1_CH5);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL1 & CLK_CLKSEL1_PWM0SEL_Msk) ? "CPU" : "PLL");
    printf("PWM1 clock is from %s\n", (CLK->CLKSEL1 & CLK_CLKSEL1_PWM1SEL_Msk) ? "CPU" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with PWM0 and PWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("  PWM0 channel 0: 360000 Hz, duty 90%%.\n");
    printf("  PWM0 channel 1: 320000 Hz, duty 80%%.\n");
    printf("  PWM0 channel 2: 250000 Hz, duty 75%%.\n");
    printf("  PWM0 channel 3: 180000 Hz, duty 70%%.\n");
    printf("  PWM0 channel 4: 160000 Hz, duty 60%%.\n");
    printf("  PWM0 channel 5: 150000 Hz, duty 50%%.\n");
    printf("  PWM1 channel 0: 120000 Hz, duty 50%%.\n");
    printf("  PWM1 channel 1: 100000 Hz, duty 40%%.\n");
    printf("  PWM1 channel 2:  90000 Hz, duty 30%%.\n");
    printf("  PWM1 channel 3:  60000 Hz, duty 25%%.\n");
    printf("  PWM1 channel 4:  45000 Hz, duty 20%%.\n");
    printf("  PWM1 channel 5:  30000 Hz, duty 10%%.\n");
    printf("    waveform output pin: PWM0_CH0(PC.0), PWM0_CH1(PC.1), PWM0_CH2(PC.2), PWM0_CH3(PC.3), PWM0_CH4(PC.4), PWM0_CH5(PC.5)\n");
    printf("                         PWM1_CH0(PC.6), PWM1_CH1(PC.7), PWM1_CH2(PA.3), PWM1_CH3(PA.2), PWM1_CH4(PA.1), PWM1_CH5(PA.0)\n");


    /* PWM0 and PWM1 channel 0~5 frequency and duty configuration are as follows */
    PWM_ConfigOutputChannel(PWM0, 0, 360000, 90);
    PWM_ConfigOutputChannel(PWM0, 1, 320000, 80);
    PWM_ConfigOutputChannel(PWM0, 2, 250000, 75);
    PWM_ConfigOutputChannel(PWM0, 3, 180000, 70);
    PWM_ConfigOutputChannel(PWM0, 4, 160000, 60);
    PWM_ConfigOutputChannel(PWM0, 5, 150000, 50);
    PWM_ConfigOutputChannel(PWM1, 0, 120000, 50);
    PWM_ConfigOutputChannel(PWM1, 1, 100000, 40);
    PWM_ConfigOutputChannel(PWM1, 2, 90000, 30);
    PWM_ConfigOutputChannel(PWM1, 3, 60000, 25);
    PWM_ConfigOutputChannel(PWM1, 4, 45000, 20);
    PWM_ConfigOutputChannel(PWM1, 5, 30000, 10);

    /* Enable output of PWM0 and PWM1 channel 0~5 */
    PWM_EnableOutput(PWM0, 0x3F);
    PWM_EnableOutput(PWM1, 0x3F);

    /* Start PWM0 counter */
    PWM_Start(PWM0, 0x3F);
    /* Start PWM1 counter */
    PWM_Start(PWM1, 0x3F);

    printf("Press any key to stop.\n");
    getchar();

    /* Start PWM0 counter */
    PWM_ForceStop(PWM0, 0x3F);
    /* Start PWM1 counter */
    PWM_ForceStop(PWM1, 0x3F);

    printf("Done.");
    while(1);

}
