/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 17/05/04 1:22p $
 * @brief    Demonstrate Timer PWM Complementary mode and Dead-Time function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"


#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


void SYS_Init(void)
{
	uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Switch STCLK source to HCLK/2 and HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_HCLK_DIV2 | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable peripheral clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL | CLK_CLKSEL1_TMR0SEL_PCLK0 | CLK_CLKSEL1_TMR1SEL_PCLK0;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set Timer0 PWM CH0(TM0) and CH1(TM0_EXT) pins */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD4MFP_TM0 | SYS_GPD_MFPL_PD2MFP_TM0_EXT);

    /* Set Timer1 PWM CH0(TM1) and CH1(TM1_EXT) pins */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD5MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD5MFP_TM1 | SYS_GPD_MFPL_PD3MFP_TM1_EXT);
}

void UART0_Init(void)
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
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32CntClkFreq, u32Period, u32CMP, u32Prescaler, u32DeadTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------------------------+\n");
    printf("|    Timer PWM Complementary mode and Dead-Time Sample Code    |\n");
    printf("+--------------------------------------------------------------+\n\n");

    u32CntClkFreq = SystemCoreClock;
    u32Prescaler = 2;                                   // Set counter clock prescaler 2
    u32Period = ((u32CntClkFreq / u32Prescaler) / 6000);
    u32CMP = (40 * u32Period) / 100;                    // Duty ratio is (u32CMP / u32Period) in up count type */
    u32DeadTime = u32CMP / 2;                           // Deat-Time interval is (HighDuty / 2)

    /* Configure Timer0 PWM */
    printf("# Timer0 PWM output frequency is 6000 Hz and duty 40%%.\n");
    printf("    - Counter clock source:    PCLK0 \n");
    printf("    - Counter clock prescaler: 2 \n");
    printf("    - Counter type:            Up count type \n");
    printf("    - Operation mode:          Complementary in auto-reload mode \n");
    printf("    - Period value:            %d \n", u32Period);
    printf("    - Comparator value:        %d \n", u32CMP);
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PD.4, PWM_CH1 on PD.2\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set PWM mode as complementary mode*/
    TPWM_ENABLE_COMPLEMENTARY_MODE(TIMER0);

    /* Set PWM Timer clock prescaler*/
    TPWM_SET_PRESCALER(TIMER0, (u32Prescaler - 1)); // Divided by 2

    /* Set PWM Timer period*/
    TPWM_SET_PERIOD(TIMER0, (u32Period - 1));

    /* Set PWM Timer duty*/
    TPWM_SET_CMPDAT(TIMER0, u32CMP);

    /* Enable output of PWM_CH0 and PWM_CH1 */
    TPWM_ENABLE_OUTPUT(TIMER0, (TPWM_CH1 | TPWM_CH0));


    /* Configure Timer1 PWM */
    printf("# Timer1 PWM output frequency is 6000 Hz and duty 20%% with dead-time insertion.\n");
    printf("    - Counter clock source:    PCLK0 \n");
    printf("    - Counter clock prescaler: 2 \n");
    printf("    - Counter type:            Up count type \n");
    printf("    - Operation mode:          Complementary in auto-reload mode \n");
    printf("    - Period value:            %d \n", u32Period);
    printf("    - Comparator value:        %d \n", u32CMP);
    printf("    - Deat-Time interval:      %d \n", u32DeadTime);
    printf("# I/O configuration:\n");
    printf("    - Timer1 PWM_CH0 on PD.5, PWM_CH1 on PD.3\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER1);

    /* Set PWM mode as complementary mode*/
    TPWM_ENABLE_COMPLEMENTARY_MODE(TIMER1);

    /* Set PWM Timer clock prescaler*/
    TPWM_SET_PRESCALER(TIMER1, (u32Prescaler - 1)); // Divided by 2

    /* Set PWM Timer period*/
    TPWM_SET_PERIOD(TIMER1, (u32Period - 1));

    /* Set PWM Timer duty*/
    TPWM_SET_CMPDAT(TIMER1, u32CMP);

    /* Enable output of PWM_CH0 and PWM_CH1 */
    TPWM_ENABLE_OUTPUT(TIMER1, (TPWM_CH1 | TPWM_CH0));

    /* Enable and configure dead-time interval is (u32DeadTime * TMR1_PWMCLK * prescaler) */
    SYS_UnlockReg(); // Unlock protected registers
    TIMER1->PWMDTCTL = TIMER_PWMDTCTL_DTCKSEL_Msk | TIMER_PWMDTCTL_DTEN_Msk | (u32DeadTime - 1);
    SYS_LockReg(); // Lock protected registers

    printf("*** Check Timer0 and Timer1 PWM output waveform by oscilloscope ***\n");

    /* Start Timer0 and Timer1 PWM counter by trigger Timer0 sync. start */
    TPWM_SET_COUNTER_SYNC_MODE(TIMER0, TPWM_COUNTER_SYNC_START_BY_TIMER0);
    TPWM_SET_COUNTER_SYNC_MODE(TIMER1, TPWM_COUNTER_SYNC_START_BY_TIMER0);
    TPWM_TRIGGER_COUNTER_SYNC(TIMER0);

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
