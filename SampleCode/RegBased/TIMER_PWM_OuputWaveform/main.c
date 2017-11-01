/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 17/05/04 1:23p $
 * @brief    Demonstrate output different duty waveform in Timer0~Timer3 PWM.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"


#define PLLCON_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    /* Switch STCLK source to HCLK/2 and HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_HCLK_DIV2 | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable peripheral clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk |
                   CLK_APBCLK0_TMR2CKEN_Msk | CLK_APBCLK0_TMR3CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL | CLK_CLKSEL1_TMR0SEL_PCLK0 | CLK_CLKSEL1_TMR1SEL_PCLK0 |
                   CLK_CLKSEL1_TMR2SEL_PCLK1 | CLK_CLKSEL1_TMR3SEL_PCLK1;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set Timer0 PWM CH0(TM0) and Timer1 PWM CH0(TM1) pin */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD7MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD4MFP_TM0 | SYS_GPD_MFPL_PD7MFP_TM1);
    /* Set Timer2 PWM CH0(TM2) and Timer3 PWM CH3(TM1) pin */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD8MFP_Msk | SYS_GPD_MFPH_PD9MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD8MFP_TM2 | SYS_GPD_MFPH_PD9MFP_TM3);
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
    uint32_t u32Period[4], u32Cmp[4];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------------+\n");
    printf("|    Timer0~Timer3 PWM Output Duty Sample Code    |\n");
    printf("+-----------------------------------------------+\n\n");

    u32Period[0] = 120;
    u32Period[1] = 100;
    u32Period[2] = 400;
    u32Period[3] = 600;
    u32Cmp[0] = (u32Period[0] * 50) / 100; // Set duty 50%
    u32Cmp[1] = (u32Period[1] * 10) / 100; // Set duty 10%
    u32Cmp[2] = (u32Period[2] * 75) / 100; // Set duty 75%
    u32Cmp[3] = (u32Period[3] * 20) / 100; // Set duty 20%
    printf("# Timer0 PWM_CH0 output frequency is %d Hz and duty is 50%%.\n", SystemCoreClock / u32Period[0]);
    printf("# Timer1 PWM_CH0 output frequency is %d Hz and duty is 10%%.\n", SystemCoreClock / u32Period[1]);
    printf("# Timer2 PWM_CH0 output frequency is %d Hz and duty is 75%%.\n", SystemCoreClock / u32Period[2]);
    printf("# Timer3 PWM_CH0 output frequency is %d Hz and duty is 20%%.\n", SystemCoreClock / u32Period[3]);
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PD.4\n");
    printf("    - Timer1 PWM_CH0 on PD.7\n");
    printf("    - Timer2 PWM_CH0 on PD.8\n");
    printf("    - Timer3 PWM_CH0 on PD.9\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);
    TPWM_ENABLE_PWM_MODE(TIMER1);
    TPWM_ENABLE_PWM_MODE(TIMER2);
    TPWM_ENABLE_PWM_MODE(TIMER3);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER1);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER2);
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER3);

    /* Set PWM up count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER1, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER2, TPWM_UP_COUNT);
    TPWM_SET_COUNTER_TYPE(TIMER3, TPWM_UP_COUNT);

    /* Set PWM period */
    TPWM_SET_PERIOD(TIMER0, (u32Period[0] - 1));
    TPWM_SET_PERIOD(TIMER1, (u32Period[1] - 1));
    TPWM_SET_PERIOD(TIMER2, (u32Period[2] - 1));
    TPWM_SET_PERIOD(TIMER3, (u32Period[3] - 1));

    /* Set PWM compare to create duty cycle */
    TPWM_SET_CMPDAT(TIMER0, u32Cmp[0]);
    TPWM_SET_CMPDAT(TIMER1, u32Cmp[1]);
    TPWM_SET_CMPDAT(TIMER2, u32Cmp[2]);
    TPWM_SET_CMPDAT(TIMER3, u32Cmp[3]);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER1, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER2, TPWM_CH0);
    TPWM_ENABLE_OUTPUT(TIMER3, TPWM_CH0);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);
    TPWM_START_COUNTER(TIMER1);
    TPWM_START_COUNTER(TIMER2);
    TPWM_START_COUNTER(TIMER3);

    printf("*** Check Timer0~Timer3 PWM_CH0 output waveform by oscilloscope ***\n");

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
