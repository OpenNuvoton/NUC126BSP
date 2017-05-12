/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 17/05/04 1:22p $
 * @brief    Change duty cycle and period of output waveform in PWM down count type.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"


#define PLLCON_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_NUC126.s.
 */
void TMR0_IRQHandler(void)
{
    static uint32_t u32Toggle = 0;

    if(TPWM_GET_PERIOD_INT_FLAG(TIMER0))
    {
        if(u32Toggle == 0)
        {
            /* Set PWM period, (100 * TMRx_PWMCLK) */
            TPWM_SET_PERIOD(TIMER0, (100 - 1));

            /* Set PWM duty, (40 * TMRx_PWMCLK) */
            TPWM_SET_CMPDAT(TIMER0, 40);
        }
        else
        {
            /* Set PWM period, (400 * TMRx_PWMCLK) */
            TPWM_SET_PERIOD(TIMER0, (400 - 1));

            /* Set PWM duty, (200 * TMRx_PWMCLK) */
            TPWM_SET_CMPDAT(TIMER0, 200);
        }
        u32Toggle ^= 1;
        TPWM_CLEAR_PERIOD_INT_FLAG(TIMER0);
    }
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
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL | CLK_CLKSEL1_TMR0SEL_PCLK0 | CLK_CLKSEL1_TMR1SEL_PCLK0;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

    /* Set Timer0 PWM CH0(TM0) pins */
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD4MFP_TM0;
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
    printf("|    Timer PWM Change Duty Cycle Sample Code    |\n");
    printf("+-----------------------------------------------+\n\n");

    printf("# Timer0 PWM counter configure: \n");
    printf("    - Counter clock source:     PCLK0 \n");
    printf("    - Counter clock prescaler:  1 \n");
    printf("    - Counter type:             Down count type \n");
    printf("    - Operation mode:           Independent in auto-reload mode \n");
    printf("    - Initial period value:     %d \n", 400);
    printf("    - Initial comparator value: %d \n", 200);
    printf("    - Updated period value:     %d \n", 100);
    printf("    - Updated comparator value: %d \n", 40);
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PD.4\n\n");
    printf("        Timer0 PWM_CH0 waveform of this sample shown below:                             \n");
    printf("\n");
    printf("        |<-        PERIOD0+1        ->|                     PERIOD0 + 1 = (399 + 1) CLKs\n");
    printf("                       |<-   CMP0   ->|                     CMP0        = 200 CLKs      \n");
    printf("                                      |<-  PERIOD1+1 ->|    PERIOD1 + 1 = (99 + 1) CLKs \n");
    printf("                                               |<CMP1 >|    CMP1        = 40 CLKs       \n");
    printf("      __                ______________          _______                                 \n");
    printf("        |______200_____|     200      |____60__|   40  |___ PWM_CH0 outut waveform      \n");
    printf("\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);

    /* Set PWM down count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_DOWN_COUNT);

    /* Set PWM Timer clock prescaler*/
    TPWM_SET_PRESCALER(TIMER0, 0); // Divided by 1

    /* Set PWM period, (400 * TMRx_PWMCLK) */
    TPWM_SET_PERIOD(TIMER0, (400 - 1));

    /* Set PWM duty, (200 * TMRx_PWMCLK) */
    TPWM_SET_CMPDAT(TIMER0, 200);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable period event interrupt */
    TPWM_ENABLE_PERIOD_INT(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer PWM counter */
    TIMER0->PWMCTL |= TIMER_PWMCTL_CNTEN_Msk;

    printf("*** Check Timer0 PWM_CH0 output waveform by oscilloscope ***\n");

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
