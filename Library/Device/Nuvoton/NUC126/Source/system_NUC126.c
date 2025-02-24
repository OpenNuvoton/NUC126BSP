/**************************************************************************//**
 * @file     system_NUC126.c
 * @version  V2.00
 * $Revision: 6 $
 * $Date: 16/10/25 4:25p $
 * @brief    NUC126 Series System Setting Source File
 *
 * @note
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "NUC126.h"



/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock  = __HSI;              /*!< System Clock Frequency (Core Clock) */
uint32_t CyclesPerUs      = (__HSI / 1000000);  /*!< Cycles per micro second             */
uint32_t PllClock         = __HSI;              /*!< PLL Output Clock Frequency          */
const uint32_t gau32ClkSrcTbl[] = {__HXT, __LXT, __HSI, __LIRC, __HIRC48, 0UL, 0UL, __HIRC};


/**
 * @brief    Update the Variable SystemCoreClock
 *
 * @param    None
 *
 * @return   None
 *
 * @details  This function is used to update the variable SystemCoreClock
 *           and must be called whenever the core clock is changed.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t u32Freq, u32ClkSrc;
    uint32_t u32HclkDiv;

    u32ClkSrc = CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk;

    /* Update PLL Clock */
    PllClock = CLK_GetPLLClockFreq();

    if(u32ClkSrc != CLK_CLKSEL0_HCLKSEL_PLL)
    {
        /* Use the clock sources directly */
        u32Freq = gau32ClkSrcTbl[u32ClkSrc];
    }
    else
    {
        /* Use PLL clock */
        u32Freq = PllClock;
    }

    u32HclkDiv = (CLK->CLKDIV0 & CLK_CLKDIV0_HCLKDIV_Msk) + 1;

    /* Update System Core Clock */
    SystemCoreClock = u32Freq / u32HclkDiv;

    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
}


/**
 * @brief    System Initialization
 *
 * @param    None
 *
 * @return   None
 *
 * @details  The necessary initialization of system. Global variables are forbidden here.
 */
void SystemInit(void)
{
#ifdef INIT_SYSCLK_AT_BOOTING
    int32_t i32TimeoutCnt;
    uint32_t u32HclkSelect;
    int8_t i8IsPllEn;

    PllClock = 0;
    i8IsPllEn = 0;
    u32HclkSelect = CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk;
    if(u32HclkSelect == CLK_CLKSEL0_HCLKSEL_HXT)
    {
        /* Set to 72MHz system clock frequency when clock source is from external 12MHz */
        CLK->PLLCTL = CLK_PLLCTL_72MHz_HXT;

        /* Waiting for PLL ready */
        i32TimeoutCnt = (__HXT / 1000); /* Timeout is about 1ms */
        while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0)
        {
            if(i32TimeoutCnt-- <= 0)
                break;
        }

        i8IsPllEn = 1;
    }
    else if(u32HclkSelect == CLK_CLKSEL0_HCLKSEL_HIRC)
    {
        /* Set to 71.8848MHz system clock frequency when clock source is from internal 22.1184MHz RC clock */
        CLK->PLLCTL = CLK_PLLCTL_72MHz_HIRC;

        /* Waiting for PLL ready */
        i32TimeoutCnt = (__HIRC / 1000); /* Timeout is about 1ms */
        while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0)
        {
            if(i32TimeoutCnt-- <= 0)
                break;
        }

        i8IsPllEn = 1;
    }

    if(i8IsPllEn)
    {
        /* Set PLL as HCLK clock source (HCLK_S is locked setting)*/
        SYS_UnlockReg();
        CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;
        SYS_LockReg();
    }
#endif
}

#if USE_ASSERT

/**
 * @brief      Assert Error Message
 *
 * @param[in]  file  the source file name
 * @param[in]  line  line number
 *
 * @return     None
 *
 * @details    The function prints the source file name and line number where
 *             the ASSERT_PARAM() error occurs, and then stops in an infinite loop.
 */
void AssertError(uint8_t * file, uint32_t line)
{

    printf("[%s] line %d : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    while(1) ;
}
#endif
