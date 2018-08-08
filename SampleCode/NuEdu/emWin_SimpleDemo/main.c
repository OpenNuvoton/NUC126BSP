/****************************************************************************
 * @file     main.c
 * @version  V2.00
 * @brief    To utilize emWin library to demonstrate interactive feature.
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NUC126.h"

#include "NUC126TouchPanel.h"

#include "GUI.h"
#include "FRAMEWIN.h"
#include "WM.h"

extern volatile GUI_TIMER_TIME OS_TimeMS;

volatile int g_enable_Touch;

extern int ts_writefile(void);
extern int ts_readfile(void);
int ts_calibrate(int xsize, int ysize);
void ts_test(int xsize, int ysize);

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _SYS_Init
*/
void _SYS_Init(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

#ifndef CRYSTAL_LESS
    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Use HIRC as UART clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
#else
    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC48, CLK_CLKDIV0_HCLK(1));

    /* Use HIRC as UART clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
#endif

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_UART0_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART0_TXD;

    SYS_LockReg();
}

/*********************************************************************
*
*       TMR0_IRQHandler
*/
void TMR0_IRQHandler(void)
{
    int Key;
    OS_TimeMS++;
#if GUI_SUPPORT_TOUCH
    if(OS_TimeMS % 10 == 0)
    {
        if(g_enable_Touch == 1)
        {
            GUI_TOUCH_Exec();
        }
    }
#endif
    if((g_enable_Touch == 1) && (OS_TimeMS % 200 == 0))
    {
        if(PB14 == 0)
        {
            Key = GUI_KEY_TAB;
            GUI_StoreKeyMsg(Key, 1);
        }
        if((PB13 == 0) || (PC9 == 0))
        {
            Key = GUI_KEY_ENTER;
            GUI_StoreKeyMsg(Key, 1);
        }
        if(PC12 == 0)
        {
            Key = GUI_KEY_UP;
            GUI_StoreKeyMsg(Key, 1);
        }
        if(PC10 == 0)
        {
            Key = GUI_KEY_DOWN;
            GUI_StoreKeyMsg(Key, 1);
        }
        if(PC11 == 0)
        {
            Key = GUI_KEY_LEFT;
            GUI_StoreKeyMsg(Key, 1);
        }
        if(PC13 == 0)
        {
            Key = GUI_KEY_RIGHT;
            GUI_StoreKeyMsg(Key, 1);
        }
    }

    TIMER_ClearIntFlag(TIMER0);
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

WM_HWIN CreateFramewin(void);
void MainTask(void)
{
    extern GUI_CONST_STORAGE GUI_BITMAP bmnuvoton_logo;    

    WM_HWIN hWin;
    char     acVersion[40] = "Nuvoton NUC126";

    GUI_Init();
    
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
#ifdef __DEMO_160x128__
    GUI_DrawBitmap(&bmnuvoton_logo, 1, 55);
#else
    GUI_DrawBitmap(&bmnuvoton_logo, (320 - bmnuvoton_logo.XSize) >> 1, (240 - bmnuvoton_logo.YSize) >> 1);
#endif
    GUI_Delay(3000);
    GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
    
    hWin = CreateFramewin();
    FRAMEWIN_SetText(hWin, acVersion);
    while(1)
    {
        GUI_Delay(1000);
    }
}

/*********************************************************************
*
*       main
*/
int main(void)
{
    //
    // Init System, IP clock and multi-function I/O
    //
    _SYS_Init();
    //
    // Init UART to 115200-8n1 for print message
    //
    UART_Open(UART0, 115200);

    g_enable_Touch = 0;
    //
    // Enable Timer0 clock and select Timer0 clock source
    //
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    //
    // Initial Timer0 to periodic mode with 1000Hz
    //
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    //
    // Enable Timer0 interrupt
    //
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TMR0_IRQn, 1);
    NVIC_EnableIRQ(TMR0_IRQn);
    //
    // Start Timer0
    //
    TIMER_Start(TIMER0);
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    GPIO_SetMode(PB, BIT13, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT14, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT9, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT12, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT10, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT11, GPIO_MODE_INPUT);
    GPIO_SetMode(PC, BIT13, GPIO_MODE_INPUT);

#if GUI_SUPPORT_TOUCH
    GUI_Init();

    Init_TouchPanel();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* SPI flash 192KB + 0x1C marker address */
    if (FMC_Read(__DEMO_TSFILE_ADDR__ + 0x1C) != 0x55AAA55A)
    {
        FMC_EnableAPUpdate();
        ts_calibrate(__DEMO_TS_WIDTH__, __DEMO_TS_HEIGHT__);
        // Erase page
        FMC_Erase(__DEMO_TSFILE_ADDR__);
        ts_writefile();
        FMC_DisableAPUpdate();
    }
    else
    {
        ts_readfile();
    }

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    g_enable_Touch = 1;

//    ts_test(__DEMO_TS_WIDTH__, __DEMO_TS_HEIGHT__);
#endif

    g_enable_Touch = 1;

    //
    // Start application
    //
    MainTask();
    while(1);
}

/*************************** End of file ****************************/
