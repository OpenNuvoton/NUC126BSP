/******************************************************************************
 * @file     main.c
 * @brief
 *           This sample code demonstrates how to implement a USB audio class device.
 *           NAU8822 is used in this sample code to play the audio data from Host.
 *           It also supports to record data from NAU8822 to Host.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"
#include "usbd_audio.h"


#define HIRC48_AUTO_TRIM    0x412   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x118)

extern volatile uint32_t g_u32MasterSlave;
extern volatile uint32_t g_u32Master;
/*--------------------------------------------------------------------------*/

void EnableCLKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    /* CLKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv;

    /* Enable CLKO clock source */
    CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

    /* Select CLKO clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_CLKOSEL_Msk)) | u32ClkSrc;
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    
    /* Enable Internal RC 48 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

    /* Waiting for Internal RC48 clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

    /* Set core clock */
    if(g_u32Master)
    {
        CLK->PLLCTL = CLK_PLLCTL_PLLSRC_Msk | 0x0e30; // 122880000Hz
        CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);
        CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;
        
        SystemCoreClock = 61440000;
        CyclesPerUs = SystemCoreClock/1000000;
        PllClock = 122880000;
        
    }
    else
    {
        CLK_SetCoreClock(72000000);
        CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;
    }

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
    if(g_u32Master)
    {
        CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC48, CLK_CLKDIV0_USB(1));
    }
    else
    {
        //CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(3));
        CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC48, CLK_CLKDIV0_USB(1));
    }
    
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(I2C0_MODULE, 0, 0);
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PLL, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPD multi-function pins for UART0 RXD and TXD, and Clock Output */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD | SYS_GPD_MFPL_PD5MFP_CLKO);

    /* Set GPA2, GPA3 to be I2C */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA2MFP_I2C0_SDA | SYS_GPA_MFPL_PA3MFP_I2C0_SCL;

    /* Set I2S interface */
    /* Configure PC.5 as I2S MCLK pin */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_SPI0_I2SMCLK;

//    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~0x000FFF0F)) | 0x00022202; /* PC[4:2, 0] : SPI0_MISO0, SPI0_MOSI0, SPI0_SS, SPI0_CLK */

    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~0x00FFFF00)) | 0x00222200; /* PB[5:2] : SPI0_MOSI0, SPI0_SS, SPI0_MISO0, SPI0_CLK */
    PB->SLEWCTL |= 0x3C;

    /* Enable CLKO(PD5) for monitor HCLK. CLKO = HIRC/64 Hz */
    EnableCLKO((3 << CLK_CLKSEL2_CLKOSEL_Pos), 5);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100 kHz */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;
    uint32_t u32TrimInit;

    /*
        This sample code is used to demo USB Audio Class + NAU8822.
        User can define PLAY_RATE in usbd_audio.h to support 48000Hz.

        The audio is input from NAU8822 AUXIN.
        The audio is output by NAU8822 Headphone output.

        NAU8822 is connect with I2S(PB2~5) and controlled by I2C0(PA2, PA3).
        NAU8822 clock source is also come from I2S (MCLK, PC5).
    
            PA2 <-> I2S0_SDA
            PA3 <-> I2S0_SCL

            PB2 <-> I2S0_BCLK
            PB3 <-> I2S0_ADC
            PB4 <-> I2S0_LRCLK
            PB5 <-> I2S0_DAC

            PC5 <-> I2S_MCLK
    
        PD5 is used to output clock (HCLK/8) to check HCLK frequency.
    
        Clock config (I2S Master Mode):
            PLL = 122,880,000 Hz
            CPU = 61,440,000 Hz
            I2S Clock Src = PLL
            
            MCLK= 12,288,000 Hz (48000 * 256)
            Sample Rate = 48000 Hz
            
        Clock config (I2S Slave Mode):    
            PLL = 144,000,000 Hz
            CPU = 72,000,000 Hz
            I2S Clock Src = PLL
            
            MCLK = 12,000,000 Hz
            Sample Rate = (Based on Codec. 48000 Hz)
            
        USB Clock Src = HIRC48 (48MHz)
            
    */


    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

    // HIRC trim
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);
    
    /* Init I2C0 to access WAU8822 */
    I2C0_Init();

    /* Initialize WAU8822 codec */
    WAU8822_Setup();
    
    SPII2S_Open(SPI0, g_u32MasterSlave, PLAY_RATE, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
    /* SPII2S driver will overwrite SPI clock source setting. Just re-set it here */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PLL, 0);
    
    /* Set MCLK and enable MCLK */
    if(g_u32Master)
        SPII2S_EnableMCLK(SPI0, 12288000);
    else
        SPII2S_EnableMCLK(SPI0, 12000000);
    
    /* I2S clock divider is set again to fix BCLK/MCLK phase delay. 
       Wrong phase delay may cause noise in codec.
    */
    {
        uint32_t u32Reg;
        
        u32Reg = SPI0->I2SCLK;
        SPI0->I2SCLK = 0;
        __NOP();
        __NOP();
        __NOP();
        SPI0->I2SCLK = u32Reg;
    }
        

    /* Fill dummy data to I2S Tx for start I2S iteration */
    for(i = 0; i < 4; i++)
        SPII2S_WRITE_TX_FIFO(SPI0, 0);

    /* Start I2S play iteration */
    SPII2S_EnableInt(SPI0, SPII2S_FIFO_TXTH_INT_MASK | SPII2S_FIFO_RXTH_INT_MASK);

    USBD_Open(&gsInfo, UAC_ClassRequest, (SET_INTERFACE_REQ)UAC_SetInterface);
    /* Endpoint configuration */
    UAC_Init();
    USBD_Start();
    
    /* Backup init trim */
    u32TrimInit = M32(TRIM_INIT);
    
    /* Waiting for USB bus stable */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
    while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);
    
    /* Enable USB crystal-less */
    SYS->IRCTCTL1 = HIRC48_AUTO_TRIM;
    
    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(SPI0_IRQn);

    while(1)
    {
        uint8_t ch;
        uint32_t u32Reg, u32Data;
        extern int32_t kbhit(void);

        /* Re-start crystal-less when any error found */
        if(SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk))
        {
            /* USB clock trim fail. Just retry */
            SYS->IRCTCTL1 = 0;  /* Disable crystal-less */
            SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk;
            
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;
            
            /* Waiting for USB bus stable */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
            while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);
            
            SYS->IRCTCTL1 = HIRC48_AUTO_TRIM; /* Re-enable crystal-less */
            printf("USB trim fail. Just retry.\n");
        }
        
        
        /* Adjust codec sampling rate to synch with USB. The adjustment range is +-0.005% */
        AdjFreq();

        /* Set audio volume according USB volume control settings */
        VolumnControl();

        /* User can change audio codec settings by I2C at run-time if necessary */
        if(!kbhit())
        {
            ch = getchar();
            if(ch == 'a')
            {
                if((SPI0->I2SCLK & 0x3f) == 5)
                    SPI0->I2SCLK = (SPI0->I2SCLK & (~0x3f)) | 6;
                else
                    SPI0->I2SCLK = (SPI0->I2SCLK & (~0x3f)) | 5;
            }
            else
            {
            
            
            printf("\nEnter codec setting:\n");
            // Get Register number
            ch = getchar();
            u32Reg = ch - '0';
            ch = getchar();
            u32Reg = u32Reg * 10 + (ch - '0');
            printf("%d\n", u32Reg);

            // Get data
            ch = getchar();
            u32Data = (ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10;
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            printf("%03x\n", u32Data);
            I2C_WriteWAU8822(u32Reg,  u32Data);
            }
        }

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

