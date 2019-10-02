/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"

#define HIRC48_AUTO_TRIM    0x412   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x118)

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 48MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRC48EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRC48STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC48;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Use HIRC48 as USB clock source */
    CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_USBDSEL_Msk)) | CLK_CLKSEL3_USBDSEL_HIRC48;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_USBDIV_Msk)) | CLK_CLKDIV0_USB(1);

    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
}

void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TrimInit;

    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init system and multi-function I/O */
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    /* Get APROM size, data flash size and address */
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    while(DetectPin == 0)
    {
        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);
        /*Init Endpoint configuration for HID */
        HID_Init();
        /* Start USB device */
        USBD_Start();

        /* Backup default trim */
        u32TrimInit = M32(TRIM_INIT);
        /* Clear SOF */
        USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

        /* DO NOT Enable USB device interrupt */
        // NVIC_EnableIRQ(USBD_IRQn);

        while(DetectPin == 0)
        {
            /* Start USB trim if it is not enabled. */
            if((SYS->IRCTCTL1 & SYS_IRCTCTL1_FREQSEL_Msk) != 2)
            {
                /* Start USB trim only when SOF */
                if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
                {
                    /* Clear SOF */
                    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                    /* Re-enable crystal-less */
                    SYS->IRCTCTL1 = HIRC48_AUTO_TRIM;
                }
            }

            /* Disable USB Trim when error */
            if(SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk))
            {
                /* Init TRIM */
                M32(TRIM_INIT) = u32TrimInit;

                /* Disable crystal-less */
                SYS->IRCTCTL1 = 0;

                /* Clear error flags */
                SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk;

                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
            }

            // polling USBD interrupt flag
            USBD_IRQHandler();

            if(bUsbDataReady == TRUE)
            {
                ParseCmd((uint8_t *)usb_rcvbuf, 64);
                EP2_Handler();
                bUsbDataReady = FALSE;
            }
        }
    }

    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);//clear bit
    FMC->ISPCTL &=  ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while(1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
