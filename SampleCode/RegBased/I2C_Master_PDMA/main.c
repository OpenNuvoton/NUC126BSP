/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:26p $
 * @brief
 *           Show a Master how to access Slave using PDMA Tx and PDMA Rx mode (Loopback)
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

#define I2C_PDMA_CH        1
#define I2C_PDMA_TX_LENGTH 100 //Master transmit 3+97=100bytes data length
#define I2C_PDMA_RX_LENGTH 97  //I2C0 will receive 97 bytes data (only data)

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8SlvRxData[3];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[256] = {0};
volatile uint8_t g_au8MstRxData[256] = {0};
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;
volatile int32_t g_u32IsTestOver;
volatile uint8_t g_u8MstTxSLA;
volatile uint16_t g_u16SlvDataLen;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static volatile I2C_FUNC s_I2C0HandlerFn = NULL;
static volatile I2C_FUNC s_I2C1HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA IRQ Interrupt                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 1 has abort error */
        if(PDMA_GET_ABORT_STS() & PDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;
        /* Clear abort flag of channel 1 */
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        /* Check transmission of channel 1 has been transfer done */
        if(PDMA_GET_TD_STS() & PDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 1 */
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C0->STATUS;

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C1->STATUS;

    if(I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C1->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 Slave TRx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8data;

    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u16SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C1);

        if(g_u16SlvDataLen < 2)
        {
            g_au8SlvRxData[g_u16SlvDataLen++] = u8data;
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        else if(g_u16SlvDataLen >= 2)
        {
            g_au8SlvData[slave_buff_addr++] = u8data;
            if(slave_buff_addr == 256)
            {
                slave_buff_addr = 0;
            }
        }

        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C1, g_au8SlvData[slave_buff_addr++]);

        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xB8)
    {
        I2C_SET_DATA(I2C1, g_au8SlvData[slave_buff_addr++]);

        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u16SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u16SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 PDMA Master Tx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted */
    {

    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur data ACK interrupt
        */
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 PDMA Master Rx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                          /* START has been transmitted and prepare SLA+W */
    {
        I2C0->DAT = (g_au8MstTxData[g_u8MstDataLen++] | 0x00);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->DAT = g_au8MstTxData[g_u8MstDataLen++];
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen == 2)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_START(I2C0);
        }
    }
    else if(u32Status == 0x10)                    /* Repeat START has been transmitted and prepare SLA+R */
    {
        g_u8MstDataLen = 0;
        I2C_SET_DATA(I2C0, g_au8MstTxData[0] | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                    /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        g_u8MstTxSLA = 1;                         /* PDMA can start PDMA receive */
    }
    else if(u32Status == 0x50)                    /* DATA has been received and ACK has been returned */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur receive data ACK interrupt
        */
    }
    else if(u32Status == 0x58)                    /* DATA has been received and NACK has been returned */
    {
        I2C_STOP(I2C0);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* I2C Master PDMA Tx Channel Configuration                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_Master_PDMA_Tx_Init(void)
{
    /* PDMA Reset */
    SYS->IPRST0 |= (1 << SYS_IPRST0_PDMARST_Msk);
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk;

    /* Initial PDMA and set PDMA Ch1 */
    PDMA->CHCTL |= (1 << I2C_PDMA_CH);

    /* PDMA Settings */
    PDMA->DSCT[I2C_PDMA_CH].CTL =
        PDMA_OP_BASIC |            // Basic Mode
        PDMA_REQ_SINGLE |          // single transfer type
        PDMA_SAR_INC |             // Source address -> incremented(g_au8MstTxData[n])
        PDMA_DAR_FIX |             // Destination address -> fixed(I2C0_DAT)
        PDMA_WIDTH_8 |             // transfer width -> 8-bit
        ((I2C_PDMA_TX_LENGTH - 1) << 16); // transfer count
    PDMA->DSCT[I2C_PDMA_CH].SA = (uint32_t)g_au8MstTxData;
    PDMA->DSCT[I2C_PDMA_CH].DA = (uint32_t)(&(I2C0->DAT));

    /* Enalbe PDMA I2C0 TX mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_I2C0_TX << PDMA_REQSEL0_3_REQSRC1_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/* I2C Master PDMA Rx Channel Configuration                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_Master_PDMA_Rx_Init(void)
{
    /* PDMA Reset */
    SYS->IPRST0 |= (1 << SYS_IPRST0_PDMARST_Msk);
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk;

    /* Initial PDMA and set PDMA Ch1 */
    PDMA->CHCTL |= (1 << I2C_PDMA_CH);

    /* PDMA Setting */
    PDMA->DSCT[I2C_PDMA_CH].CTL =
        PDMA_OP_BASIC |            // Basic mode
        PDMA_REQ_SINGLE |          // Single transfer type
        PDMA_SAR_FIX  |            // source address -> Fix(I2C0_DAT)
        PDMA_DAR_INC  |            // destination address -> incremented(g_au8MstRxData[n])
        PDMA_WIDTH_8 |             // transfer width -> 8-bit
        ((I2C_PDMA_RX_LENGTH - 1) << 16); // transfer count
    PDMA->DSCT[I2C_PDMA_CH].SA = (uint32_t)(&(I2C0->DAT));
    PDMA->DSCT[I2C_PDMA_CH].DA = (uint32_t)g_au8MstRxData;

    /* Enalbe PDMA I2C0 RX mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_I2C0_RX << PDMA_REQSEL0_3_REQSRC1_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  System Initial                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Enable external HXT 12MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for external HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock and I2C controller */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk | CLK_APBCLK0_I2C1CKEN_Msk);

    /* PDMA Clock Enable */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Select UART module clock source as HXT */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PA multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_I2C0_SDA | SYS_GPA_MFPL_PA3MFP_I2C0_SCL);

    /* Set PC multi-function pins for I2C1 SDA and SCL */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC5MFP_I2C1_SDA | SYS_GPC_MFPL_PC4MFP_I2C1_SCL);

    /* I2C pins enable schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk);
    PC->SMTEN |= (GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void I2C0_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C0 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->CTL |= I2C_CTL_I2CEN_Msk;

    /* I2C0 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C0->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", (SystemCoreClock / (((I2C0->CLKDIV) + 1) << 2)));

    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->ADDR0 = (I2C0->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x15 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->ADDR1 = (I2C0->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x35 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->ADDR2 = (I2C0->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x55 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->ADDR3 = (I2C0->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x75 << I2C_ADDR3_ADDR_Pos);

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C1 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C1RST_Msk;

    /* Enable I2C1 Controller */
    I2C1->CTL |= I2C_CTL_I2CEN_Msk;

    /* I2C1 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C1->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", (SystemCoreClock / (((I2C1->CLKDIV) + 1) << 2)));

    /* Set I2C1 4 Slave Addresses */
    /* Slave Address : 0x16 */
    I2C1->ADDR0 = (I2C1->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x16 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x36 */
    I2C1->ADDR1 = (I2C1->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x36 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x56 */
    I2C1->ADDR2 = (I2C1->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x56 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x76 */
    I2C1->ADDR3 = (I2C1->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x76 << I2C_ADDR3_ADDR_Pos);

    /* Enable I2C1 interrupt and set corresponding NVIC bit */
    I2C1->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C1_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->CTL &= ~I2C_CTL_INTEN_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;
}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C1->CTL &= ~I2C_CTL_INTEN_Msk;
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C1->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C1CKEN_Msk;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i, u32TimeOutCnt;

    uint8_t err = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("\n");
    printf("+----------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code. Show how a [Master] access Slave |\n");
    printf("| use [PDMA Tx] mode and [PDMA Rx] mode                    |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)                  |\n");
    printf("+----------------------------------------------------------+\n");

    printf("\nConfigure I2C0 as a Master, I2C1 as a Slave.\n");
    printf("The I/O connection I2C0 to I2C1\n");
    printf("I2C0_SDA(PA.2), I2C0_SCL(PA.3)\n");
    printf("I2C1_SDA(PC.5), I2C1_SCL(PC.4)\n\n");

    /* Init I2C0, I2C1 */
    I2C0_Init();
    I2C1_Init();

    /* I2C1 enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

    /* Clear Slave receive data buffer */
    for(i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C1 function to Slave receive/transmit data */
    s_I2C1HandlerFn = I2C_SlaveTRx;
    printf("\nI2C1 Slave Mode is Running.\n\n");

    /* Init I2C0 Master PDMA transmit */
    g_u8DeviceAddr = 0x16;

    /* Prepare 100 data bytes */
    g_au8MstTxData[0] = ((g_u8DeviceAddr << 1) | 0x00);   //1 byte SLV + W
    g_au8MstTxData[1] = 0x00;                             //2 bytes Data address
    g_au8MstTxData[2] = 0x00;

    for(i = 3; i < 100; i++)                              //Prepare others are transmit Data from 3 to 99 (97 bytes)
        g_au8MstTxData[i] = i;

    /* Enable PDMA Channel 1 INT */
    PDMA->INTEN |= (1 << I2C_PDMA_CH);
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Init Master PDMA Tx */
    I2C_Master_PDMA_Tx_Init();

    /* Enable I2C0 PDMA TX mode */
    I2C0->CTL1 |= I2C_CTL1_TXPDMAEN_Msk;

    /* Disable PDMA Stretch bus */
    I2C0->CTL1 &= ~I2C_CTL1_PDMASTR_Msk;       //I2C0 will send STOP automatically.

    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterTx;

    printf("Start I2C0 Master transmit data with PDMA...\n");

    /* Send START condition, start the PDMA data transmit */
    I2C_START(I2C0);

    /* Waiting for PDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            goto lexit;
        }
    }
    g_u32IsTestOver = 0;

    /* Waiting for I2C bus become free */
    u32TimeOutCnt = I2C_TIMEOUT;
    while((I2C0->STATUS1 & I2C_STATUS1_ONBUSY_Msk) == I2C_STATUS1_ONBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for I2C bus become free time-out!\n");
            goto lexit;
        }
    }

    /* Disable I2C0 PDMA TX mode */
    I2C0->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;
    printf("I2C0 PDMA transmit data pass...\n\n");

    g_u8MstEndFlag = 0;

    /* Clear Master receive data buffer */
    for(i = 1; i < 200; i++)
        g_au8MstRxData[i] = 0;

    /* Init Master PDMA Rx */
    I2C_Master_PDMA_Rx_Init();

    /* I2C function to read data from slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterRx;

    g_u8MstDataLen = 0;
    g_u8MstTxSLA = 0;
    g_u8MstDataLen = 0;

    printf("Start I2C0 Master receive data with PDMA...\n");

    /* Send START condition, start the PDMA data receive */
    I2C_START(I2C0);
    u32TimeOutCnt = I2C_TIMEOUT;
    while(g_u8MstTxSLA == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for I2C time-out!\n");
            goto lexit;
        }
    }

    /* Enable I2C0 PDMA RX after Slave address read ACK */
    I2C0->CTL1 |= I2C_CTL1_RXPDMAEN_Msk;      //Enalbe PDMA RX, Start receive data from Slave

    /* Waiting for PDMA receive done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA receive done time-out!\n");
            goto lexit;
        }
    }

    /* Disable I2C0 PDMA RX */
    I2C0->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;

    /* PDMA Transfer Done */
    g_u32IsTestOver = 0;

    /* Check Receive data ending */
    u32TimeOutCnt = I2C_TIMEOUT;
    while(g_u8MstEndFlag == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for I2C Rx time-out!\n");
            goto lexit;
        }
    }

    /* Compare I2C0 transmit data and I2C0 receive data */
    for(i = 0; i < 97; i++)
    {
        if(g_au8MstRxData[i] != g_au8MstTxData[i + 3])
        {
            /* Compare fail */
            err = 1;
            printf("[%03d]: Master Tx[0x%X] != Master Rx[0x%X]\n", i,  g_au8MstTxData[i + 3], g_au8MstRxData[i]);
        }
    }

    if(err)
        printf("I2C0 PDMA receive data fail...\n");
    else
        printf("I2C0 PDMA Tx/Rx data pass...\n");

lexit:

    while(1);
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

