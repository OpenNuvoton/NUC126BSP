/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:33p $
 * @brief    Show a Master how to access 7-bit address Slave (loopback)
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC126.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvRxData[4];
volatile uint16_t g_u16RecvAddr;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8SlvDataLen;

enum UI2C_MASTER_EVENT m_Event;
enum UI2C_SLAVE_EVENT s_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);
static UI2C_FUNC s_UI2C0HandlerFn = NULL;
static UI2C_FUNC s_UI2C1HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_IRQHandler(void)
{
    uint32_t u32Status;

    //UI2C0 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);
    if(s_UI2C0HandlerFn != NULL)
        s_UI2C0HandlerFn(u32Status);

    //UI2C1 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C1);
    if(s_UI2C1HandlerFn != NULL)
        s_UI2C1HandlerFn(u32Status);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 Rx Callback Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_MasterRx(uint32_t u32Status)
{
    if(UI2C_GET_TIMEOUT_FLAG(UI2C0))
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_ClearTimeoutFlag(UI2C0);
    }
    else if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);  /* Clear START INT Flag */

        if(m_Event == MASTER_SEND_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x00); /* Write SLA+W to Register TXDAT */
            m_Event = MASTER_SEND_ADDRESS;
        }
        else if(m_Event == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
            m_Event = MASTER_SEND_H_RD_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */

        if(m_Event == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            m_Event = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if(m_Event == MASTER_SEND_DATA)
        {
            if(g_u8MstDataLen != 2)
            {
                UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                m_Event = MASTER_SEND_REPEAT_START;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send repeat START signal */
            }
        }
        else if(m_Event == MASTER_SEND_H_RD_ADDRESS)
        {
            m_Event = MASTER_READ_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);             /* Clear NACK INT Flag */

        if(m_Event == MASTER_SEND_ADDRESS)
        {
            m_Event = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
        }
        else if(m_Event == MASTER_READ_DATA)
        {
            g_u8MstRxData = (uint8_t) UI2C_GET_DATA(UI2C0);
            g_u8MstEndFlag = 1;
            m_Event = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
            printf("Get Wrong NACK Event\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 Tx Callback Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_MasterTx(uint32_t u32Status)
{
    if((UI2C0->PROTSTS & UI2C_PROTSTS_TOIF_Msk) == UI2C_PROTSTS_TOIF_Msk)
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_TOIF_Msk);
    }
    else if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);               /* Clear START INT Flag */

        UI2C_SET_DATA(UI2C0, (g_u8DeviceAddr << 1) | 0x00);          /* Write SLA+W to Register TXDAT */
        m_Event = MASTER_SEND_ADDRESS;

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);   /* Clear ACK INT Flag */

        if(m_Event == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);   /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            m_Event = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if(m_Event == MASTER_SEND_DATA)
        {
            if(g_u8MstDataLen != 3)
            {
                UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                g_u8MstEndFlag = 1;
                m_Event = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));        /* Send STOP signal */
            }
        }
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);                  /* Clear NACK INT Flag */

        g_u8MstEndFlag = 0;
        if(m_Event == MASTER_SEND_ADDRESS)
        {
            /* SLA+W has been transmitted and NACK has been received */
            m_Event = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));            /* Send START signal */
        }
        else if(m_Event == MASTER_SEND_DATA)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            m_Event = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
        }
        else
            printf("Get Wrong NACK Event\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C1 TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_LB_SlaveTRx(uint32_t u32Status)
{
    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if(s_Event == SLAVE_ADDRESS_ACK)
        {
            g_u8SlvDataLen = 0;

            if((UI2C1->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                s_Event = SLAVE_SEND_DATA;
                UI2C_SET_DATA(UI2C1, g_au8SlvData[slave_buff_addr]);
                slave_buff_addr++;
            }
            else
            {
                s_Event = SLAVE_GET_DATA;
            }
            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if(s_Event == SLAVE_GET_DATA)
        {
            g_au8SlvRxData[g_u8SlvDataLen] = (uint8_t)UI2C_GET_DATA(UI2C1);
            g_u8SlvDataLen++;

            if(g_u8SlvDataLen == 2)
            {
                /* Address has been received; ACK has been returned*/
                slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
            }
            if(g_u8SlvDataLen == 3)
            {
                g_au8SlvData[slave_buff_addr] = g_au8SlvRxData[2];
                g_u8SlvDataLen = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STORIF_Msk);

        g_u8SlvDataLen = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    //CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable IP clock */
    CLK_EnableModuleClock(USCI0_MODULE);
    CLK_EnableModuleClock(USCI1_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

    /* Set PC multi-function pins for UI2C0_SDA(PC.5) and UI2C0_SDA(PC.4) */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC5MFP_USCI0_DAT0 | SYS_GPC_MFPL_PC4MFP_USCI0_CLK);

    /* Set PD multi-function pins for UI2C1_SDA(PD.14) and UI2C1_SDA(PD.15) */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD14MFP_USCI1_DAT0 | SYS_GPD_MFPH_PD15MFP_USCI1_CLK);

    /* I2C pins enable schmitt trigger */
    PC->SMTEN |= (GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk);
    PD->SMTEN |= (GPIO_SMTEN_SMTEN14_Msk | GPIO_SMTEN_SMTEN15_Msk);
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);

    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x35, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */

    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                    /* Slave Address : 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                    /* Slave Address : 0x4 */

    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);

}

void UI2C1_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C1 and set clock to 100k */
    UI2C_Open(UI2C1, u32ClkSpeed);

    /* Get USCI_I2C1 Bus Clock */
    printf("UI2C1 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C1));

    /* Set USCI_I2C1 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C1, 0, 0x16, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x16 */
    UI2C_SetSlaveAddr(UI2C1, 1, 0x36, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x36 */

    /* Set USCI_I2C1 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C1, 0, 0x04);                    /* Slave Address : 0x4 */
    UI2C_SetSlaveAddrMask(UI2C1, 1, 0x02);                    /* Slave Address : 0x2 */

    /* Enable UI2C1 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C1, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);

}

int32_t Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;

    g_u8DeviceAddr = slvaddr;

    for(i = 0; i < 0x100; i++)
    {
        g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
        g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);

        g_u8MstDataLen = 0;
        g_u8MstEndFlag = 0;

        /* USCI_I2C function to write data to slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_MasterTx;

        /* USCI_I2C as master sends START signal */
        m_Event = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Tx Finish */
        while(g_u8MstEndFlag == 0);
        g_u8MstEndFlag = 0;

        /* USCI_I2C function to read data from slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_MasterRx;

        g_u8MstDataLen = 0;
        g_u8DeviceAddr = slvaddr;

        /* USCI_I2C as master sends START signal */
        m_Event = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Rx Finish */
        while(g_u8MstEndFlag == 0);
        g_u8MstEndFlag = 0;

        /* Compare data */
        if(g_u8MstRxData != g_au8MstTxData[2])
        {
            printf("USCI_I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
            return -1;
        }
    }
    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}

int main()
{
    uint32_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Open(UART0, 115200);

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Master access        |\n");
    printf("|  7-bit address Slave (Loopback)                       |\n");
    printf("|  UI2C0(Master)  <----> UI2C1(Slave)                   |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a master, UI2C1 as Slave.\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PC.5), UI2C0_SCL(PC.4)\n");
    printf("UI2C1_SDA(PD.14), UI2C1_SCL(PD.15)\n");

    /* Init USCI_I2C0 and USCI_I2C1 */
    UI2C0_Init(100000);
    UI2C1_Init(100000);

    s_Event = SLAVE_ADDRESS_ACK;

    UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    for(i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_UI2C1HandlerFn = UI2C_LB_SlaveTRx;

    /* Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
    Read_Write_SLAVE(0x16);
    Read_Write_SLAVE(0x36);
    printf("SLAVE Address test OK.\n");
    /* Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    Read_Write_SLAVE(0x16 & ~0x04);
    Read_Write_SLAVE(0x36 & ~0x02);
    printf("SLAVE Address Mask test OK.\n");

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
