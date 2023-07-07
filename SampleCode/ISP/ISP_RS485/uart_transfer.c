/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "targetdev.h"
#include "uart_transfer.h"

__attribute__((aligned(4))) uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/* please check "targetdev.h" for chip specifc define option */

/*---------------------------------------------------------------------------------------------------------*/
/* INTSTS to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART_T_IRQHandler(void)
{
    /* Determine interrupt source */
    uint32_t u32IntSrc = UART_T->INTSTS;

    /* RDA FIFO interrupt and RDA timeout interrupt */    
    if (u32IntSrc & (UART_INTSTS_RXTOIF_Msk|UART_INTSTS_RDAIF_Msk)) { 
        
        /* Read data until RX FIFO is empty or data is over maximum packet size */
        while (((UART_T->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE)) {	
            uart_rcvbuf[bufhead++] = UART_T->DAT;
        }
    }

    /* Reset data buffer index */      
    if (bufhead == MAX_PKT_SIZE) {
        bUartDataReady = TRUE;
        bufhead = 0;
    } else if (u32IntSrc & UART_INTSTS_RXTOIF_Msk) {
        bufhead = 0;
    }
}

extern __attribute__((aligned(4))) uint8_t response_buff[64];
void PutString(void)
{
    uint32_t i;

    /* UART send response to master */    
    for (i = 0; i < MAX_PKT_SIZE; i++) {
        
        /* Wait for TX not full */        
        while ((UART_T->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));

        /* UART send data */         
        UART_T->DAT = response_buff[i];
    }
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select UART function mode */
    UART_T->FUNCSEL = UART_FUNCSEL_UART;
    /* Set UART line configuration */
    UART_T->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* Set UART Rx and RTS trigger level */
    UART_T->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;
    /* Set UART baud rate */
    UART_T->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, 115200));
    /* Set time-out interrupt comparaTOUT */
    UART_T->TOUT = (UART_T->TOUT & ~UART_TOUT_TOIC_Msk) | (0x40);
    NVIC_SetPriority(UART_T_IRQn, 2);
    NVIC_EnableIRQ(UART_T_IRQn);
    /* Enable tim-out counter, Rx tim-out interrupt and Rx ready interrupt */
    UART_T->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
