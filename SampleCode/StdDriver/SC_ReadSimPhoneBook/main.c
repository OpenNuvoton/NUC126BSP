/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 3 $
 * $Date: 17/05/04 1:54p $
 * @brief    Read the SIM phone book from SC0 port.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC126.h"
#include "sclib.h"


#define PLL_CLOCK       72000000


/* The definition of commands used in this sample code and directory structures could
   be found in GSM 11.11 which is free for download from Internet.
   Different from the command defined in ISO 7816-4, CLS of SIM command is 0xA0,
   So the command defined below starting with 0xA0 */

// Select File
const uint8_t au8SelectMF[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
const uint8_t au8SelectDF_TELECOM[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x7F, 0x10};
const uint8_t au8SelectEF_ADN[] = {0xA0, 0xA4, 0x00, 0x00, 0x02, 0x6F, 0x3A};
//Get Response
uint8_t au8GetResp[] = {0xA0, 0xC0, 0x00, 0x00, 0x00};
//Read Record
uint8_t au8ReadRec[] = {0xA0, 0xB2, 0x01, 0x04, 0x00};
//Verify CHV, CHV = Card Holder Verification information
uint8_t au8VerifyChv[] = {0xA0, 0x20, 0x00, 0x01, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t buf[300];
uint32_t len;

/**
  * @brief  Ask user to input PIN from console
  * @param  None
  * @return None
  * @details Valid input characters (0~9) are echo to console and store in command buffer.
  *         Backspace key can delete previous input digit, ESC key delete all input digits.
  *         Valid PIN length is between 4~8digits. If PIN length is shorter than 8
  *         digits, an Enter key can terminate the input procedure.
  */
void get_pin(void)
{
    int i = 0;
    char c = 0;

    printf("Please input PIN number:");
    while(i < 8)
    {
        c = getchar();
        if(c >= 0x30 && c <= 0x39)      // Valid input characters (0~9)
        {
            au8VerifyChv[5 + i] = c;
            printf("%c", c);
            i++;
        }
        else if(c == 0x7F)    // DEL (Back space)
        {
            i--;
            printf("%c", c);
        }
        else if(c == 0x0D)     // Enter
        {
            if(i >= 4)  //Min CHV length is 4 digits
                break;
        }
        else if(c == 0x1B)    //ESC
        {
            printf("\nPlease input PIN number:");
            i = 0;  // retry
        }
        else
        {
            continue;
        }

    }

    // Fill remaining digits with 0xFF
    for(; i < 8; i++)
    {
        au8VerifyChv[5 + i] = 0xFF;
    }

    printf("\n");

    return;
}

/**
  * @brief  Send verify command to verify CHV1
  * @param  Remaining retry count, valid values are between 3~1
  * @return Unlock SIM card success or not
  * @retval 0 Unlock success
  * @retval -1 Unlock failed
  */
int unlock_sim(uint32_t u32RetryCnt)
{
    while(u32RetryCnt > 0)
    {

        get_pin(); // Ask user input PIN

        if(SCLIB_StartTransmission(1, au8VerifyChv, 13, buf, &len) != SCLIB_SUCCESS)
        {
            printf("Command Verify CHV failed\n");
            break;
        }
        if(buf[0] == 0x90 || buf[1] == 0x00)
        {
            printf("Pass\n");
            return 0;
        }
        else
        {
            u32RetryCnt--;
            printf("Failed, remaining retry count: %d\n", u32RetryCnt);
        }
    }

    printf("Oops, SIM card locked\n");

    return -1;
}

/**
  * @brief  Read phone book and print on console
  * @param  Phone book record number
  * @return None
  */
void read_phoneBook(uint32_t cnt)
{
    int i, j, k;

    /*
        EF_ADN structure looks like below:

        Byte            Description                         M/O Length
        1 to X          Alpha Identifier                    O   X bytes
        X+1             Length of BCD number/SSC contents   M   1 byte
        X+2             TON and NPI                         M   1 byte
        X+3 to X + 12   Dialling Number/SSC String          M   10 bytes
        X+13            Capability/Configuration Identifier M   1 byte
        X+14            Extension1 Record Identifier        M   1 byte
    */
    for(i = 1; i < cnt + 1; i++)
    {
        au8ReadRec[2] = (uint8_t)i;
        if(SCLIB_StartTransmission(1, au8ReadRec, 5, buf, &len) != SCLIB_SUCCESS)
        {
            printf("Command Read Record failed\n");
            break;
        }
        if(buf[0] == 0xFF) // This is an empty entry
            continue;
        printf("\n======== %d ========", i);
        printf("\nName: ");
        for(j = 0; buf[j] != 0xFF; j++)
        {
            printf("%c", buf[j]);
        }
        while(buf[j] == 0xFF)   // Skip reset of the Alpha Identifier bytes
            j++;

        printf("\nNumber: ");
        j += 2; // Skip Length of BCD and TNO/NPI
        for(k = 0; k < 10; k++)
        {
            if((buf[j + k] & 0xf) != 0xF)
                printf("%c", (buf[j + k] & 0xf) + 0x30);
            else
                break;

            if((buf[j + k] >> 4) != 0xF)
                printf("%c", (buf[j + k] >> 4) + 0x30);
            else
                break;
        }
    }
    printf("\n");
    return;
}

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void)
{
    /* Please don't remove any of the function calls below */
    if(SCLIB_CheckCDEvent(0))
        return; // Card insert/remove event occurred, no need to check other event...

    SCLIB_CheckTimeOutEvent(0);
    SCLIB_CheckTxRxEvent(0);
    SCLIB_CheckErrorEvent(0);

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Enable SC0 module clock */
    CLK_EnableModuleClock(SC0_MODULE);

    /* Select SC0 module clock source from HXT divide 3 */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HXT, CLK_CLKDIV1_SC0(3));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PA.0 ~ PA.3 and PB.2 for SC0 interface */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk |
                       SYS_GPA_MFPL_PA1MFP_Msk |
                       SYS_GPA_MFPL_PA2MFP_Msk |
                       SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB2MFP_Msk;
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK |
                      SYS_GPA_MFPL_PA1MFP_SC0_DAT |
                      SYS_GPA_MFPL_PA2MFP_SC0_RST |
                      SYS_GPA_MFPL_PA3MFP_SC0_PWR);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB2MFP_SC0_nCD;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int retval;
    int retry = 0, cnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|    Smart Card Driver Sample Code    |\n");
    printf("+-------------------------------------+\n\n");

    printf("# This sample code will read phone book from SIM card.\n");
    printf("# I/O configuration:\n");
    printf("    SC0CLK (PA.0) <--> card slot clock pin\n");
    printf("    SC0DAT (PA.1) <--> card slot data pin\n");
    printf("    SC0PWR (PA.3) <--> card slot power pin\n");
    printf("    SC0RST (PA.2) <--> card slot reset pin\n");
    printf("    SC0CD  (PB.2) <--> card slot card detect pin\n");
    printf("\n");

    NVIC_EnableIRQ(SC01_IRQn);

    /* Open smartcard interface 0. SC_CD pin state low indicates card insert and SC_PWR pin low raise VCC pin to card */
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);

    /* Wait 'til card insert */
    while(SC_IsCardInserted(SC0) == FALSE);
    /* Activate slot 0 */
    retval = SCLIB_Activate(0, FALSE);

    if(retval != SCLIB_SUCCESS)
    {
        printf("SIM card activate failed\n");
        goto exit;
    }

    /*
        Each SIM card contains a file system, below is a simplified example

                                    MF (Master File)
                                           |
                              -------------+--------------
                              |            |             |
                   EF (Elementary File)    EF   DF (Dedicated File)
                                                         |
                                                ---------+--------
                                                |        |       |
                                                EF       DF      EF
                                                         |
                                                         EF
        Each file has an two byte ID, where the first byte indicates the type of file
        '3F': Master File
        '7F': Dedicated File
        '2F': Elementary File under the Master File
        '6F': Elementary File under a Dedicated File

    */
    // Select master file.
    if(SCLIB_StartTransmission(0, (uint8_t *)au8SelectMF, 7, buf, &len) != SCLIB_SUCCESS)
    {
        printf("Command Select MF failed\n");
        goto exit;
    }

    // If there is no error during transmission, check the response from card
    if(len == 2 && buf[0] == 0x9F)
    {
        // Everything goes fine, SIM card response 0x9F following by the response data length
        au8GetResp[4] = buf[1]; // response data length
        // Issue "get response" command to get the response from SIM card
        if(SCLIB_StartTransmission(0, au8GetResp, 5, buf, &len) != SCLIB_SUCCESS)
        {
            printf("Command Get response failed\n");
            goto exit;
        }
    }
    else
    {
        printf("Unknown response\n");
        goto exit;
    }

    // Response ends with 0x9000 means command success
    if(buf[len - 2] != 0x90 || buf[len - 1] != 0x00)
    {
        printf("Cannot select MF\n");
        goto exit;
    }
    /*
        Response of select MF, DF listed here:
        Byte    Description
        1~2     RFU
        3~4     Total amount of memory of the selected directory which is not
                allocated to any of the DFs or EFs under the selected directory
        5~6     File ID
        7       Type of File
        8~12    RFU
        13      Length of the following data
        14      File characteristics
        15      Number of DFs which are a direct child of the current directory
        16      Number of EFs which are a direct child of the current directory
        17      Number of CHVs, UNBLOCK CHVs and administrative codes
        18      RFU
        19      CHV1 status
                b8 0: secret code not initialized, 1: secret code initialized
                b7~b5 RFU
                b4~b1 Number of false presentations remaining, 0 means blocked
        20      UNBLOCK CHV1 status
        21      CHV2 status
        22      UNBLOCK CHV2 status
        23      RFU
        24~34   Reserved for the administrative management (optional)
    */

    // Read byte 19 listed in above table to check if SIM is locked
    if(buf[18] & 0x80)
    {
        if((retry = (buf[18] & 0xF)) == 0)   //=> Blocked!!
        {
            printf("SIM locked, and unlock retry count exceed\n");
            goto exit;
        }
    }

    // Select Dedicated File DFTELECOM which contains service related information
    if(SCLIB_StartTransmission(0, (uint8_t *)au8SelectDF_TELECOM, 7, buf, &len) != SCLIB_SUCCESS)
    {
        printf("Command Select DF failed\n");
        goto exit;
    }
    // Don't care about the response of au8SelectDF_TELECOM command here as long as there's no error.


    /* Select Elementary File ADN, where ADN stands for "Abbreviated dialling numbers",
       this is the file used to store phone book */
    if(SCLIB_StartTransmission(0, (uint8_t *)au8SelectEF_ADN, 7, buf, &len) != SCLIB_SUCCESS)
    {
        printf("Command Select EF failed\n");
        goto exit;
    }

    if(len == 2 && buf[0] == 0x9F)     // response data length
    {
        // Everything goes fine, SIM card response 0x9F following by the response data length
        au8GetResp[4] = buf[1];
        if(SCLIB_StartTransmission(0, au8GetResp, 5, buf, &len) != SCLIB_SUCCESS)
        {
            printf("Command Get response failed\n");
            goto exit;
        }
    }
    else
    {
        printf("Unknown response\n");
        goto exit;
    }

    /*
        Response of select EF listed here:
        Byte    Description
        1~2     RFU
        3~4     File size
        5~6     File ID
        7       Type of File
        8       RFU
        9~11    Access conditions. 0: ALW, 1: CHV1, 2: CHV2, 3: RFU, 4: ADM...
                Byte 9 b8~b4 for read, seek, b3~b1 for update
                Byte 10 b8~b4 for increase, b3~b1 is RFU
                Byte 11 b8~b4 for rehabilitate, b3~b1 for invalidate
        12      File status
        13      Length of the following data (byte 14 to the end)
        14      Structure of EF
        15      Length of a record
    */

    au8ReadRec[4] = buf[14]; // Phone book record length
    cnt = ((buf[2] << 8) + buf[3]) / buf[14];   // Phone book record number

    // Read or update EF_ADN can be protected by CHV1, so check if CHV1 is enabled
    if((buf[8] & 0x10) == 0x10)    //Protect by CHV1 ?
    {
        if(unlock_sim(retry) < 0)
        {
            printf("Unlock SIM card failed\n");
            goto exit;
        }
    }

    read_phoneBook(cnt);
    printf("Done\n");
exit:
    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
