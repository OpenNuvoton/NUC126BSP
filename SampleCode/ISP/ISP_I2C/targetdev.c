/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x31
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "ISP_USER.h"

// Supports 128K/256K (APROM)
uint32_t GetApromSize()
{
    uint32_t size = 0x20000, data;
    int result;

    do {
        result = FMC_Read_User(size, &data);

        if (result < 0) {
            return size;
        } else {
            size *= 2;
        }
    } while (1);
}

#define CONFIG0_DFEN                0x01
void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    *size = 0;
    FMC_Read_User(Config0, &uData);

    if ((uData & CONFIG0_DFEN) == 0) { //DFEN enable
        FMC_Read_User(Config1, &uData);

        if (uData > g_apromSize || (uData & 0x7FF)) { //avoid config1 value from error
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    } else {
        *addr = g_apromSize;
        *size = 0;
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
