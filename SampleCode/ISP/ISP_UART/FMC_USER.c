/***************************************************************************//**
 * @file     fmc_user.c
 * @version  V1.00 
 * @brief    FMC driver source file

 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "FMC_USER.h"

/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32data  32-bit Data to program
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of TRM.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with SYS_IsRegLocked().
 */
int FMC_Write_User(uint32_t u32Addr, uint32_t u32Data)
{
    unsigned int Reg;
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = 0x1;
    __ISB();
    Reg = FMC->ISPCTL;

    if (Reg & FMC_ISPCTL_ISPFF_Msk) {
        FMC->ISPCTL = Reg;
        return -1;
    }

    return 0;
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note
 *              Please make sure that Register Write-Protection Function has been disabled
 *              before using this function. User can check the status of
 *              Register Write-Protection Function with SYS_IsRegLocked().
 */
int FMC_Read_User(uint32_t u32Addr, uint32_t *data)
{
    unsigned int Reg;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
    __ISB();
    Reg = FMC->ISPCTL;

    if (Reg & FMC_ISPCTL_ISPFF_Msk) {
        FMC->ISPCTL = Reg;
        return -1;
    }

    *data = FMC->ISPDAT;
    return 0;
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 512 bytes.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with SYS_IsRegLocked().
 */
int FMC_Erase_User(uint32_t u32Addr)
{
    unsigned int Reg;
    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = 0x1;
    __ISB();
    Reg = FMC->ISPCTL;

    if (Reg & FMC_ISPCTL_ISPFF_Msk) {
        FMC->ISPCTL = Reg;
        return -1;
    }

    return 0;
}

void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)    // Read data from flash
{
    unsigned int rLoop;

    for (rLoop = addr_start; rLoop < addr_end; rLoop += 4) {
        FMC_Read_User(rLoop, data);
        data++;
    }

    return;
}

void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)  // Write data into flash
{
    unsigned int wLoop;

    for (wLoop = addr_start; wLoop < addr_end; wLoop += 4) {
        FMC_Write_User(wLoop, *data);
        data++;
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
