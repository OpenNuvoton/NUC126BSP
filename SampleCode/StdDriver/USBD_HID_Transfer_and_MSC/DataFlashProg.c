/****************************************************************************
 * @file     DataFlashProg.c
 * @brief    NUC126 Series Data Flash Access API
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/


/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "NUC126.h"
#include "DataFlashProg.h"

#if 0
# define dbg     printf
#else
# define dbg(...)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

uint32_t g_u32Tag = (uint32_t) - 1;
uint32_t g_sectorBuf[FLASH_PAGE_SIZE / 4];

#define WRITE_THROUGH       0   //0: write through off. Better performance. 1: write through on. Slower.


/* This is low level read function of USB Mass Storage */
void DataFlashRead(uint32_t addr, uint32_t size, uint32_t buffer)
{
    uint32_t alignAddr, offset, *pu32;
    int32_t i;


    /* Modify the address to MASS_STORAGE_OFFSET */
    addr += MASS_STORAGE_OFFSET;

    /* Get address base on page size alignment */
    alignAddr = addr & (~(FLASH_PAGE_SIZE - 1));

    /* Get the sector offset*/
    offset = (addr & (FLASH_PAGE_SIZE - 1));

    dbg("R[%08x] %x ALIGN[%08x] O %x\n", addr, size, alignAddr, offset);

    pu32 = (uint32_t *)buffer;

#if WRITE_THROUGH
    for(i = 0; i < size / 4; i++)
    {
        /* Read from flash */
        pu32[i] = M32(addr + i * 4);
    }
#else
    for(i = 0; i < size / 4; i++)
    {
        /* Read from cache */
        if(((alignAddr + i * 4) >= g_u32Tag) && ((alignAddr + i * 4) < g_u32Tag + FLASH_PAGE_SIZE))
        {
            offset = (alignAddr + i * 4 - g_u32Tag) / 4;
            pu32[i] = g_sectorBuf[offset];
        }
        else
        {
            /* Read from flash */
            pu32[i] = M32(addr + i * 4);
        }
    }
#endif
}


void FlashCacheFlush(void)
{
    int32_t i;

    if(g_u32Tag == (uint32_t) - 1)
        return;

    dbg("Flush %08x\n", g_u32Tag);

    /* We need to flush out cache before update it */
    FMC->ISPADDR = g_u32Tag;
    FMC->ISPCMD = 0x22;
    FMC->ISPTRG = 1;
    while(FMC->ISPTRG);
    i = 0;
    FMC->ISPCMD = 0x21;
    FMC->ISPDAT = g_sectorBuf[i++];
    do
    {
        FMC->ISPTRG = 1;
        while(FMC->ISPTRG);
        FMC->ISPADDR += 4;
        FMC->ISPDAT = g_sectorBuf[i++];
    }
    while(i <= FLASH_PAGE_SIZE / 4);

    g_u32Tag = (uint32_t) - 1;
}

void DataFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer)
{
    /* This is low level write function of USB Mass Storage */
    int32_t len, i, offset;
    uint32_t *pu32;
    uint32_t alignAddr;

    /* Modify the address to MASS_STORAGE_OFFSET */
    addr += MASS_STORAGE_OFFSET;

    len = (int32_t)size;

    do
    {

        /* Get address base on page size alignment */
        alignAddr = addr & (~(FLASH_PAGE_SIZE - 1));

        /* Get the sector offset*/
        offset = (addr & (FLASH_PAGE_SIZE - 1));

        dbg("W[%08x] %x ALIGN[%08x] O %x\n", addr, size, alignAddr, offset);

        /* check cache buffer */
        if(g_u32Tag != alignAddr)
        {
            if(g_u32Tag != (uint32_t) - 1)
            {
                dbg("Flush: TAG %08x\n", g_u32Tag);

                /* We need to flush out cache before update it */
                FMC->ISPADDR = g_u32Tag;
                FMC->ISPCMD = 0x22;
                FMC->ISPTRG = 1;
                while(FMC->ISPTRG);
                i = 0;
                FMC->ISPCMD = 0x21;
                FMC->ISPDAT = g_sectorBuf[i++];
                do
                {
                    FMC->ISPTRG = 1;
                    while(FMC->ISPTRG);
                    FMC->ISPADDR += 4;
                    FMC->ISPDAT = g_sectorBuf[i++];
                }
                while(i <= FLASH_PAGE_SIZE / 4);

            }


            /* load data to cache buffer */
            i = 0;
            FMC->ISPADDR = alignAddr;
            FMC->ISPCMD = 0;
            do
            {
                FMC->ISPTRG = 1;
                while(FMC->ISPTRG);
                g_sectorBuf[i++] = FMC->ISPDAT;
                FMC->ISPADDR += 4;
            }
            while(i <= FLASH_PAGE_SIZE / 4);
            g_u32Tag = alignAddr;
        }

        /* Update the data */
        pu32 = (uint32_t *)buffer;
        len = FLASH_PAGE_SIZE - offset;
        if(size < len)
            len = size;

        for(i = 0; i < len / 4; i++)
        {
            g_sectorBuf[offset / 4 + i] = pu32[i];
        }

        size -= len;
        addr += len;
        buffer += len;

    }
    while(size > 0);

#if WRITE_THROUGH
    FlashCacheFlush();
#endif
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

