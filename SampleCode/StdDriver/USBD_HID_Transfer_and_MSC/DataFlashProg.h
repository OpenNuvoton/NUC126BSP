/******************************************************************************
 * @file     DataFlashProg.h
 * @brief    Data flash programming driver header
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#define MASS_STORAGE_OFFSET       0x00008000  /* To avoid the code to write APROM */
#define DATA_FLASH_STORAGE_SIZE   (64*1024)  /* Configure the DATA FLASH storage size. To pass USB-IF MSC Test, it needs > 64KB */
#define FLASH_PAGE_SIZE           2048
#define BUFFER_PAGE_SIZE          2048


#endif  /* __DATA_FLASH_PROG_H__ */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
