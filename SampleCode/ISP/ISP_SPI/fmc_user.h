/***************************************************************************//**
 * @file     fmc_user.h
 * @brief    NUC126 series FMC driver header file
 * @version  2.0.0
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef FMC_USER_H
#define FMC_USER_H

#include "targetdev.h"

#define Config0         FMC_CONFIG0_ADDR
#define Config1         FMC_CONFIG1_ADDR

#define ISPGO           0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define _FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */
#define _FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


extern int EraseAP(uint32_t addr_start, uint32_t addr_end);
extern void UpdateConfig(uint32_t *data, uint32_t *res);
extern void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);
extern void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);
extern void GetDataFlashInfo(uint32_t *addr, uint32_t *size);
int FMC_Write_User(uint32_t u32Addr, uint32_t u32Data);
int FMC_Read_User(uint32_t u32Addr, uint32_t *data);
int FMC_Erase_User(uint32_t u32Addr);

#endif

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
