/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2024 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

    .section _image,"ax"
    .global loaderImageBase
    .global loaderImageLimit
    .align 4
loaderImageBase:
    .incbin     "FMC_IAP_LDROM.bin"
loaderImageLimit:
