/*********************************************************************
*                 SEGGER Software GmbH                               *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2018  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.46 - Graphical user interface for embedded applications **
All  Intellectual Property rights in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product. This file may
only be used in accordance with the following terms:

The  software has  been licensed by SEGGER Software GmbH to Nuvoton Technology Corporation
at the address: No. 4, Creation Rd. III, Hsinchu Science Park, Taiwan
for the purposes  of  creating  libraries  for its 
Arm Cortex-M and  Arm9 32-bit microcontrollers, commercialized and distributed by Nuvoton Technology Corporation
under  the terms and conditions  of  an  End  User  
License  Agreement  supplied  with  the libraries.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Software GmbH
Licensed to:              Nuvoton Technology Corporation, No. 4, Creation Rd. III, Hsinchu Science Park, 30077 Hsinchu City, Taiwan
Licensed SEGGER software: emWin
License number:           GUI-00735
License model:            emWin License Agreement, signed February 27, 2018
Licensed platform:        Cortex-M and ARM9 32-bit series microcontroller designed and manufactured by or for Nuvoton Technology Corporation
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2018-03-26 - 2019-03-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include <stddef.h>
#include <stdio.h>

#include "GUI.h"
#include "GUIDRV_FlexColor.h"

#include "NUC126.h"

#include "NUC126TouchPanel.h"

/*********************************************************************
*
*       Layer configuration
*
**********************************************************************
*/
//
// Physical display size
//
#define XSIZE_PHYS 128
#define YSIZE_PHYS 160

//
// Color conversion
//
#define COLOR_CONVERSION GUICC_M565

//
// Display driver
//
#define DISPLAY_DRIVER GUIDRV_FLEXCOLOR

//
// Orientation
//
//#define DISPLAY_ORIENTATION (0)
//#define DISPLAY_ORIENTATION (GUI_MIRROR_X)
//#define DISPLAY_ORIENTATION (GUI_MIRROR_Y)
//#define DISPLAY_ORIENTATION (GUI_MIRROR_X | GUI_MIRROR_Y)
//#define DISPLAY_ORIENTATION (GUI_SWAP_XY)
#define DISPLAY_ORIENTATION (GUI_MIRROR_X | GUI_SWAP_XY)
//#define DISPLAY_ORIENTATION (GUI_MIRROR_Y | GUI_SWAP_XY)
//#define DISPLAY_ORIENTATION (GUI_MIRROR_X | GUI_MIRROR_Y | GUI_SWAP_XY)

//
// Hardware related
//
#define SPI_LCD_PORT  SPI1

#define GPIO_SPI0_SS PD12
#define GPIOPORT_SPI0_SS PD
#define PINMASK_SPI0_SS BIT12

#define GPIO_SPI0_CLK PD15
#define GPIOPORT_SPI0_CLK PD
#define PINMASK_SPI0_CLK BIT15

#define GPIO_SPI0_MISO PD14
#define GPIOPORT_SPI0_MISO PD
#define PINMASK_SPI0_MISO BIT14

#define GPIO_SPI0_MOSI PD13
#define GPIOPORT_SPI0_MOSI PD
#define PINMASK_SPI0_MOSI BIT13

#define GPIO_LCM_DC PD6
#define GPIOPORT_LCM_DC PD
#define PINMASK_LCM_DC BIT6

#define GPIO_LCM_RESET PD1
#define GPIOPORT_LCM_RESET PD
#define PINMASK_LCM_RESET BIT1

#define SPI_CS_SET    GPIO_SPI0_SS = 1
#define SPI_CS_CLR    GPIO_SPI0_SS = 0

#define LCM_DC_SET    GPIO_LCM_DC = 1
#define LCM_DC_CLR    GPIO_LCM_DC = 0

#define LCM_RESET_SET GPIO_LCM_RESET = 1
#define LCM_RESET_CLR GPIO_LCM_RESET = 0

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
  #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
  #define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
  #error Color conversion not defined!
#endif
#ifndef   DISPLAY_DRIVER
  #error No display driver defined!
#endif
#ifndef   DISPLAY_ORIENTATION
  #define DISPLAY_ORIENTATION 0
#endif

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _Read1
*/
static U8 _Read1(void) {
    #if 1
    /* FIXME if panel supports read back feature */
    return 0;
    #else
	LCM_DC_SET;
    SPI_CS_CLR;
	SPI_WRITE_TX(SPI_LCD_PORT, 0x00);
	SPI_READ_RX(SPI_LCD_PORT);
	SPI_CS_SET;
	return (SPI_READ_RX(SPI_LCD_PORT));
    #endif
}

/*********************************************************************
*
*       _ReadM1
*/
static void _ReadM1(U8 * pData, int NumItems) {
    #if 1
    /* FIXME if panel supports read back feature */
    #else
	LCM_DC_SET;
    SPI_CS_CLR;
	while (NumItems--) {
  	SPI_WRITE_TX(SPI_LCD_PORT, 0x00);
  	while(SPI_IS_BUSY(SPI_LCD_PORT));
		*pData++ = SPI_READ_RX(SPI_LCD_PORT);
	}
	SPI_CS_SET;
    #endif
}

/*********************************************************************
*
*       _Write0
*/
static void _Write0(U8 Cmd) {
	LCM_DC_CLR;
    SPI_CS_CLR;
    
    SPI_WRITE_TX(SPI_LCD_PORT, Cmd);
    while(SPI_IS_BUSY(SPI_LCD_PORT));

    SPI_CS_SET;
}

/*********************************************************************
*
*       _Write1
*/
static void _Write1(U8 Data) {
	LCM_DC_SET;
    SPI_CS_CLR;

    SPI_WRITE_TX(SPI_LCD_PORT, Data);

    while(SPI_IS_BUSY(SPI_LCD_PORT));
    SPI_CS_SET;
}

/*********************************************************************
*
*       _WriteM1
*/
static void _WriteM1(U8 * pData, int NumItems) {
	LCM_DC_SET;
    SPI_CS_CLR;
	while (NumItems--) {
        SPI_WRITE_TX(SPI_LCD_PORT, *pData++);
        while(SPI_IS_BUSY(SPI_LCD_PORT));
	}
	SPI_CS_SET;
}

static void _Open_SPI(void)
{
    GPIO_SetMode(GPIOPORT_LCM_DC, PINMASK_LCM_DC, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_LCM_RESET, PINMASK_LCM_RESET, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_SPI0_SS, PINMASK_SPI0_SS, GPIO_MODE_OUTPUT); //cs pin for gpiod

    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD13MFP_SPI1_MOSI | SYS_GPD_MFPH_PD15MFP_SPI1_CLK | SYS_GPD_MFPH_PD14MFP_SPI1_MISO);

    /* Enable SPI0 */
    CLK_EnableModuleClock(SPI1_MODULE);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, 0);

    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 24000000);
    
    /* Disable auto SS function, control SS signal manually. */
    SPI_DisableAutoSS(SPI_LCD_PORT);
    SPI_ENABLE(SPI_LCD_PORT);
}

static void _delay_ms(unsigned int time)
{
    unsigned int i;
    for(i=0; i<time; i++)
        CLK_SysTickDelay(1000);
}

/*********************************************************************
*
*       _InitController
*
* Purpose:
*   Initializes the display controller
*/
static void _InitController(void) {
    int i;

	_Open_SPI();

    LCM_RESET_SET;
    LCM_RESET_CLR;

    _delay_ms(100);

    LCM_RESET_SET;

    _delay_ms(120);
    _Write0(0x11);    //Exit Sleep
    _delay_ms(120);
//------------------------------------------------------------------//
//-------------------Software Reset---------------------------------//

    _Write0(0xB1);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write0(0xB2);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write0(0xB3);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);


    _Write0(0xB4);  //Column inversion
    _Write1(0x07);
    //ST7735R Power Sequence
    _Write0(0xC0);
    _Write1(0xA2);
    _Write1(0x02);
    _Write1(0x84);
    _Write1(0xC1);
    _Write1(0xC5);
    _Write0(0xC2);
    _Write1(0x0A);
    _Write1(0x00);

    _Write0(0xC3);
    _Write1(0x8A);
    _Write1(0x2A);
    _Write1(0xC4);
    _Write1(0x8A);
    _Write1(0xEE);

    _Write0(0xC5); //VCOM
    _Write1(0x0E);

    _Write0(0x36); //MX, MY, RGB mode
    _Write1(0xC0);
    _Write1(0xC8); //??C8 ??08 A8

    //ST7735R Gamma Sequence
    _Write0(0xe0);
    _Write1(0x0f);
    _Write1(0x1a);
    _Write1(0x0f);
    _Write1(0x18);
    _Write1(0x2f);
    _Write1(0x28);
    _Write1(0x20);
    _Write1(0x22);
    _Write1(0x1f);
    _Write1(0x1b);
    _Write1(0x23);
    _Write1(0x37);
    _Write1(0x00);
    _Write1(0x07);
    _Write1(0x02);
    _Write1(0x10);

    _Write0(0xe1);
    _Write1(0x0f);
    _Write1(0x1b);
    _Write1(0x0f);
    _Write1(0x17);
    _Write1(0x33);
    _Write1(0x2c);
    _Write1(0x29);
    _Write1(0x2e);
    _Write1(0x30);
    _Write1(0x30);
    _Write1(0x39);
    _Write1(0x3f);
    _Write1(0x00);
    _Write1(0x07);
    _Write1(0x03);
    _Write1(0x10);

    _Write0(0x2a);
    _Write1(0x02);
    _Write1(0x00+2);
    _Write1(0x02);
    _Write1(0x7F+2);

    _Write0(0x2b);
    _Write1(0x01);
    _Write1(0x00+1);
    _Write1(0x01);
    _Write1(0x9F+1);

    _Write0(0xF0); //Enable test command
    _Write1(0x01);
    _Write0(0xF6); //Disable ram power save mode
    _Write1(0x00);

    _Write0(0x3A); //65k mode
    _Write1(0x05);

    _Write0(0x2c);
    for(i=0; i<0x5000; i++)
    {
        _Write1(0x00>>8);
        _Write1(0x00);
    }

    _Write0(0x29);    //Display on
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/
void LCD_X_Config(void) {
  GUI_DEVICE * pDevice;
  GUI_PORT_API PortAPI = {0};
  CONFIG_FLEXCOLOR Config = {0};

  //
  // Set display driver and color conversion for 1st layer
  //
  pDevice = GUI_DEVICE_CreateAndLink(DISPLAY_DRIVER, COLOR_CONVERSION, 0, 0);
  //
  // Orientation
  //
  Config.Orientation = DISPLAY_ORIENTATION;
  GUIDRV_FlexColor_Config(pDevice, &Config);
  //
  // Display driver configuration
  //
  if (DISPLAY_ORIENTATION & GUI_SWAP_XY) {
    LCD_SetSizeEx (0, YSIZE_PHYS,   XSIZE_PHYS);
    LCD_SetVSizeEx(0, VYSIZE_PHYS,  VXSIZE_PHYS);
  } else {
    LCD_SetSizeEx (0, XSIZE_PHYS,   YSIZE_PHYS);
    LCD_SetVSizeEx(0, VXSIZE_PHYS,  VYSIZE_PHYS);
  }
  //
  // Function selection, hardware routines (PortAPI) and operation mode (bus, bpp and cache)
  //
  PortAPI.pfWrite8_A0  = _Write0;
  PortAPI.pfWrite8_A1  = _Write1;
  PortAPI.pfWriteM8_A0 = _WriteM1;
  PortAPI.pfWriteM8_A1 = _WriteM1;
  PortAPI.pfRead8_A0   = _Read1;    /* FIXME if panel supports read back feature */
  PortAPI.pfRead8_A1   = _Read1;    /* FIXME if panel supports read back feature */
  PortAPI.pfReadM8_A0  = _ReadM1;   /* FIXME if panel supports read back feature */
  PortAPI.pfReadM8_A1  = _ReadM1;   /* FIXME if panel supports read back feature */
  GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M16C0B8);
  

#if GUI_SUPPORT_TOUCH
// LCD calibration
//
// Calibrate touch screen
//
    GUI_TOUCH_Calibrate(GUI_COORD_X, 0, (__DEMO_TS_WIDTH__ - 1), 0, (__DEMO_TS_WIDTH__ - 1));
    GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, (__DEMO_TS_HEIGHT__-  1), 0, (__DEMO_TS_HEIGHT__ - 1));
#endif
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  int r;

  GUI_USE_PARA(LayerIndex);
  GUI_USE_PARA(pData);
  switch (Cmd) {
  //
  // Required
  //
  case LCD_X_INITCONTROLLER: {
    //
    // Called during the initialization process in order to set up the
    // display controller and put it into operation. If the display
    // controller is not initialized by any external routine this needs
    // to be adapted by the customer...
    //
    _InitController();
    return 0;
  }
  default:
    r = -1;
  }
  return r;
}
#if GUI_SUPPORT_TOUCH
extern int ts_phy2log(int *sumx, int *sumy);

void GUI_TOUCH_X_ActivateX(void) {
}

void GUI_TOUCH_X_ActivateY(void) {
}


 
int  GUI_TOUCH_X_MeasureX(void) {
  int sumx;
  int sumy;
	if (Read_TouchPanel(&sumx, &sumy))
	{
//		printf("X = %d\n", sumx);
		ts_phy2log(&sumx, &sumy);		
    return sumx;
	}
	return -1;
}

int  GUI_TOUCH_X_MeasureY(void) {
  int sumx;
  int sumy;
	if ( Read_TouchPanel(&sumx, &sumy) )
	{
//		printf("Y = %d\n", sumy);
		ts_phy2log(&sumx, &sumy);				
    return sumy;
	}
	return -1;
}
#endif
/*************************** End of file ****************************/
