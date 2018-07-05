#ifndef __NUC126TOUCHPANEL_H__
#define __NUC126TOUCHPANEL_H__

#define __DEMO_TSFILE_ADDR__    0x00020000 /* SPI flash 128KB address */

#define __DEMO_TS_WIDTH__       160
#define __DEMO_TS_HEIGHT__      128

int Init_TouchPanel(void);
int Read_TouchPanel(int *x, int *y);
int Uninit_TouchPanel(void);
int Check_TouchPanel(void);
#endif
