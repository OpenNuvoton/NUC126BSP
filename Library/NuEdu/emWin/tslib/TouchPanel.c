
#include "NUC126.h"

#include "TouchPanel.h"

static volatile    uint32_t    g_u32AdcIntFlag_TP;

void ADC_IRQHandler(void)
{
    /* Clear the A/D ADINT1 interrupt flag */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    g_u32AdcIntFlag_TP = 1;

}

int Init_TouchPanel(void)
{
    /* Enable peripheral clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(ADC_MODULE, 0, CLK_CLKDIV0_ADC(8));

    return 1;
}
#if 0
static void _BubbleSort(uint16_t arr[], int len)
{
    int i, j, temp;
    for(i = 0; i < len - 1; ++i)
        for(j = 0; j < len - 1 - i; ++j)
            if(arr[j] > arr[j + 1])
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
}
#endif
static uint16_t _Api_Get_TP_X(void)
{
    uint32_t x_adc_in;
//    uint16_t nNormalizationFactor;
//    static uint16_t nXBorderMin = Init_MinBorderCaliValue_X;
//    static uint16_t nXBorderMax = Init_MaxBorderCaliValue_X;
//    static uint8_t nMaxBorderCaliCnt = 0;
//    static uint8_t nMinBorderCaliCnt = 0;
//    static uint16_t nMaxBorderCaliBuffer[5] = {0};
//    static uint16_t nMinBorderCaliBuffer[5] = {0};
    /* Init ADC for TP */
    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end and enable the A/D converter */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT6);

    /*=== Get X from ADC input ===*/
    GPIO_SetMode(PE, BIT2, GPIO_MODE_OUTPUT);   // XR
    GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);   // YD
    GPIO_SetMode(PB, BIT10, GPIO_MODE_OUTPUT);  // XL
    PE2 = 1; //XR High
    PB10 = 0; //XL Low

    /* Configure the GPB9 ADC analog input pins.  */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE2MFP_Msk);    // Disable ADC CH9
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB9MFP_Msk);    // Enable ADC CH6
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB9MFP_ADC0_CH6;  //YU sample

    /* Disable the GPB8 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT9);            //YU

    /* Enable the sample module 1 interrupt.  */
    ADC_EnableInt(ADC, ADC_ADF_INT);                //Enable sample module A/D ADINT1 interrupt.
    NVIC_EnableIRQ(ADC_IRQn);

    /* Clear the A/D ADINT1 interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Reset the ADC interrupt indicator and trigger sample module 1 to start A/D conversion */
    g_u32AdcIntFlag_TP = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC interrupt (g_u32AdcIntFlag_TP will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag_TP == 0);
    x_adc_in = ADC_GET_CONVERSION_DATA(ADC, 6);

    ADC_Close(ADC);
#if 0
    //Normalization
    if(x_adc_in <= ReportThreshold)
    {
        //Dymanic Border Calibration
        if(x_adc_in > nXBorderMax)
        {
            nMaxBorderCaliBuffer[nMaxBorderCaliCnt] = x_adc_in;
            nMaxBorderCaliCnt++;
            if(nMaxBorderCaliCnt == 5)
            {
                _BubbleSort(nMaxBorderCaliBuffer, 5);
                nMaxBorderCaliCnt = 0;
                nXBorderMax = (nMaxBorderCaliBuffer[0] + nMaxBorderCaliBuffer[1]) / 2;
            }
        }
        if(x_adc_in < nXBorderMin)
        {
            nMinBorderCaliBuffer[nMinBorderCaliCnt] = x_adc_in;
            nMinBorderCaliCnt++;
            if(nMinBorderCaliCnt == 5)
            {
                _BubbleSort(nMinBorderCaliBuffer, 5);
                nMinBorderCaliCnt = 0;
                nXBorderMin = (nMinBorderCaliBuffer[3] + nMinBorderCaliBuffer[4]) / 2;
            }
        }
        x_adc_in -= nXBorderMin;
        nNormalizationFactor = (nXBorderMax - nXBorderMin) * 1000 / LCD_Resolution_X;
        x_adc_in *= 1000;
        x_adc_in /= nNormalizationFactor;
    }
#endif
    return x_adc_in;
}

/*-----------------------------------------------*/
// Get Y Position from Touch Panel (ADC input)
//
/*-----------------------------------------------*/
static uint16_t _Api_Get_TP_Y(void)
{
    uint32_t y_adc_in;
//    uint16_t nNormalizationFactor;
//    static uint16_t nYBorderMin = Init_MinBorderCaliValue_Y;
//    static uint16_t nYBorderMax = Init_MaxBorderCaliValue_Y;
//    static uint8_t nMaxBorderCaliCnt = 0;
//    static uint8_t nMinBorderCaliCnt = 0;
//    static uint16_t nMaxBorderCaliBuffer[5] = {0};
//    static uint16_t nMinBorderCaliBuffer[5] = {0};
    /* Init ADC for TP */
    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end and enable the A/D converter */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT9);

    /*=== Get Y from ADC input ===*/
    GPIO_SetMode(PB, BIT9, GPIO_MODE_OUTPUT);   // YU
    GPIO_SetMode(PB, BIT11, GPIO_MODE_OUTPUT);  // YD
    GPIO_SetMode(PB, BIT10, GPIO_MODE_INPUT);   // XL
    PB9 = 1;
    PB11 = 0;

    /* Configure the GPE2 ADC analog input pins.  */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB9MFP_Msk);    // Disable ADC CH6
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE2MFP_Msk);    // Enable ADC CH9
    SYS->GPE_MFPL |= SYS_GPE_MFPL_PE2MFP_ADC0_CH9;  //XR

    /* Disable the GPB9 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PE, BIT2);            //XR

    /* Enable the sample module 1 interrupt.  */
    ADC_EnableInt(ADC, ADC_ADF_INT);    //Enable sample module A/D ADINT1 interrupt.
    NVIC_EnableIRQ(ADC_IRQn);

    /* Clear the A/D ADINT1 interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Reset the ADC interrupt indicator and trigger sample module 2 to start A/D conversion */
    g_u32AdcIntFlag_TP = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC interrupt (g_u32AdcIntFlag_TP will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag_TP == 0);
    y_adc_in = ADC_GET_CONVERSION_DATA(ADC, 9);
    ADC_Close(ADC);
#if 0
    /*=== Calculate the Y ===*/
    if(y_adc_in <= ReportThreshold)
    {
        //Dymanic Border Calibration
        if(y_adc_in > nYBorderMax)
        {
            nMaxBorderCaliBuffer[nMaxBorderCaliCnt] = y_adc_in;
            nMaxBorderCaliCnt++;
            if(nMaxBorderCaliCnt == 5)
            {
                _BubbleSort(nMaxBorderCaliBuffer, 5);
                nMaxBorderCaliCnt = 0;
                nYBorderMax = (nMaxBorderCaliBuffer[0] + nMaxBorderCaliBuffer[1]) / 2;
            }
        }
        if(y_adc_in < nYBorderMin)
        {
            nMinBorderCaliBuffer[nMinBorderCaliCnt] = y_adc_in;
            nMinBorderCaliCnt++;
            if(nMinBorderCaliCnt == 5)
            {
                _BubbleSort(nMinBorderCaliBuffer, 5);
                nMinBorderCaliCnt = 0;
                nYBorderMin = (nMinBorderCaliBuffer[3] + nMinBorderCaliBuffer[4]) / 2;
            }
        }
        y_adc_in -= nYBorderMin;
        nNormalizationFactor = (nYBorderMax - nYBorderMin) * 1000 / LCD_Resolution_Y;
        y_adc_in *= 1000;
        y_adc_in /= nNormalizationFactor;
    }
#endif
    return y_adc_in;
}

int Read_TouchPanel(int *x, int *y)
{
    *x = _Api_Get_TP_X();
    *y = _Api_Get_TP_Y();
    if(((*x & 0x0F00) == 0x0F00) || ((*y & 0x0F00) == 0x0F00))
        return 0;
    else
        return 1;
}

int Uninit_TouchPanel(void)
{
    return 1;
}

int Check_TouchPanel(void)
{
    return 0;   //Pen up;
}

