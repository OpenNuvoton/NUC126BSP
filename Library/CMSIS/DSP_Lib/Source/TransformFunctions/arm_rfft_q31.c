/* ----------------------------------------------------------------------    
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.    
*    
* $Date: 16/10/14 6:02p $Revision: 	V.1.4.5  
*    
* Project: 	    CMSIS DSP Library    
* Title:	    arm_rfft_q31.c    
*    
* Description:	RFFT & RIFFT Q31 process function    
*    
*    
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the 
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.    
* -------------------------------------------------------------------- */

#include "arm_math.h"

/*--------------------------------------------------------------------    
*		Internal functions prototypes    
--------------------------------------------------------------------*/

void arm_split_rfft_q31(
    q31_t * pSrc,
    uint32_t fftLen,
    q31_t * pATable,
    q31_t * pBTable,
    q31_t * pDst,
    uint32_t modifier);

void arm_split_rifft_q31(
    q31_t * pSrc,
    uint32_t fftLen,
    q31_t * pATable,
    q31_t * pBTable,
    q31_t * pDst,
    uint32_t modifier);

/**    
* @addtogroup RealFFT    
* @{    
*/

/**    
* @brief Processing function for the Q31 RFFT/RIFFT.   
* @param[in]  *S    points to an instance of the Q31 RFFT/RIFFT structure.   
* @param[in]  *pSrc points to the input buffer.   
* @param[out] *pDst points to the output buffer.   
* @return none.   
*    
* \par Input an output formats:   
* \par    
* Internally input is downscaled by 2 for every stage to avoid saturations inside CFFT/CIFFT process.   
* Hence the output format is different for different RFFT sizes.    
* The input and output formats for different RFFT sizes and number of bits to upscale are mentioned in the tables below for RFFT and RIFFT:   
* \par    
* \image html RFFTQ31.gif "Input and Output Formats for Q31 RFFT"    
*    
* \par    
* \image html RIFFTQ31.gif "Input and Output Formats for Q31 RIFFT"    
*/
void arm_rfft_q31(
    const arm_rfft_instance_q31 * S,
    q31_t * pSrc,
    q31_t * pDst)
{
    const arm_cfft_instance_q31 *S_CFFT = S->pCfft;
    uint32_t i;
    uint32_t L2 = S->fftLenReal >> 1;

    /* Calculation of RIFFT of input */
    if(S->ifftFlagR == 1u)
    {
        /*  Real IFFT core process */
        arm_split_rifft_q31(pSrc, L2, S->pTwiddleAReal,
                            S->pTwiddleBReal, pDst, S->twidCoefRModifier);
        
        /* Complex IFFT process */
        arm_cfft_q31(S_CFFT, pDst, S->ifftFlagR, S->bitReverseFlagR);
        
        for(i=0;i<S->fftLenReal;i++)
        {
            pDst[i] = pDst[i] << 1;
        }
    }
    else
    {
        /* Calculation of RFFT of input */
        
        /* Complex FFT process */
        arm_cfft_q31(S_CFFT, pSrc, S->ifftFlagR, S->bitReverseFlagR);

        /*  Real FFT core process */
        arm_split_rfft_q31(pSrc, L2, S->pTwiddleAReal,
                            S->pTwiddleBReal, pDst, S->twidCoefRModifier);
    }
}

/**    
* @} end of RealFFT group    
*/

/**    
* @brief  Core Real FFT process    
* @param[in]   *pSrc 				points to the input buffer.    
* @param[in]   fftLen  			    length of FFT.   
* @param[in]   *pATable 			points to the twiddle Coef A buffer.    
* @param[in]   *pBTable 			points to the twiddle Coef B buffer.    
* @param[out]  *pDst 				points to the output buffer.    
* @param[in]   modifier 	        twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.   
* @return none.    
*/
void arm_split_rfft_q31(
    q31_t * pSrc,
    uint32_t fftLen,
    q31_t * pATable,
    q31_t * pBTable,
    q31_t * pDst,
    uint32_t modifier)
{
    uint32_t i;                                    /* Loop Counter */
    q31_t outR, outI;                              /* Temporary variables for output */
    q31_t *pCoefA, *pCoefB;                        /* Temporary pointers for twiddle factors */
    q31_t CoefA1, CoefA2, CoefB1;                  /* Temporary variables for twiddle coefficients */
    q31_t *pOut1 = &pDst[2], *pOut2 = &pDst[(4u * fftLen) - 1u];
    q31_t *pIn1 = &pSrc[2], *pIn2 = &pSrc[(2u * fftLen) - 1u];

    /* Init coefficient pointers */
    pCoefA = &pATable[modifier * 2u];
    pCoefB = &pBTable[modifier * 2u];

    i = fftLen - 1u;

    while(i > 0u)
    {
        /*    
        outR = (pSrc[2 * i] * pATable[2 * i] - pSrc[2 * i + 1] * pATable[2 * i + 1]    
        + pSrc[2 * n - 2 * i] * pBTable[2 * i] +    
        pSrc[2 * n - 2 * i + 1] * pBTable[2 * i + 1]);    
        */

        /* outI = (pIn[2 * i + 1] * pATable[2 * i] + pIn[2 * i] * pATable[2 * i + 1] +    
        pIn[2 * n - 2 * i] * pBTable[2 * i + 1] -    
        pIn[2 * n - 2 * i + 1] * pBTable[2 * i]); */

        CoefA1 = *pCoefA++;
        CoefA2 = *pCoefA;

        /* outR = (pSrc[2 * i] * pATable[2 * i] */    
        mult_32x32_keep32_R(outR, *pIn1, CoefA1);

        /* outI = pIn[2 * i] * pATable[2 * i + 1] */
        mult_32x32_keep32_R(outI, *pIn1++, CoefA2);

        /* - pSrc[2 * i + 1] * pATable[2 * i + 1] */
        multSub_32x32_keep32_R(outR, *pIn1, CoefA2);

        /* (pIn[2 * i + 1] * pATable[2 * i] */
        multAcc_32x32_keep32_R(outI, *pIn1++, CoefA1);

        /* pSrc[2 * n - 2 * i] * pBTable[2 * i]  */
        multSub_32x32_keep32_R(outR, *pIn2, CoefA2);
        CoefB1 = *pCoefB;

        /* pIn[2 * n - 2 * i] * pBTable[2 * i + 1] */
        multSub_32x32_keep32_R(outI, *pIn2--, CoefB1);

        /* pSrc[2 * n - 2 * i + 1] * pBTable[2 * i + 1] */
        multAcc_32x32_keep32_R(outR, *pIn2, CoefB1);

        /* pIn[2 * n - 2 * i + 1] * pBTable[2 * i] */
        multSub_32x32_keep32_R(outI, *pIn2--, CoefA2);

        /* write output */
        *pOut1++ = outR;
        *pOut1++ = outI;

        /* write complex conjugate output */
        *pOut2-- = -outI;
        *pOut2-- = outR;

        /* update coefficient pointer */
        pCoefB = pCoefB + (modifier * 2u);
        pCoefA = pCoefA + ((modifier * 2u) - 1u);

        i--;
    }
    pDst[2u * fftLen] = (pSrc[0] - pSrc[1]) >> 1;
    pDst[(2u * fftLen) + 1u] = 0;

    pDst[0] = (pSrc[0] + pSrc[1]) >> 1;
    pDst[1] = 0;
}

/**    
* @brief  Core Real IFFT process    
* @param[in]   *pSrc 				points to the input buffer.   
* @param[in]   fftLen  			    length of FFT.    
* @param[in]   *pATable 			points to the twiddle Coef A buffer.   
* @param[in]   *pBTable 			points to the twiddle Coef B buffer.    
* @param[out]  *pDst 				points to the output buffer.   
* @param[in]   modifier 	        twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.   
* @return none.    
*/
void arm_split_rifft_q31(
    q31_t * pSrc,
    uint32_t fftLen,
    q31_t * pATable,
    q31_t * pBTable,
    q31_t * pDst,
    uint32_t modifier)
{
    q31_t outR, outI;                              /* Temporary variables for output */
    q31_t *pCoefA, *pCoefB;                        /* Temporary pointers for twiddle factors */
    q31_t CoefA1, CoefA2, CoefB1;                  /* Temporary variables for twiddle coefficients */
    q31_t *pIn1 = &pSrc[0], *pIn2 = &pSrc[(2u * fftLen) + 1u];

    pCoefA = &pATable[0];
    pCoefB = &pBTable[0];

    while(fftLen > 0u)
    {
        /*    
        outR = (pIn[2 * i] * pATable[2 * i] + pIn[2 * i + 1] * pATable[2 * i + 1] +    
        pIn[2 * n - 2 * i] * pBTable[2 * i] -    
        pIn[2 * n - 2 * i + 1] * pBTable[2 * i + 1]);    

        outI = (pIn[2 * i + 1] * pATable[2 * i] - pIn[2 * i] * pATable[2 * i + 1] -    
        pIn[2 * n - 2 * i] * pBTable[2 * i + 1] -    
        pIn[2 * n - 2 * i + 1] * pBTable[2 * i]);   
        */
        CoefA1 = *pCoefA++;
        CoefA2 = *pCoefA;

        /* outR = (pIn[2 * i] * pATable[2 * i] */
        mult_32x32_keep32_R(outR, *pIn1, CoefA1);

        /* - pIn[2 * i] * pATable[2 * i + 1] */
        mult_32x32_keep32_R(outI, *pIn1++, -CoefA2);
        
        /* pIn[2 * i + 1] * pATable[2 * i + 1] */
        multAcc_32x32_keep32_R(outR, *pIn1, CoefA2);

        /* pIn[2 * i + 1] * pATable[2 * i] */
        multAcc_32x32_keep32_R(outI, *pIn1++, CoefA1);

        /* pIn[2 * n - 2 * i] * pBTable[2 * i] */
        multAcc_32x32_keep32_R(outR, *pIn2, CoefA2);
        CoefB1 = *pCoefB;

        /* pIn[2 * n - 2 * i] * pBTable[2 * i + 1] */
        multSub_32x32_keep32_R(outI, *pIn2--, CoefB1);

        /* pIn[2 * n - 2 * i + 1] * pBTable[2 * i + 1] */
        multAcc_32x32_keep32_R(outR, *pIn2, CoefB1);

        /* pIn[2 * n - 2 * i + 1] * pBTable[2 * i] */
        multAcc_32x32_keep32_R(outI, *pIn2--, CoefA2);

        /* write output */
        *pDst++ = outR;
        *pDst++ = outI;

        /* update coefficient pointer */
        pCoefB = pCoefB + (modifier * 2u);
        pCoefA = pCoefA + ((modifier * 2u) - 1u);

        /* Decrement loop count */
        fftLen--;
    }
}
