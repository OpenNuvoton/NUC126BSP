/* ----------------------------------------------------------------------    
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.    
*    
* $Date: 16/10/14 6:01p $Revision: 	V.1.4.5
*    
* Project: 	    CMSIS DSP Library    
* Title:		arm_negate_q31.c    
*    
* Description:	Negates Q31 vectors.    
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

/**    
 * @ingroup groupMath    
 */

/**    
 * @addtogroup negate    
 * @{    
 */

/**    
 * @brief  Negates the elements of a Q31 vector.    
 * @param[in]  *pSrc points to the input vector    
 * @param[out]  *pDst points to the output vector    
 * @param[in]  blockSize number of samples in the vector    
 * @return none.    
 *    
 * <b>Scaling and Overflow Behavior:</b>    
 * \par    
 * The function uses saturating arithmetic.    
 * The Q31 value -1 (0x80000000) will be saturated to the maximum allowable positive value 0x7FFFFFFF.    
 */

void arm_negate_q31(
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize)
{
  q31_t in;                                      /* Temporary variable */
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0_FAMILY

/* Run the below code for Cortex-M4 and Cortex-M3 */
  q31_t in1, in2, in3, in4;

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = -A */
    /* Negate and then store the results in the destination buffer. */
    in1 = *pSrc++;
    in2 = *pSrc++;
    in3 = *pSrc++;
    in4 = *pSrc++;

    *pDst++ = __QSUB(0, in1);
    *pDst++ = __QSUB(0, in2);
    *pDst++ = __QSUB(0, in3);
    *pDst++ = __QSUB(0, in4);

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0_FAMILY */


  while(blkCnt > 0u)
  {
    /* C = -A */
    /* Negate and then store the result in the destination buffer. */
    in = *pSrc++;
    *pDst++ = (in == INT32_MIN) ? INT32_MAX : -in;

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**    
 * @} end of negate group    
 */
